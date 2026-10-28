#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <gui/tuner.h>
#include <signal_path/signal_path.h>
#include <module.h>
#include <dsp/multirate/rational_resampler.h>
#include <gui/widgets/symbol_diagram.h>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "dect_block.h"

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "dect_decoder",
    /* Description:     */ "DECT (GAP) voice decoder with encryption indicator, port of gr-dect2",
    /* Author:          */ "Based on gr-dect2 by Pavel Yazev",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ -1
};

ConfigManager config;

// The DECT occupied bandwidth is about 1.2 x the symbol rate
#define VFO_BANDWIDTH 1.3824e6

namespace {
    struct CarrierPlan {
        const char* name;
        double firstFreq;      // Hz
        int count;
        bool descending;       // carrier numbers count down in frequency (ETSI numbering for the European band)
        const char* labelPrefix;
    };

    // Europe/Asia/Africa/...: carrier c = 1897.344 MHz - c * 1.728 MHz, c = 0..9
    // US / Canada (DECT 6.0): 1921.536 MHz + n * 1.728 MHz, n = 0..4
    const CarrierPlan PLANS[] = {
        { "Europe / ETSI (1880-1900 MHz)", 1897.344e6, 10, true, "Carrier" },
        { "US / Canada (DECT 6.0)", 1921.536e6, 5, false, "Channel" },
        { "Manual (do not retune)", 0.0, 0, false, "" },
    };
    const int PLAN_COUNT = sizeof(PLANS) / sizeof(PLANS[0]);

    double carrierFreq(int plan, int idx) {
        const CarrierPlan& p = PLANS[plan];
        return p.descending ? (p.firstFreq - idx * dect::CARRIER_SPACING) : (p.firstFreq + idx * dect::CARRIER_SPACING);
    }

    std::string carrierLabel(int plan, int idx) {
        const CarrierPlan& p = PLANS[plan];
        char buf[64];
        int number = p.descending ? idx : (idx + 1);
        snprintf(buf, sizeof(buf), "%s %d  -  %.3f MHz", p.labelPrefix, number, carrierFreq(plan, idx) / 1e6);
        return std::string(buf);
    }
}

class DECTDecoderModule : public ModuleManager::Instance {
public:
    DECTDecoderModule(std::string name) : diag(0.6f, 388) {
        this->name = name;

        // Load config
        config.acquire();
        if (!config.conf.contains(name)) {
            config.conf[name]["plan"] = 0;
            config.conf[name]["carrier"] = 0;
            config.conf[name]["autoSelect"] = true;
            config.conf[name]["muteEncrypted"] = false;
        }
        plan = std::clamp<int>(config.conf[name]["plan"], 0, PLAN_COUNT - 1);
        carrier = config.conf[name]["carrier"];
        autoSelect = config.conf[name]["autoSelect"];
        muteEncrypted = config.conf[name].contains("muteEncrypted") ? (bool)config.conf[name]["muteEncrypted"] : false;
        config.release(true);
        clampCarrier();

        // Initialize VFO: the receive chain wants exactly 4 samples per symbol
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, VFO_BANDWIDTH, dect::SAMPLE_RATE, VFO_BANDWIDTH, VFO_BANDWIDTH, true);
        vfo->setSnapInterval(1000);

        // Initialize DSP
        rx.init(vfo->output);
        rx.decoder.setAutoSelect(autoSelect);
        rx.decoder.setMuteEncrypted(muteEncrypted);
        rx.setSymbolHandler(symbolHandler, this);
        resamp.init(&rx.out, dect::AUDIO_RATE, audioSampRate);

        rx.start();
        resamp.start();

        // Setup audio stream
        srChangeHandler.ctx = this;
        srChangeHandler.handler = sampleRateChangeHandler;
        stream.init(&resamp.out, &srChangeHandler, audioSampRate);
        sigpath::sinkManager.registerStream(name, &stream);
        stream.start();

        gui::menu.registerEntry(name, menuHandler, this, this);
    }

    ~DECTDecoderModule() {
        gui::menu.removeEntry(name);
        stream.stop();
        if (enabled) {
            rx.stop();
            resamp.stop();
            sigpath::vfoManager.deleteVFO(vfo);
            vfo = NULL;
        }
        sigpath::sinkManager.unregisterStream(name);
    }

    void postInit() {}

    void enable() {
        double bw = gui::waterfall.getBandwidth();
        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, std::clamp<double>(0, -bw / 2.0, bw / 2.0), VFO_BANDWIDTH, dect::SAMPLE_RATE, VFO_BANDWIDTH, VFO_BANDWIDTH, true);
        vfo->setSnapInterval(1000);

        rx.setInput(vfo->output);
        rx.start();
        resamp.start();

        enabled = true;
    }

    void disable() {
        rx.stop();
        resamp.stop();
        enabled = false;
        sigpath::vfoManager.deleteVFO(vfo);
        vfo = NULL;
    }

    bool isEnabled() {
        return enabled;
    }

private:
    void clampCarrier() {
        int count = PLANS[plan].count;
        carrier = (count > 0) ? std::clamp<int>(carrier, 0, count - 1) : 0;
    }

    void saveConfig() {
        config.acquire();
        config.conf[name]["plan"] = plan;
        config.conf[name]["carrier"] = carrier;
        config.conf[name]["autoSelect"] = autoSelect;
        config.conf[name]["muteEncrypted"] = muteEncrypted;
        config.release(true);
    }

    void tuneToCarrier() {
        if (PLANS[plan].count <= 0) { return; }
        tuner::tune(tuner::TUNER_MODE_NORMAL, name, carrierFreq(plan, carrier));
    }

    // Absolute frequency the VFO is currently at
    double vfoFrequency() {
        if (!enabled || !vfo) { return 0.0; }
        return gui::waterfall.getCenterFrequency() + vfo->getOffset();
    }

    static void menuHandler(void* ctx) {
        DECTDecoderModule* _this = (DECTDecoderModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if (!_this->enabled) { style::beginDisabled(); }

        // ---- Carrier selection ----
        ImGui::LeftLabel("Band");
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::BeginCombo(CONCAT("##dect_plan_", _this->name), PLANS[_this->plan].name)) {
            for (int i = 0; i < PLAN_COUNT; i++) {
                if (ImGui::Selectable(PLANS[i].name, i == _this->plan)) {
                    _this->plan = i;
                    _this->clampCarrier();
                    _this->saveConfig();
                }
            }
            ImGui::EndCombo();
        }

        if (PLANS[_this->plan].count > 0) {
            ImGui::LeftLabel("Carrier");
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::BeginCombo(CONCAT("##dect_carrier_", _this->name), carrierLabel(_this->plan, _this->carrier).c_str())) {
                for (int i = 0; i < PLANS[_this->plan].count; i++) {
                    if (ImGui::Selectable(carrierLabel(_this->plan, i).c_str(), i == _this->carrier)) {
                        _this->carrier = i;
                        _this->saveConfig();
                        _this->tuneToCarrier();
                    }
                }
                ImGui::EndCombo();
            }

            if (ImGui::Button(CONCAT("Tune to carrier##dect_tune_", _this->name), ImVec2(menuWidth, 0))) {
                _this->tuneToCarrier();
            }
        }

        // ---- Warnings about the source configuration ----
        double srcRate = sigpath::iqFrontEnd.getEffectiveSamplerate();
        double vfoFreq = _this->vfoFrequency();
        ImGui::Text("VFO: %.4f MHz", vfoFreq / 1e6);
        if (srcRate < dect::SAMPLE_RATE) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "Source rate %.3f MSPS is below 4.608 MSPS.", srcRate / 1e6);
            ImGui::TextWrapped("Use 4.608 MSPS or more (9.216 or 18.432 MSPS divide evenly and are cheapest).");
        }
        if (PLANS[_this->plan].count > 0) {
            double nearestOff = 1e12;
            for (int i = 0; i < PLANS[_this->plan].count; i++) {
                double off = vfoFreq - carrierFreq(_this->plan, i);
                if (fabs(off) < fabs(nearestOff)) { nearestOff = off; }
            }
            if (fabs(nearestOff) > 0.4 * dect::CARRIER_SPACING) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "VFO is not on a DECT carrier of this band.");
            }
        }

        // ---- Signal ----
        ImGui::SetNextItemWidth(menuWidth);
        _this->diag.draw();

        uint64_t bursts = _this->rx.burstCount;
        auto now = std::chrono::steady_clock::now();
        if (bursts != _this->lastBurstCount) {
            _this->lastBurstCount = bursts;
            _this->lastBurstTime = now;
        }
        bool receiving = (bursts > 0) && (std::chrono::duration_cast<std::chrono::milliseconds>(now - _this->lastBurstTime).count() < 1000);

        ImGui::TextUnformatted("Status:");
        ImGui::SameLine();
        if (receiving) { ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "DECT signal"); }
        else { ImGui::TextUnformatted("Idle"); }

        // ---- Parts on air ----
        dect::PartInfo parts[dect::MAX_PARTS];
        int selected;
        _this->rx.decoder.getParts(parts, selected);

        ImGui::TextUnformatted("Parts on air (click to listen):");
        if (ImGui::BeginTable(CONCAT("##dect_parts_", _this->name), 5, ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders)) {
            ImGui::TableSetupColumn("#");
            ImGui::TableSetupColumn("ID");
            ImGui::TableSetupColumn("Type");
            ImGui::TableSetupColumn("Voice");
            ImGui::TableSetupColumn("Enc");
            ImGui::TableHeadersRow();

            bool any = false;
            for (int i = 0; i < dect::MAX_PARTS; i++) {
                const dect::PartInfo& p = parts[i];
                if (!p.active) { continue; }
                any = true;

                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                char label[32];
                snprintf(label, sizeof(label), "%s%d##dect_part_%d_%s", (i == selected) ? "> " : "", i, i, _this->name.c_str());
                if (ImGui::Selectable(label, i == selected, ImGuiSelectableFlags_SpanAllColumns)) {
                    _this->rx.decoder.selectPart(i);
                    _this->autoSelect = false;
                    _this->rx.decoder.setAutoSelect(false);
                    _this->saveConfig();
                }

                ImGui::TableSetColumnIndex(1);
                if (p.partIdReceived) {
                    ImGui::Text("%02X%02X%02X%02X%02X", p.partId[0], p.partId[1], p.partId[2], p.partId[3], p.partId[4]);
                }
                else {
                    ImGui::TextUnformatted("--");
                }
                ImGui::TableSetColumnIndex(2);
                ImGui::TextUnformatted(p.type == dect::PART_RFP ? "RFP (base)" : "PP (handset)");
                ImGui::TableSetColumnIndex(3);
                if (p.voicePresent && !p.qtReceived) { ImGui::TextUnformatted("syncing"); }
                else { ImGui::TextUnformatted(p.voicePresent ? "yes" : "-"); }
                ImGui::TableSetColumnIndex(4);
                switch (p.encSignalling) {
                case dect::ENC_SIG_STARTING: ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "starting"); break;
                case dect::ENC_SIG_ACTIVE:   ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "ACTIVE"); break;
                case dect::ENC_SIG_STOPPED:  ImGui::TextUnformatted("stopped"); break;
                default:                     ImGui::TextUnformatted("-"); break;
                }
            }
            if (!any) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted("--");
            }
            ImGui::EndTable();
        }

        if (ImGui::Checkbox(CONCAT("Follow active call##dect_auto_", _this->name), &_this->autoSelect)) {
            _this->rx.decoder.setAutoSelect(_this->autoSelect);
            _this->saveConfig();
        }

        if (ImGui::Checkbox(CONCAT("Mute encrypted audio##dect_mute_", _this->name), &_this->muteEncrypted)) {
            _this->rx.decoder.setMuteEncrypted(_this->muteEncrypted);
            _this->saveConfig();
        }

        uint64_t good, bad;
        _this->rx.decoder.getStats(good, bad);
        ImGui::Text("Voice frames: %llu ok, %llu bad CRC", (unsigned long long)good, (unsigned long long)bad);
        if (_this->muteEncrypted) {
            ImGui::Text("Muted as encrypted: %llu", (unsigned long long)_this->rx.decoder.getMutedFrames());
        }

        // ---- Encryption verdict for the part being listened to ----
        dect::VoiceAnalysis va = _this->rx.decoder.getVoiceAnalysis();
        ImGui::TextUnformatted("Link:");
        ImGui::SameLine();
        if (va.signalled == dect::ENC_SIG_ACTIVE) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "ENCRYPTED (MAC signalling)");
        }
        else if (va.payload == dect::PAYLOAD_RANDOM) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.2f, 1.0f), "Likely encrypted (random payload)");
        }
        else if (va.payload == dect::PAYLOAD_PHASE_MISMATCH) {
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "Decoder desync (voice fits frame offset +%d)", va.bestPhase);
        }
        else if (va.payload == dect::PAYLOAD_STRUCTURED) {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "Clear voice");
        }
        else {
            ImGui::TextUnformatted("Analysing...");
        }
        if (va.frames > 0) {
            ImGui::Text("Payload score %.3f (other phase %.3f, %d frames)", va.usedScore, va.bestScore, va.frames);
        }

        if (!_this->enabled) { style::endDisabled(); }
    }

    static void symbolHandler(const float* data, int count, void* ctx) {
        DECTDecoderModule* _this = (DECTDecoderModule*)ctx;
        float* buf = _this->diag.acquireBuffer();
        memcpy(buf, data, count * sizeof(float));
        _this->diag.releaseBuffer();
    }

    static void sampleRateChangeHandler(float sampleRate, void* ctx) {
        DECTDecoderModule* _this = (DECTDecoderModule*)ctx;
        _this->audioSampRate = sampleRate;
        _this->resamp.tempStop();
        _this->resamp.setOutSamplerate(sampleRate);
        _this->resamp.tempStart();
    }

    std::string name;
    bool enabled = true;

    int plan = 0;
    int carrier = 0;
    bool autoSelect = true;
    bool muteEncrypted = false;

    // DSP chain
    VFOManager::VFO* vfo = NULL;
    dect::Receiver rx;
    dsp::multirate::RationalResampler<dsp::stereo_t> resamp;

    ImGui::SymbolDiagram diag;

    double audioSampRate = 48000;
    EventHandler<float> srChangeHandler;
    SinkManager::Stream stream;

    uint64_t lastBurstCount = 0;
    std::chrono::steady_clock::time_point lastBurstTime;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    config.setPath(core::args["root"].s() + "/dect_decoder_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new DECTDecoderModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (DECTDecoderModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}

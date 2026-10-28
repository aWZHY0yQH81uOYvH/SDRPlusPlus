#pragma once
#include <dsp/processor.h>
#include <dsp/types.h>
#include <deque>
#include <atomic>
#include "dect_dsp.h"

namespace dect {
    // SDR++ DSP block: complex baseband @ 4.608 MSPS in, 8 kHz mono audio (as stereo_t) out.
    //
    // Voice arrives from the air as 10 ms bursts but the audio stream has to be continuous. The output is therefore
    // clocked by the input: 8000 audio samples are produced per 4.608e6 input samples, taken from a small FIFO that
    // is primed with a couple of frames before playback starts (so burst jitter does not cause dropouts) and padded
    // with silence when there is no voice.
    class Receiver : public dsp::Processor<dsp::complex_t, dsp::stereo_t> {
        using base_type = dsp::Processor<dsp::complex_t, dsp::stereo_t>;
    public:
        Receiver() {}

        Receiver(dsp::stream<dsp::complex_t>* in) { init(in); }

        ~Receiver() {
            if (!base_type::_block_init) { return; }
            base_type::stop();
        }

        void init(dsp::stream<dsp::complex_t>* in) {
            receiver.setBurstHandler([this](const Burst& b) {
                burstCount++;
                decoder.onBurst(b);
            });
            receiver.setLostPartHandler([this](uint32_t id) { decoder.partLost(id); });
            receiver.collectSoftSymbols = true;
            base_type::init(in);
        }

        // Called from the DSP thread with 'count' discriminator values (about +-1.2 rad nominal, positive = '1')
        void setSymbolHandler(void (*handler)(const float* data, int count, void* ctx), void* ctx) {
            symHandler = handler;
            symCtx = ctx;
        }

        void reset() {
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            phaseDiff.reset();
            receiver.reset();
            receiver.softSymbols.clear();
            decoder.reset();
            fifo.clear();
            playing = false;
            sampleDebt = 0;
            base_type::tempStart();
        }

        inline int process(int count, const dsp::complex_t* in, dsp::stereo_t* out) {
            for (int i = 0; i < count; i++) {
                receiver.process(phaseDiff.process(in[i].re, in[i].im));
            }

            // Collect decoded voice
            for (int16_t s : decoder.pcm) { fifo.push_back(s); }
            decoder.pcm.clear();
            while (fifo.size() > MAX_FIFO) { fifo.pop_front(); }

            // Soft symbols for the display
            if (symHandler && receiver.softSymbols.size() >= SYMBOL_CHUNK) {
                symHandler(receiver.softSymbols.data() + receiver.softSymbols.size() - SYMBOL_CHUNK, SYMBOL_CHUNK, symCtx);
            }
            receiver.softSymbols.clear();

            // Produce the audio owed for this much input
            sampleDebt += count;
            int outCount = (int)(sampleDebt / SAMPLES_PER_AUDIO_SAMPLE);
            sampleDebt -= (uint64_t)outCount * SAMPLES_PER_AUDIO_SAMPLE;

            for (int i = 0; i < outCount; i++) {
                if (!playing && fifo.size() >= PRIME_SAMPLES) { playing = true; }
                float v = 0.0f;
                if (playing) {
                    if (fifo.empty()) {
                        playing = false;
                    }
                    else {
                        v = (float)fifo.front() / 32768.0f;
                        fifo.pop_front();
                    }
                }
                out[i].l = v;
                out[i].r = v;
            }
            return outCount;
        }

        int run() {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            int outCount = process(count, base_type::_in->readBuf, base_type::out.writeBuf);

            base_type::_in->flush();
            if (outCount) {
                if (!base_type::out.swap(outCount)) { return -1; }
            }
            return count;
        }

        PacketDecoder decoder;
        std::atomic<uint64_t> burstCount{ 0 };

    private:
        static constexpr size_t PRIME_SAMPLES = 2 * VOICE_SAMPLES_PER_FRAME;     // 20 ms of audio
        static constexpr size_t MAX_FIFO = 50 * VOICE_SAMPLES_PER_FRAME;         // 0.5 s
        static constexpr size_t SYMBOL_CHUNK = 388;

        PhaseDiff phaseDiff;
        PacketReceiver receiver;

        std::deque<int16_t> fifo;
        bool playing = false;
        uint64_t sampleDebt = 0;

        void (*symHandler)(const float* data, int count, void* ctx) = NULL;
        void* symCtx = NULL;
    };
}

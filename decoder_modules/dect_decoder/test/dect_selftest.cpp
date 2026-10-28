// Standalone end-to-end self test for the DECT receive chain (no SDR++ needed).
//
//   c++ -std=c++17 -O2 -I../src dect_selftest.cpp -o dect_selftest && ./dect_selftest
//
// A synthetic DECT transmitter (GFSK, BT=0.5, +-288 kHz deviation, S/A/B/X fields with CRCs and scrambling) puts three
// parts on the air: RFP "A" (voice), its paired PP (voice) and a second RFP "B" (no voice). The IQ goes through a
// frequency offset, a channel filter and optional noise, then through PhaseDiff -> PacketReceiver -> PacketDecoder.
//
// It also covers the encryption indicators: MAC encryption control messages (Mt tails), a ciphered B-field (random
// keystream) with and without the signalling having been seen, and a deliberately wrong scrambler phase.
//
// Note: the transmitter is built from the same CRC / scrambler primitives as the decoder, so this verifies the signal
// chain, sync, part tracking, pairing and voice plumbing, not the protocol tables against an external DECT capture.
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <complex>
#include <random>
#include <vector>
#include "dect_dsp.h"

using namespace dect;
typedef std::complex<float> cf;

static int failures = 0;
#define CHECK(cond, ...)                                       \
    do {                                                       \
        if (cond) { printf("  [ ok ] "); }                     \
        else { printf("  [FAIL] "); failures++; }              \
        printf(__VA_ARGS__);                                   \
        printf("\n");                                          \
    } while (0)

struct TxPart {
    PartType type;
    int slot;
    bool voice;
    uint8_t id[5];
    float amplitude;
    int firstFrame = 0;
    int lastFrame = 1 << 30;
    std::vector<uint8_t> codes;   // ADPCM code words for this part's voice, 80 per frame
    // Encryption handshake: PP "start encryption request" in frame sigStartFrame, RFP "confirm" one frame later, PP "grant"
    // one frame after that. Stop: PP "stop request" in sigStopFrame, RFP "stop confirm" one frame later.
    int sigStartFrame = -1;
    int sigStopFrame = -1;
    int cipherFrame = -1;         // B-field is ciphered with a random keystream from this frame on
    int cipherEndFrame = 1 << 30; // ... until (excluding) this frame
    int scrambleOffset = 0;       // transmitter scrambles with phase (frame number + offset): receiver frame number looks wrong
};

static std::vector<uint8_t> makeCodes(int frames, uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<uint8_t> c(frames * 80);
    // Mostly small codes with occasional large ones, so the ADPCM state moves around
    for (auto& v : c) {
        int r = rng() % 100;
        v = (r < 70) ? (rng() % 4) | ((rng() & 1) ? 8 : 0) : (rng() % 16);
    }
    return c;
}

static void bitsOfByte(uint8_t b, std::vector<uint8_t>& out) {
    for (int i = 7; i >= 0; i--) { out.push_back((b >> i) & 1); }
}

// Build the 420 transmitted bits (S-field + A + B + X) for one burst
static std::vector<uint8_t> buildBurst(const TxPart& p, int frame, int frameInPart) {
    std::vector<uint8_t> bits;
    uint32_t sync = (p.type == PART_RFP) ? RFP_SYNC_FIELD : ~RFP_SYNC_FIELD;
    for (int i = 31; i >= 0; i--) { bits.push_back((sync >> i) & 1); }

    // A-field: header (tail id, Q1, B-field type, Q2) + 5 tail bytes + R-CRC
    int fn = frame % 16;
    uint8_t ta, tail[5];
    int mtCmd = -1;   // MAC encryption control command nibble, if one is sent in this frame
    if (p.sigStartFrame >= 0) {
        int rel = frame - p.sigStartFrame;
        if (p.type == PART_PP && rel == 0) { mtCmd = 0x0; }          // start encryption: request
        else if (p.type == PART_RFP && rel == 1) { mtCmd = 0x1; }    // start encryption: confirm
        else if (p.type == PART_PP && rel == 2) { mtCmd = 0x2; }     // start encryption: grant
    }
    if (p.sigStopFrame >= 0) {
        int rel = frame - p.sigStopFrame;
        if (p.type == PART_PP && rel == 0) { mtCmd = 0x4; }          // stop encryption: request
        else if (p.type == PART_RFP && rel == 1) { mtCmd = 0x5; }    // stop encryption: confirm
    }
    if (mtCmd >= 0) {
        ta = 6;                                          // Mt: MAC control, header 0101 = encryption control
        tail[0] = (uint8_t)(0x50 | mtCmd);
        for (int i = 1; i < 5; i++) { tail[i] = (uint8_t)(0x20 + i); }   // FMID / PMID
    }
    else if (p.type == PART_RFP && fn == 8) {
        ta = 4;                                          // Qt
        for (int i = 0; i < 5; i++) { tail[i] = 0x10 + i; }
    }
    else if (fn % 4 == 1) {
        ta = 1;                                          // some other tail type
        for (int i = 0; i < 5; i++) { tail[i] = (uint8_t)(frame * 7 + i); }
    }
    else {
        ta = 3;                                          // Nt: part identity
        memcpy(tail, p.id, 5);
    }
    uint8_t ba = p.voice ? 0 : 7;
    uint8_t a[8];
    a[0] = (uint8_t)((ta << 5) | (ba << 1));
    memcpy(a + 1, tail, 5);
    uint16_t crc = detail::calcRCRC(a, 6);
    a[6] = crc >> 8;
    a[7] = crc & 0xFF;
    for (int i = 0; i < 8; i++) { bitsOfByte(a[i], bits); }

    // B-field: voice codes, scrambled with the frame number
    uint8_t b[40];
    if (p.voice) {
        const uint8_t* c = &p.codes[(size_t)frameInPart * 80];
        for (int i = 0; i < 40; i++) {
            uint8_t byte = (uint8_t)((c[2 * i] << 4) | c[2 * i + 1]);
            if (p.cipherFrame >= 0 && frame >= p.cipherFrame && frame < p.cipherEndFrame) {
                uint32_t k = (uint32_t)frame * 2654435761u + (uint32_t)i * 40503u + (uint32_t)p.slot * 977u;
                k = k * 1664525u + 1013904223u;
                k ^= k >> 15;
                k = k * 22695477u + 1u;
                byte ^= (uint8_t)(k >> 24);            // keystream: statistically uniform, differs for every byte
            }
            b[i] = byte ^ detail::scrambler(fn + p.scrambleOffset)[i % 31];
        }
    }
    else {
        for (int i = 0; i < 40; i++) { b[i] = (uint8_t)(i * 37 + frame); }
    }
    for (int i = 0; i < 40; i++) { bitsOfByte(b[i], bits); }

    // X-CRC over the descrambled B-field; the X-field is scrambled with the continuing sequence (as seen on a real capture)
    const uint8_t* wh = detail::scrambler(fn + p.scrambleOffset);
    uint8_t plainB[40];
    for (int i = 0; i < 40; i++) { plainB[i] = b[i] ^ wh[i % 31]; }
    uint8_t x = (uint8_t)(detail::calcXCRC(plainB) ^ (wh[40 % 31] >> 4));
    for (int i = 3; i >= 0; i--) { bits.push_back((x >> i) & 1); }
    return bits;
}

// GFSK modulate (+ settled frequency +288 kHz for '1')
static void modulate(const std::vector<uint8_t>& bits, float amp, double cfoHz, std::mt19937& rng, std::vector<cf>& out) {
    // Gaussian pulse shaping (BT=0.5) of rectangular NRZ at 4 samples/symbol
    const double sigma = sqrt(log(2.0)) / (2.0 * M_PI * 0.5) * SAMPLES_PER_SYMBOL;   // in samples
    const int half = 6;
    double taps[2 * half + 1], sum = 0;
    for (int k = -half; k <= half; k++) { sum += taps[k + half] = exp(-0.5 * k * k / (sigma * sigma)); }
    for (auto& t : taps) { t /= sum; }

    std::vector<double> nrz;
    for (uint8_t bit : bits) { for (int i = 0; i < SAMPLES_PER_SYMBOL; i++) { nrz.push_back(bit ? 1.0 : -1.0); } }
    nrz.insert(nrz.begin(), 2 * SAMPLES_PER_SYMBOL, nrz.front());
    nrz.insert(nrz.end(), 2 * SAMPLES_PER_SYMBOL, nrz.back());

    double phase = (rng() % 1000) / 1000.0 * 2 * M_PI;
    for (size_t n = half; n + half < nrz.size(); n++) {
        double s = 0;
        for (int k = -half; k <= half; k++) { s += taps[k + half] * nrz[n + k]; }
        phase += 2 * M_PI * (288000.0 * s + cfoHz) / SAMPLE_RATE;
        out.push_back(cf(amp * (float)cos(phase), amp * (float)sin(phase)));
    }
}

struct Scene {
    std::vector<TxPart> parts;
    int frames = 60;
    double cfoHz = 0;
    double snrDb = 1e9;      // signal power to noise power within the ~1.4 MHz channel
    int startOffset = 777;   // arbitrary, not a multiple of 4
    uint32_t seed = 1;
};

static std::vector<cf> synthesize(const Scene& sc) {
    std::mt19937 rng(sc.seed);
    std::vector<cf> sig((size_t)(sc.frames + 1) * INTER_FRAME_TIME, cf(0, 0));
    for (const TxPart& p : sc.parts) {
        for (int f = p.firstFrame; f < sc.frames && f <= p.lastFrame; f++) {
            std::vector<uint8_t> bits = buildBurst(p, f, f - p.firstFrame);
            std::vector<cf> burst;
            modulate(bits, p.amplitude, sc.cfoHz, rng, burst);
            size_t start = (size_t)f * INTER_FRAME_TIME + (size_t)p.slot * INTER_SLOT_TIME + sc.startOffset;
            for (size_t i = 0; i < burst.size() && start + i < sig.size(); i++) { sig[start + i] = burst[i]; }
        }
    }

    // Noise: white over the full 4.608 MHz; the channel filter keeps ~1.38/4.608 of it
    if (sc.snrDb < 200) {
        double sigPow = 1.0;   // unit-amplitude reference part
        double chanFrac = 1.38e6 / SAMPLE_RATE;
        double noisePow = sigPow / pow(10.0, sc.snrDb / 10.0) / chanFrac;
        std::normal_distribution<float> nd(0.0f, (float)sqrt(noisePow / 2.0));
        for (auto& s : sig) { s += cf(nd(rng), nd(rng)); }
    }

    // Channel filter (what the SDR++ VFO does): windowed-sinc low pass, ~0.69 MHz cutoff
    const int N = 65;
    double h[N], hs = 0;
    for (int i = 0; i < N; i++) {
        double t = i - (N - 1) / 2.0;
        double fc = 0.69e6 / SAMPLE_RATE;
        double sinc = (t == 0) ? 2 * fc : sin(2 * M_PI * fc * t) / (M_PI * t);
        double w = 0.42 - 0.5 * cos(2 * M_PI * i / (N - 1)) + 0.08 * cos(4 * M_PI * i / (N - 1));
        hs += h[i] = sinc * w;
    }
    std::vector<cf> out(sig.size());
    for (size_t n = 0; n < sig.size(); n++) {
        cf acc(0, 0);
        for (int i = 0; i < N; i++) {
            long idx = (long)n - i;
            if (idx >= 0) { acc += (float)(h[i] / hs) * sig[idx]; }
        }
        out[n] = acc;
    }
    return out;
}

struct Result {
    std::vector<int16_t> pcm;
    PartInfo parts[MAX_PARTS];
    int selected = 0;
    uint64_t good = 0, bad = 0, bursts = 0, muted = 0;
    VoiceAnalysis va;
};

// Run the receive chain over the IQ in blocks (like the SDR++ stream buffers do)
static Result runChain(const std::vector<cf>& iq, bool autoSel, int manualSel = 0, bool muteEncrypted = false) {
    PhaseDiff pd;
    PacketReceiver rx;
    PacketDecoder dec;
    Result r;
    dec.setAutoSelect(autoSel);
    dec.setMuteEncrypted(muteEncrypted);
    if (!autoSel) { dec.selectPart(manualSel); }
    rx.setBurstHandler([&](const Burst& b) { r.bursts++; dec.onBurst(b); });
    rx.setLostPartHandler([&](uint32_t id) { dec.partLost(id); });

    size_t pos = 0;
    while (pos < iq.size()) {
        size_t n = std::min<size_t>(4999, iq.size() - pos);   // odd block size on purpose
        for (size_t i = 0; i < n; i++) { rx.process(pd.process(iq[pos + i].real(), iq[pos + i].imag())); }
        pos += n;
        r.pcm.insert(r.pcm.end(), dec.pcm.begin(), dec.pcm.end());
        dec.pcm.clear();
    }
    int sel;
    dec.getParts(r.parts, sel);
    r.selected = sel;
    dec.getStats(r.good, r.bad);
    r.muted = dec.getMutedFrames();
    r.va = dec.getVoiceAnalysis();
    return r;
}

static std::vector<int16_t> expectedAudio(const std::vector<uint8_t>& codes, int firstFrame, int frames) {
    G721Decoder g;
    std::vector<int16_t> out;
    for (int f = firstFrame; f < frames; f++) {
        for (int i = 0; i < 80; i++) { out.push_back(g.decode(codes[(size_t)f * 80 + i])); }
    }
    return out;
}

static Scene makeScene(int frames) {
    Scene sc;
    sc.frames = frames;
    TxPart rfp{ PART_RFP, 0, true, { 0x01, 0x02, 0x03, 0x04, 0x05 }, 1.0f };
    TxPart pp{ PART_PP, 12, true, { 0x01, 0x02, 0x03, 0x04, 0x05 }, 0.5f };
    TxPart rfp2{ PART_RFP, 5, false, { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE }, 0.8f };
    rfp.codes = makeCodes(frames, 11);
    pp.codes = makeCodes(frames, 22);
    sc.parts = { rfp, pp, rfp2 };
    return sc;
}

// Just the call: an RFP and its PP, both carrying voice
static Scene makeCallScene(int frames) {
    Scene sc = makeScene(frames);
    sc.parts.pop_back();
    sc.cfoHz = 20e3;
    return sc;
}

static const char* payloadName(PayloadClass c) {
    switch (c) {
    case PAYLOAD_UNKNOWN: return "unknown";
    case PAYLOAD_STRUCTURED: return "structured (voice-like)";
    case PAYLOAD_RANDOM: return "random";
    case PAYLOAD_PHASE_MISMATCH: return "structured under another scrambler phase";
    }
    return "?";
}

static void printAnalysis(const Result& r) {
    printf("         analysis: payload %s, used-phase score %.4f, best other phase %d score %.4f, %d frames, signalled=%d\n",
           payloadName(r.va.payload), r.va.usedScore, r.va.bestPhase, r.va.bestScore, r.va.frames, (int)r.va.signalled);
}

int main() {
    const int FRAMES = 64;

    printf("Test 1: clean signal, +35 kHz carrier offset, auto-select\n");
    {
        Scene sc = makeScene(FRAMES);
        sc.cfoHz = 35e3;
        auto iq = synthesize(sc);
        Result r = runChain(iq, true);

        int active = 0, rfps = 0, pps = 0, paired = 0;
        for (auto& p : r.parts) {
            if (!p.active) { continue; }
            active++;
            (p.type == PART_RFP ? rfps : pps)++;
            if (p.pair >= 0) { paired++; }
        }
        CHECK(active == 3 && rfps == 2 && pps == 1, "3 parts tracked (%d RFP, %d PP)", rfps, pps);
        CHECK(paired == 2, "RFP and PP with the same ID were paired (%d linked parts)", paired);
        CHECK(r.bursts >= (uint64_t)(FRAMES - 1) * 3, "bursts received: %llu of %d", (unsigned long long)r.bursts, FRAMES * 3);

        int frames = (int)(r.pcm.size() / 80);
        CHECK(frames > 40 && r.bad == 0, "voice frames decoded: %d (bad X-CRC: %llu)", frames, (unsigned long long)r.bad);
        auto exp = expectedAudio(sc.parts[0].codes, FRAMES - frames, FRAMES);
        size_t mism = 0;
        for (size_t i = 0; i < std::min(exp.size(), r.pcm.size()); i++) { mism += exp[i] != r.pcm[i]; }
        CHECK(exp.size() == r.pcm.size() && mism == 0, "audio of RFP is bit-exact with the transmitted ADPCM stream (%zu samples, %zu mismatches)", r.pcm.size(), mism);
    }

    printf("Test 2: select the PP (needs the RFP for frame numbers), manual selection\n");
    {
        Scene sc = makeScene(FRAMES);
        sc.cfoHz = -20e3;
        auto iq = synthesize(sc);
        // Part ids are assigned in order of first arrival: RFP A = 0 (slot 0), RFP B = 1 (slot 5), PP = 2 (slot 12)
        Result r = runChain(iq, false, 2);
        int frames = (int)(r.pcm.size() / 80);
        CHECK(r.parts[2].type == PART_PP && r.parts[2].active, "part 2 is the PP");
        CHECK(frames > 40 && r.bad == 0, "voice frames decoded: %d (bad X-CRC: %llu)", frames, (unsigned long long)r.bad);
        auto exp = expectedAudio(sc.parts[1].codes, FRAMES - frames, FRAMES);
        size_t mism = 0;
        for (size_t i = 0; i < std::min(exp.size(), r.pcm.size()); i++) { mism += exp[i] != r.pcm[i]; }
        CHECK(exp.size() == r.pcm.size() && mism == 0, "audio of PP is bit-exact (%zu samples, %zu mismatches)", r.pcm.size(), mism);
    }

    printf("Test 3: a part that stops transmitting is released\n");
    {
        Scene sc = makeScene(FRAMES);
        sc.parts[1].lastFrame = 15;      // PP disappears after frame 15
        auto iq = synthesize(sc);
        Result r = runChain(iq, true);
        CHECK(!r.parts[2].active && r.parts[0].active, "PP released, RFP still active");
        CHECK(r.parts[0].pair < 0, "RFP no longer paired");
    }

    printf("Test 4: noise (SNR measured in the ~1.4 MHz channel), frames decoded / received voice frames\n");
    {
        double snrs[] = { 24, 18, 14, 11, 8 };
        for (double snr : snrs) {
            int totalGood = 0, totalBad = 0, trials = 4, expectFrames = 0;
            for (int t = 0; t < trials; t++) {
                Scene sc = makeScene(FRAMES);
                sc.snrDb = snr;
                sc.seed = 100 + t;
                sc.cfoHz = 25e3;
                auto iq = synthesize(sc);
                Result r = runChain(iq, true);
                totalGood += (int)r.good;
                totalBad += (int)r.bad;
                expectFrames += FRAMES - 8;   // voice can only start from the first Qt (frame 8)
            }
            printf("         SNR %4.0f dB: %3d/%3d frames good, %3d with bad X-CRC (silenced)\n", snr, totalGood, expectFrames, totalBad);
            if (snr >= 18) { CHECK(totalGood >= expectFrames * 9 / 10, "at least 90%% of frames decoded at %0.f dB", snr); }
        }
    }

    printf("Test 5: clear call: no encryption indicators\n");
    {
        Scene sc = makeCallScene(100);
        Result r = runChain(synthesize(sc), true);
        printAnalysis(r);
        CHECK(r.va.payload == PAYLOAD_STRUCTURED, "payload statistics say voice-like");
        CHECK(r.va.signalled == ENC_SIG_NONE && !r.va.encrypted(), "no encryption signalling, not flagged encrypted");
    }

    printf("Test 6: encryption handshake (frames 10-12) and ciphered audio from frame 12\n");
    {
        Scene sc = makeCallScene(140);
        for (auto& p : sc.parts) { p.sigStartFrame = 10; p.cipherFrame = 12; }
        Result r = runChain(synthesize(sc), true);
        printAnalysis(r);
        CHECK(r.parts[0].encSignalling == ENC_SIG_ACTIVE && r.parts[0].encMsgCount >= 1, "RFP: MAC signals encryption active (%u messages)", r.parts[0].encMsgCount);
        CHECK(r.parts[1].encSignalling == ENC_SIG_ACTIVE, "PP: encryption state shared through the pairing");
        CHECK(r.va.payload == PAYLOAD_RANDOM, "payload statistics say random");
        CHECK(r.va.encrypted(), "call flagged as encrypted");
        CHECK(r.bad == 0, "the X-CRC still passes on ciphered frames (%llu bad): the CRC cannot tell", (unsigned long long)r.bad);
    }

    printf("Test 7: ciphered call, handshake not seen (receiver joined late)\n");
    {
        Scene sc = makeCallScene(100);
        for (auto& p : sc.parts) { p.cipherFrame = 0; }
        Result r = runChain(synthesize(sc), true);
        printAnalysis(r);
        CHECK(r.va.signalled == ENC_SIG_NONE, "no signalling seen");
        CHECK(r.va.payload == PAYLOAD_RANDOM && r.va.encrypted(), "flagged encrypted from the payload statistics alone");
    }

    printf("Test 8: wrong scrambler phase (frame number off by 3) is a decoding problem, not encryption\n");
    {
        Scene sc = makeCallScene(100);
        for (auto& p : sc.parts) { p.scrambleOffset = 3; }
        Result r = runChain(synthesize(sc), true);
        printAnalysis(r);
        CHECK(r.va.payload == PAYLOAD_PHASE_MISMATCH && r.va.bestPhase == 3, "reported as phase mismatch, offset %d", r.va.bestPhase);
        CHECK(!r.va.encrypted(), "NOT flagged as encrypted");
    }

    printf("Test 9: encryption switched off again (stop request/confirm in frames 70-71)\n");
    {
        Scene sc = makeCallScene(160);
        for (auto& p : sc.parts) { p.sigStartFrame = 10; p.cipherFrame = 12; p.sigStopFrame = 70; p.cipherEndFrame = 72; }
        Result r = runChain(synthesize(sc), true);
        printAnalysis(r);
        CHECK(r.va.signalled == ENC_SIG_STOPPED, "MAC signals encryption stopped");
        CHECK(r.va.payload == PAYLOAD_STRUCTURED && !r.va.encrypted(), "payload voice-like again, not flagged");
    }

    printf("Test 10: mute option replaces ciphered audio with silence\n");
    {
        Scene sc = makeCallScene(100);
        for (auto& p : sc.parts) { p.cipherFrame = 0; }
        auto iq = synthesize(sc);
        Result loud = runChain(iq, true, 0, false);
        Result quiet = runChain(iq, true, 0, true);
        auto peak = [](const std::vector<int16_t>& v, size_t from) { int m = 0; for (size_t i = from; i < v.size(); i++) { m = std::max(m, abs((int)v[i])); } return m; };
        printf("         peak level of the last 40 frames: %d unmuted, %d muted (%llu frames muted)\n", peak(loud.pcm, loud.pcm.size() - 3200), peak(quiet.pcm, quiet.pcm.size() - 3200), (unsigned long long)quiet.muted);
        CHECK(peak(loud.pcm, loud.pcm.size() - 3200) > 1000, "unmuted: ciphertext decodes to loud noise");
        CHECK(peak(quiet.pcm, quiet.pcm.size() - 3200) == 0 && quiet.muted > 40, "muted: silence");
    }

    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}

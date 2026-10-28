#pragma once
// ---------------------------------------------------------------------------
// DECT (GAP) receive chain: GFSK discriminator -> burst sync -> A/B-field
// decoding -> G.721 voice.
//
// This is a port of the GNU Radio out-of-tree module gr-dect2:
//     https://github.com/pavelyazev/gr-dect2
//     Copyright 2015 Pavel Yazev <pyazev@gmail.com>
// (GPLv3 or later, same as SDR++). The three GNU Radio blocks map as follows:
//
//     dect2_phase_diff      ->  dect::PhaseDiff
//     dect2_packet_receiver ->  dect::PacketReceiver
//     dect2_packet_decoder  ->  dect::PacketDecoder
//
// The algorithms (sync search, best sample point selection, part tracking,
// R-CRC / X-CRC, descrambling tables) are kept the same as the original.
// Deliberate differences from the original:
//   * No GNU Radio types: plain C++17, no dependencies, so it can be unit tested
//     without SDR++.
//   * Members that were uninitialised in the original are zero-initialised.
//   * Part-pair pointers became indices (the part table is copied to the UI).
//   * Part activity is polled every 1024 samples instead of every sample (the
//     timeout is 4 frames = 184320 samples so this is not observable).
//   * Optional "auto select": follow whichever part currently carries voice.
//
// Signal format expected by this chain: complex baseband, exactly 4 samples per
// symbol (4.608 MSPS), channel filtered to ~1.4 MHz, tuned to a DECT carrier.
// ---------------------------------------------------------------------------
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <mutex>
#include <functional>
#include <atomic>
#include "g721.h"

namespace dect {
    constexpr double SYMBOL_RATE = 1152000.0;
    constexpr int SAMPLES_PER_SYMBOL = 4;
    constexpr double SAMPLE_RATE = SYMBOL_RATE * SAMPLES_PER_SYMBOL;     // 4.608 MSPS
    constexpr int AUDIO_RATE = 8000;
    constexpr int SAMPLES_PER_AUDIO_SAMPLE = 576;                        // SAMPLE_RATE / AUDIO_RATE
    constexpr double CARRIER_SPACING = 1728000.0;

    constexpr int MAX_PARTS = 8;                       // Maximum number of DECT parts tracked at once
    constexpr int SMPL_BUF_LEN = 32 * 4;
    constexpr int TIME_TOL = 10;                       // Time tolerance (samples)
    constexpr int INTER_SLOT_TIME = 480 * 4;
    constexpr int INTER_FRAME_TIME = INTER_SLOT_TIME * 24;
    constexpr int S_FIELD_BITS = 32;
    constexpr int A_FIELD_BITS = 64;
    constexpr int B_FIELD_BITS = 320;
    constexpr int X_FIELD_BITS = 4;
    constexpr int D_FIELD_BITS = A_FIELD_BITS + B_FIELD_BITS + X_FIELD_BITS;   // 388 (full slot, "P32")
    constexpr uint32_t RFP_SYNC_FIELD = 0xAAAAE98A;
    constexpr bool PP_IGNORE_XCRC = true;
    constexpr int VOICE_SAMPLES_PER_FRAME = 80;        // 320 B-field bits / 4 bit ADPCM codes

    enum PartType { PART_RFP = 0, PART_PP = 1 };

    // atan2 approximation (max error ~0.004 rad). Only the sign and rough magnitude of the
    // phase step are used, so this is plenty.
    inline float fastAtan2(float y, float x) {
        const float ONEQTR_PI = 0.78539816f;
        const float THRQTR_PI = 2.35619449f;
        float absY = fabsf(y) + 1e-10f;
        float r, angle;
        if (x < 0.0f) {
            r = (x + absY) / (absY - x);
            angle = THRQTR_PI;
        }
        else {
            r = (x - absY) / (x + absY);
            angle = ONEQTR_PI;
        }
        angle += (0.1963f * r * r - 0.9817f) * r;
        return (y < 0.0f) ? -angle : angle;
    }

    // -----------------------------------------------------------------------
    // GFSK discriminator. Phase difference between samples n-3 and n, which is
    // what gr-dect2's dect2_phase_diff does (in[i] * conj(in[i+3])).
    // A positive frequency deviation (bit '1') gives a negative output.
    // -----------------------------------------------------------------------
    class PhaseDiff {
    public:
        PhaseDiff() { reset(); }

        void reset() {
            for (int i = 0; i < 3; i++) { hRe[i] = hIm[i] = 0.0f; }
            pos = 0;
        }

        inline float process(float re, float im) {
            // Ring of the last 3 samples: the slot about to be overwritten holds sample n-3
            int old = pos;
            float oRe = hRe[old], oIm = hIm[old];
            hRe[old] = re;
            hIm[old] = im;
            pos = (pos == 2) ? 0 : pos + 1;
            // old * conj(new)
            float pr = oRe * re + oIm * im;
            float pi = oIm * re - oRe * im;
            return fastAtan2(pi, pr);
        }

    private:
        float hRe[3], hIm[3];
        int pos;
    };

    // One received D-field (A + B + X) with the metadata of the burst it came from
    struct Burst {
        uint32_t partRxId;
        uint64_t rxSeq;
        PartType type;
        uint8_t bits[D_FIELD_BITS];
    };

    // -----------------------------------------------------------------------
    // Burst receiver: finds the S-field sync word, picks the best of the four
    // sample phases, slices the bits and tracks which burst belongs to which
    // "part" (a transmitter) from the burst timing.
    // -----------------------------------------------------------------------
    class PacketReceiver {
    public:
        PacketReceiver() { reset(); }

        void reset() {
            memset(rxBitsBuf, 0, sizeof(rxBitsBuf));
            memset(smplBuf, 0, sizeof(smplBuf));
            memset(partTime, 0, sizeof(partTime));
            memset(partSeq, 0, sizeof(partSeq));
            rxBitsIdx = 0;
            smplBufIdx = 0;
            syncState = WAIT_BEGIN;
            partType = PART_RFP;
            beginPos = endPos = 0;
            smplCnt = 0;
            outBitCnt = 0;
            incSmplCnt = 0;
            partActivity = 0;
            curPartRxId = -1;
        }

        void setBurstHandler(std::function<void(const Burst&)> h) { onBurst = h; }
        void setLostPartHandler(std::function<void(uint32_t)> h) { onLostPart = h; }

        // Soft symbols (discriminator value at the chosen sampling instant) of the payload of the burst being
        // received. Cleared by the caller.
        std::vector<float> softSymbols;
        bool collectSoftSymbols = false;
        int debugShift = 0;

        inline void process(float v) {
            uint32_t rxBit = (v >= 0.0f) ? 0 : 1;
            uint32_t& reg = rxBitsBuf[rxBitsIdx];
            reg = (reg << 1) | rxBit;
            smplBuf[smplBufIdx] = v;

            switch (syncState) {
            case WAIT_BEGIN: {
                // Because there are four samples per symbol the sync word can be seen at several consecutive
                // positions. Remember the interval, then look for the best sample point.
                bool det = (reg == RFP_SYNC_FIELD);
                if (det) {
                    partType = PART_RFP;
                }
                else {
                    det = (reg == ~RFP_SYNC_FIELD);
                    if (det) { partType = PART_PP; }
                }
                if (det) {
                    beginPos = smplBufIdx;
                    syncState = WAIT_END;
                }
                break;
            }
            case WAIT_END: {
                bool det = (partType == PART_RFP) ? (reg == RFP_SYNC_FIELD) : (reg == ~RFP_SYNC_FIELD);
                if (!det) {
                    endPos = (smplBufIdx - 1) & (SMPL_BUF_LEN - 1);
                    smplCnt = (1 + findBestSamplePoint() + debugShift) & 3;

                    curPartRxId = registerPart();
                    if (curPartRxId < 0) {
                        syncState = WAIT_BEGIN;
                        break;
                    }
                    outBitCnt = 0;
                    syncState = POST_WAIT;
                }
                break;
            }
            case POST_WAIT:
                if (smplCnt == 0) {
                    if (outBitCnt == 0) {
                        burst.partRxId = (uint32_t)curPartRxId;
                        burst.rxSeq = partSeq[curPartRxId];
                        burst.type = partType;
                    }
                    burst.bits[outBitCnt] = (uint8_t)rxBit;
                    if (collectSoftSymbols) { softSymbols.push_back(-v); }
                    if (++outBitCnt == D_FIELD_BITS) {
                        syncState = WAIT_BEGIN;
                        if (onBurst) { onBurst(burst); }
                    }
                }
                break;
            }

            // Release parts that went quiet and tell the decoder
            if ((incSmplCnt & 1023) == 0 && partActivity) {
                int lost;
                while ((lost = checkPartActivity()) >= 0) {
                    if (onLostPart) { onLostPart((uint32_t)lost); }
                }
            }

            smplBufIdx = (smplBufIdx + 1) & (SMPL_BUF_LEN - 1);
            rxBitsIdx = (rxBitsIdx + 1) & 3;
            smplCnt = (smplCnt + 1) & 3;
            incSmplCnt++;
        }

    private:
        // Returns the id of a part that has been inactive for too long (and releases it), or -1
        int checkPartActivity() {
            if (partActivity) {
                uint32_t j = 0;
                uint32_t mask = 1;
                while (mask <= partActivity) {
                    if (partActivity & mask) {
                        if (incSmplCnt - partTime[j] > (uint64_t)(4 * INTER_FRAME_TIME)) {
                            partActivity &= ~mask;
                            return (int)j;
                        }
                    }
                    mask <<= 1;
                    j++;
                }
            }
            return -1;
        }

        // Several DECT parts may be on air. Each one transmits in the same slot of every 10 ms frame, so a burst
        // belongs to a known part if it arrives a whole number of frames after that part's previous burst.
        // Returns the part id, or -1 if there is no room for another part.
        int registerPart() {
            if (partActivity) {
                uint32_t j = 0;
                uint32_t mask = 1;
                uint32_t seq = 0;
                while (j < (uint32_t)MAX_PARTS) {
                    if (partActivity & mask) {
                        uint64_t ltmp = (incSmplCnt - partTime[j]) % INTER_FRAME_TIME;
                        if (ltmp < (uint64_t)TIME_TOL) {
                            seq = (uint32_t)((incSmplCnt - partTime[j]) / INTER_FRAME_TIME);
                            break;
                        }
                        else if (INTER_FRAME_TIME - ltmp <= (uint64_t)TIME_TOL) {
                            seq = 1 + (uint32_t)((incSmplCnt - partTime[j]) / INTER_FRAME_TIME);
                            break;
                        }
                    }
                    mask <<= 1;
                    j++;
                }

                if (j < (uint32_t)MAX_PARTS) {
                    partTime[j] = incSmplCnt;
                    partSeq[j] = (partSeq[j] + seq) & 0x1F;
                    return (int)j;
                }

                // Add a new part
                j = 0;
                mask = 1;
                while (j < (uint32_t)MAX_PARTS) {
                    if (partActivity & mask) {
                        mask <<= 1;
                        j++;
                    }
                    else {
                        partActivity |= mask;
                        break;
                    }
                }
                if (j < (uint32_t)MAX_PARTS) {
                    partTime[j] = incSmplCnt;
                    partSeq[j] = 0;
                    return (int)j;
                }
                return -1;
            }

            partTime[0] = incSmplCnt;
            partSeq[0] = 0;
            partActivity = 1;
            return 0;
        }

        // Among the sample positions where the sync word was seen, pick the one with the largest eye opening
        int findBestSamplePoint() {
            if (beginPos != endPos) {
                float maxVal = 0.0f;
                uint32_t maxIndex = beginPos;
                while (true) {
                    uint32_t index = beginPos;
                    float acc = 0.0f;
                    for (uint32_t j = 0; j < 32; j++) {
                        acc += fabsf(smplBuf[index]);
                        index = (index - 4) & (SMPL_BUF_LEN - 1);
                    }
                    if (acc > maxVal) {
                        maxVal = acc;
                        maxIndex = beginPos;
                    }
                    if (beginPos == endPos) { break; }
                    beginPos = (beginPos + 1) & (SMPL_BUF_LEN - 1);
                }
                return (int)((endPos - maxIndex) & (SMPL_BUF_LEN - 1));
            }
            return 0;
        }

        enum SyncState { WAIT_BEGIN, WAIT_END, POST_WAIT };

        // One shift register per sample phase (4 samples per symbol)
        uint32_t rxBitsBuf[4];
        unsigned rxBitsIdx;
        SyncState syncState;
        PartType partType;

        uint32_t beginPos, endPos;
        float smplBuf[SMPL_BUF_LEN];
        uint32_t smplBufIdx;
        uint32_t smplCnt;
        uint32_t outBitCnt;
        uint64_t incSmplCnt;

        uint64_t partTime[MAX_PARTS];
        uint32_t partSeq[MAX_PARTS];
        uint32_t partActivity;
        int curPartRxId;

        Burst burst;
        std::function<void(const Burst&)> onBurst;
        std::function<void(uint32_t)> onLostPart;
    };

    // -----------------------------------------------------------------------
    // CRCs and descrambler (taken unchanged from gr-dect2)
    // -----------------------------------------------------------------------
    namespace detail {
        // scramble table with corrections by Jakub Hruska (from gr-dect2)
        static const uint8_t scrt[8][31] = {
            { 0x3B, 0xCD, 0x21, 0x5D, 0x88, 0x65, 0xBD, 0x44, 0xEF, 0x34, 0x85, 0x76, 0x21, 0x96, 0xF5, 0x13, 0xBC, 0xD2, 0x15, 0xD8, 0x86, 0x5B, 0xD4, 0x4E, 0xF3, 0x48, 0x57, 0x62, 0x19, 0x6F, 0x51 },
            { 0x32, 0xDE, 0xA2, 0x77, 0x9A, 0x42, 0xBB, 0x10, 0xCB, 0x7A, 0x89, 0xDE, 0x69, 0x0A, 0xEC, 0x43, 0x2D, 0xEA, 0x27, 0x79, 0xA4, 0x2B, 0xB1, 0x0C, 0xB7, 0xA8, 0x9D, 0xE6, 0x90, 0xAE, 0xC4 },
            { 0x2D, 0xEA, 0x27, 0x79, 0xA4, 0x2B, 0xB1, 0x0C, 0xB7, 0xA8, 0x9D, 0xE6, 0x90, 0xAE, 0xC4, 0x32, 0xDE, 0xA2, 0x77, 0x9A, 0x42, 0xBB, 0x10, 0xCB, 0x7A, 0x89, 0xDE, 0x69, 0x0A, 0xEC, 0x43 },
            { 0x27, 0x79, 0xA4, 0x2B, 0xB1, 0x0C, 0xB7, 0xA8, 0x9D, 0xE6, 0x90, 0xAE, 0xC4, 0x32, 0xDE, 0xA2, 0x77, 0x9A, 0x42, 0xBB, 0x10, 0xCB, 0x7A, 0x89, 0xDE, 0x69, 0x0A, 0xEC, 0x43, 0x2D, 0xEA },
            { 0x19, 0x6F, 0x51, 0x3B, 0xCD, 0x21, 0x5D, 0x88, 0x65, 0xBD, 0x44, 0xEF, 0x34, 0x85, 0x76, 0x21, 0x96, 0xF5, 0x13, 0xBC, 0xD2, 0x15, 0xD8, 0x86, 0x5B, 0xD4, 0x4E, 0xF3, 0x48, 0x57, 0x62 },
            { 0x13, 0xBC, 0xD2, 0x15, 0xD8, 0x86, 0x5B, 0xD4, 0x4E, 0xF3, 0x48, 0x57, 0x62, 0x19, 0x6F, 0x51, 0x3B, 0xCD, 0x21, 0x5D, 0x88, 0x65, 0xBD, 0x44, 0xEF, 0x34, 0x85, 0x76, 0x21, 0x96, 0xF5 },
            { 0x0C, 0xB7, 0xA8, 0x9D, 0xE6, 0x90, 0xAE, 0xC4, 0x32, 0xDE, 0xA2, 0x77, 0x9A, 0x42, 0xBB, 0x10, 0xCB, 0x7A, 0x89, 0xDE, 0x69, 0x0A, 0xEC, 0x43, 0x2D, 0xEA, 0x27, 0x79, 0xA4, 0x2B, 0xB1 },
            { 0x79, 0xA4, 0x2B, 0xB1, 0x0C, 0xB7, 0xA8, 0x9D, 0xE6, 0x90, 0xAE, 0xC4, 0x32, 0xDE, 0xA2, 0x77, 0x9A, 0x42, 0xBB, 0x10, 0xCB, 0x7A, 0x89, 0xDE, 0x69, 0x0A, 0xEC, 0x43, 0x2D, 0xEA, 0x27 }
        };

        // Scrambling sequence for a TDMA frame number (f = frame mod 8). Confirmed on a real capture: the burst that
        // carries the Qt (frame 8) is scrambled with row 0.
        inline const uint8_t* scrambler(unsigned frame) { return scrt[frame & 7]; }

        static const uint16_t crcTable[16] = {
            0x0000, 0x0589, 0x0b12, 0x0e9b, 0x1624, 0x13ad, 0x1d36, 0x18bf,
            0x2c48, 0x29c1, 0x275a, 0x22d3, 0x3a6c, 0x3fe5, 0x317e, 0x34f7
        };

        // A-field CRC (R-CRC, x^16+x^10+x^8+x^7+x^3+1)
        inline uint16_t calcRCRC(const uint8_t* data, unsigned len) {
            uint16_t crc = 0x0000;
            while (len--) {
                unsigned idx = (crc >> 12) ^ (*data >> 4);
                crc = crcTable[idx & 0x0f] ^ (crc << 4);
                idx = (crc >> 12) ^ (*data >> 0);
                crc = crcTable[idx & 0x0f] ^ (crc << 4);
                data++;
            }
            return crc ^ 0x0001;
        }

        // B-field CRC (X-CRC, 4 bits) over the 80 "test bits" of the descrambled B-field (the caller descrambles)
        inline uint8_t calcXCRC(const uint8_t* bField) {
            uint8_t rbits[10];
            const uint8_t gp = 0x10;
            uint8_t crc, next;
            uint32_t i, j;
            uint8_t rbyte = 0;
            uint32_t rbitCnt = 0, rbyteCnt = 0;

            memset(rbits, 0, sizeof(rbits));
            for (i = 0; i <= (83 - 4); i++) {
                uint32_t bi = i + 48 * (1 + (i >> 4));
                uint32_t nb = bi >> 3;
                uint32_t bw = bField[nb];

                rbyte <<= 1;
                rbyte |= (bw >> (7 - (bi - (nb << 3)))) & 1;

                if (++rbitCnt == 8) {
                    rbits[rbyteCnt++] = rbyte;
                    rbitCnt = 0;
                }
            }

            crc = rbits[0];
            i = 0;
            while (i < 10) {
                if (i < (10 - 1)) { next = rbits[i + 1]; }
                else { next = 0; }
                i++;
                j = 0;
                while (j < 8) {
                    while (!(crc & 0x80)) {
                        crc <<= 1;
                        crc |= !!(next & 0x80);
                        next <<= 1;
                        j++;
                        if (j > 7) { break; }
                    }
                    if (j > 7) { break; }
                    crc <<= 1;
                    crc |= !!(next & 0x80);
                    next <<= 1;
                    j++;
                    crc ^= gp;
                }
            }
            return crc >> 4;
        }

        // MSB-first bit packing: bits[0] becomes the MSB of out[0]
        inline void packBits(const uint8_t* bits, int count, uint8_t* out) {
            memset(out, 0, (count + 7) / 8);
            for (int i = 0; i < count; i++) {
                out[i >> 3] |= (uint8_t)((bits[i] & 1) << (7 - (i & 7)));
            }
        }
    }

    // Encryption as signalled by the MAC layer. Encryption control is an "Mt" (MAC control) tail whose first nibble is
    // 0101 (EN 300 175-3 clause 7.2.5.7). Messages are exchanged request -> confirm -> grant.
    enum EncSignalling : uint8_t {
        ENC_SIG_NONE = 0,    // no encryption control message seen
        ENC_SIG_STARTING,    // "start encryption: request" seen
        ENC_SIG_ACTIVE,      // start encryption confirmed or granted (the connection is, or is about to be, ciphered)
        ENC_SIG_STOPPED      // "stop encryption" confirmed or granted
    };

    // What the decoded voice payload looks like. Voice (and idle line noise) gives strongly skewed ADPCM code
    // statistics; ciphertext is uniformly random. The B-field is also descrambled with the 7 other possible phases of
    // the scrambler: if the data turns out to be structured with one of them, the frame number is off (a decoding
    // problem), which is not the same thing as encryption.
    enum PayloadClass : uint8_t {
        PAYLOAD_UNKNOWN = 0,     // not enough good voice frames yet
        PAYLOAD_STRUCTURED,      // looks like voice / line noise: clear call
        PAYLOAD_RANDOM,          // random under every scrambler phase: ciphered (or garbage that passes the CRC)
        PAYLOAD_PHASE_MISMATCH   // structured under another scrambler phase: frame number wrong, not encrypted
    };

    struct VoiceAnalysis {
        PayloadClass payload = PAYLOAD_UNKNOWN;
        EncSignalling signalled = ENC_SIG_NONE;
        float usedScore = 0.0f;      // structure score with the scrambler phase in use (0 = uniform random, 15 = one code only)
        float bestScore = 0.0f;      // best structure score among the other 7 phases
        int bestPhase = 0;           // frame number offset (1..7) that gave bestScore
        int frames = 0;              // good voice frames in the analysis window

        // True if the call is flagged as encrypted by signalling and/or by the payload statistics
        bool encrypted() const { return signalled == ENC_SIG_ACTIVE || payload == PAYLOAD_RANDOM; }
    };

    struct PartInfo {
        bool active = false;
        bool voicePresent = false;
        uint64_t lastVoiceTick = 0;             // decoder burst counter at the last burst with a voice header
        bool partIdReceived = false;
        bool qtReceived = false;
        bool rfpFrameNumberCorrected = false;   // frame number was already taken from the paired RFP
        uint8_t frameNumber = 0;
        uint64_t rxSeq = 0;
        uint8_t partId[5] = { 0, 0, 0, 0, 0 };
        PartType type = PART_RFP;
        uint64_t packetCount = 0;
        uint64_t aFieldBadCrcCount = 0;
        int pair = -1;                          // index of the paired part (RFP <-> PP with the same ID), or -1
        EncSignalling encSignalling = ENC_SIG_NONE;
        uint8_t encCommand = 0;                 // last encryption control command nibble (valid if encMsgCount > 0)
        uint16_t encKeyIndex = 0;               // cipher key index from "start encryption with key index"
        uint32_t encMsgCount = 0;               // encryption control messages seen
    };

    // -----------------------------------------------------------------------
    // Burst decoder: A-field parsing, part bookkeeping and voice extraction.
    // -----------------------------------------------------------------------
    class PacketDecoder {
    public:
        PacketDecoder() { reset(); }

        void reset() {
            std::lock_guard<std::mutex> lck(mtx);
            for (int i = 0; i < MAX_PARTS; i++) { parts[i] = PartInfo(); }
            g721.reset();
            resetAnalysis();
            selected = 0;
            revision++;
            pcm.clear();
            framesGood = framesBad = framesMuted = 0;
        }

        // Voice decoded since the last call. Each decoded burst yields 80 samples @ 8 kHz; a frame with a failed
        // CRC yields 80 zeros. Call from the same thread that feeds the receiver.
        std::vector<int16_t> pcm;

        void selectPart(int rxId) {
            std::lock_guard<std::mutex> lck(mtx);
            selectLocked(rxId);
        }

        int getSelectedPart() {
            std::lock_guard<std::mutex> lck(mtx);
            return selected;
        }

        void setAutoSelect(bool en) { autoSelect = en; }

        // Copy of the part table for the UI
        void getParts(PartInfo* out, int& selectedOut) {
            std::lock_guard<std::mutex> lck(mtx);
            for (int i = 0; i < MAX_PARTS; i++) { out[i] = parts[i]; }
            selectedOut = selected;
        }

        void getStats(uint64_t& good, uint64_t& bad) {
            good = framesGood;
            bad = framesBad;
        }

        uint64_t getMutedFrames() { return framesMuted; }

        // Encryption verdict for the part that is being listened to
        VoiceAnalysis getVoiceAnalysis() {
            std::lock_guard<std::mutex> lck(mtx);
            return analyseLocked();
        }

        // Replace the audio of frames flagged as encrypted by silence (otherwise the ciphertext is decoded to loud noise)
        void setMuteEncrypted(bool en) { muteEncrypted = en; }

        // Part went quiet (reported by the receiver)
        void partLost(uint32_t rxId) {
            if (rxId >= (uint32_t)MAX_PARTS) { return; }
            std::lock_guard<std::mutex> lck(mtx);
            PartInfo& p = parts[rxId];
            p.active = false;
            p.voicePresent = false;
            p.qtReceived = false;
            p.encSignalling = ENC_SIG_NONE;
            if ((int)rxId == selected) { resetAnalysis(); }
            if (p.pair >= 0) {
                PartInfo& other = parts[p.pair];
                if (p.type == PART_RFP) { other.voicePresent = false; }
                other.pair = -1;
                p.pair = -1;
            }
            revision++;
        }

        void onBurst(const Burst& b) {
            if (b.partRxId >= (uint32_t)MAX_PARTS) { return; }
            std::lock_guard<std::mutex> lck(mtx);
            burstTick++;

            const int rxId = (int)b.partRxId;
            PartInfo& cur = parts[rxId];
            const PartType ptype = b.type;

            if (cur.active) {
                uint64_t seqDiff = (b.rxSeq - cur.rxSeq) & 0x1F;

                if (ptype == PART_RFP) {
                    cur.frameNumber = (uint8_t)((cur.frameNumber + seqDiff) & 0xF);
                    // The RFP defines the multiframe timing, so hand the frame number to its pair
                    if (cur.pair >= 0) {
                        parts[cur.pair].frameNumber = cur.frameNumber;
                        parts[cur.pair].rfpFrameNumberCorrected = true;
                    }
                }
                else {
                    if (cur.rfpFrameNumberCorrected) { cur.rfpFrameNumberCorrected = false; }
                    else { cur.frameNumber = (uint8_t)((cur.frameNumber + seqDiff) & 0xF); }
                }
                cur.rxSeq = b.rxSeq;
                cur.packetCount++;
            }
            else {
                cur = PartInfo();
                cur.active = true;
                cur.rxSeq = b.rxSeq;
                cur.type = ptype;
                revision++;
            }

            // Look for the RFP that belongs to a PP (same part ID)
            if (cur.pair < 0 && cur.type == PART_PP && cur.partIdReceived) {
                for (int i = 0; i < MAX_PARTS; i++) {
                    if (i != rxId && parts[i].active && sameId(cur.partId, parts[i].partId)) {
                        cur.pair = i;
                        parts[i].pair = rxId;
                        revision++;
                    }
                }
            }

            // A-field
            uint8_t aField[A_FIELD_BITS / 8];
            detail::packBits(b.bits, A_FIELD_BITS, aField);
            const EncSignalling encBefore = cur.encSignalling;
            decodeAField(cur, aField);

            // Encryption is a property of the connection, so the paired RFP/PP shares it
            if (cur.encSignalling != encBefore && cur.pair >= 0) { parts[cur.pair].encSignalling = cur.encSignalling; }

            if (ptype == PART_RFP && cur.qtReceived && cur.pair >= 0) { parts[cur.pair].qtReceived = true; }

            if (cur.voicePresent) { cur.lastVoiceTick = burstTick; }

            // "Follow active call": stay on the part that is being listened to (a single burst without a voice header
            // or a short dropout must not make it jump to the other side of the call). Only when it has been without
            // voice for a while, or is gone, move on, preferring a part of the same kind (base or handset).
            if (autoSelect) {
                const PartInfo& s = parts[selected];
                const bool selOk = s.active && s.qtReceived && (burstTick - s.lastVoiceTick) < AUTO_STALE_BURSTS;
                if (!selOk) {
                    int pick = -1;
                    for (int pass = 0; pass < 2 && pick < 0; pass++) {
                        for (int i = 0; i < MAX_PARTS; i++) {
                            const PartInfo& c = parts[i];
                            if (i == selected || !c.active || !c.voicePresent || !c.qtReceived) { continue; }
                            if (burstTick - c.lastVoiceTick >= AUTO_STALE_BURSTS) { continue; }
                            if (pass == 0 && c.type != preferredType) { continue; }
                            pick = i;
                            break;
                        }
                    }
                    if (pick >= 0) { selectLocked(pick); }
                }
            }

            // B-field (voice) of the selected part
            if (rxId == selected && cur.active && cur.voicePresent && cur.qtReceived) {
                uint8_t bField[B_FIELD_BITS / 8];
                detail::packBits(b.bits + A_FIELD_BITS, B_FIELD_BITS, bField);

                const uint8_t* xb = b.bits + A_FIELD_BITS + B_FIELD_BITS;
                uint8_t xField = (uint8_t)(((xb[0] & 1) << 3) | ((xb[1] & 1) << 2) | ((xb[2] & 1) << 1) | (xb[3] & 1));

                // The X-CRC is calculated over the B-field *after* descrambling, and the X-field itself is scrambled with
                // the continuing scrambler sequence (byte 40 of the 31 byte pattern, high nibble). Verified on a real
                // capture: with this order every burst of a base station passes with the expected frame number sequence;
                // checking the CRC on the still scrambled bits passes only ~1 in 16 (chance level).
                const uint8_t* whX = detail::scrambler(cur.frameNumber);
                uint8_t plain[B_FIELD_BITS / 8];
                for (int i = 0; i < B_FIELD_BITS / 8; i++) { plain[i] = bField[i] ^ whX[i % 31]; }
                const uint8_t xDescr = (uint8_t)(xField ^ (whX[(B_FIELD_BITS / 8) % 31] >> 4));

                // On the captured handset (PP) bursts the X-CRC never matched although the A-field CRC and the eye were
                // clean and the descrambled payload looked like valid ADPCM; the cause is unknown. For PP bursts the
                // X-CRC therefore does not gate the audio (the 16 bit A-field CRC still has to pass).
                if (ptype == PART_RFP) { voteScramblerPhase(bField, xField, cur.frameNumber); }
                const bool xOk = (detail::calcXCRC(plain) == xDescr) || (ptype == PART_PP && PP_IGNORE_XCRC);
                if (xOk) {
                    // Note: the X-CRC is not encrypted, so a ciphered frame normally passes it as well
                    accumulateAnalysis(bField, cur.frameNumber);
                    if (muteEncrypted && analyseLocked().encrypted()) {
                        for (int i = 0; i < VOICE_SAMPLES_PER_FRAME; i++) { pcm.push_back(0); }
                        framesMuted++;
                    }
                    else {
                        const uint8_t* wh = detail::scrambler(cur.frameNumber);
                        for (int i = 0; i < B_FIELD_BITS / 8; i++) {
                            uint8_t d = bField[i] ^ wh[i % 31];
                            pcm.push_back(g721.decode((d >> 4) & 0xF));
                            pcm.push_back(g721.decode(d & 0xF));
                        }
                    }
                    framesGood++;
                }
                else {
                    for (int i = 0; i < VOICE_SAMPLES_PER_FRAME; i++) { pcm.push_back(0); }
                    framesBad++;
                }
            }
        }

        std::atomic<uint32_t> revision{ 0 };   // incremented whenever the part table changes in a visible way
        std::atomic<bool> autoSelect{ true };

    private:
        // Payload statistics. For every good voice frame the B-field is descrambled with all 8 phases of the scrambler
        // (the one in use, plus 7 hypotheses "the frame number is off by d") and the 4 bit ADPCM code histogram of each is
        // kept in an exponentially decaying window (about 70 frames = 0.7 s).
        //
        // structure score = 16 * sum((p_i - 1/16)^2): 0 for uniformly random codes, 15 if only one code occurs.
        // Calibrated offline on G.726 32 kbit/s streams made with ffmpeg from espeak speech (also with background noise),
        // idle line noise at -70..-10 dBFS, loud white noise, tones, DTMF and digital silence: the window score of every
        // clear stream was >= 0.17 (speech >= 0.5). Simulated ciphertext (the same streams XORed with random bytes, plus
        // pure random data) stayed <= 0.01 over ~26,000 windows, and a wrong scrambler phase on clear data scored <= 0.1.
        // The thresholds sit in the gap. This is synthetic speech through one encoder, not captures of real handsets.
        static constexpr float ANALYSIS_DECAY = 0.985f;
        static constexpr int ANALYSIS_MIN_FRAMES = 32;
        static constexpr float STRUCTURED_MIN = 0.05f;
        static constexpr float MISMATCH_RATIO = 4.0f;
        static constexpr float CRC_VOTE_DECAY = 0.95f;
        static constexpr uint64_t AUTO_STALE_BURSTS = 400;   // bursts, i.e. a few seconds with a call on air
        uint64_t burstTick = 0;
        PartType preferredType = PART_RFP;
        float crcVotes[8] = { 0 };
        float crcBursts = 0.0f;

        void resetAnalysis() {
            memset(winHist, 0, sizeof(winHist));
            analysisFrames = 0;
            memset(crcVotes, 0, sizeof(crcVotes));
            crcBursts = 0.0f;
        }

        // The X-CRC only matches with the right scrambler phase, so a frame number that is off (missed bursts, wrong
        // multiframe sync) shows up as X-CRC failures that pass again under another offset d. Count, with decay, how
        // often each offset d = 0..7 passes.
        void voteScramblerPhase(const uint8_t* bField, uint8_t xField, uint8_t frameNumber) {
            crcBursts = crcBursts * CRC_VOTE_DECAY + 1.0f;
            for (int d = 0; d < 8; d++) {
                const uint8_t* wh = detail::scrambler(frameNumber + d);
                uint8_t plain[B_FIELD_BITS / 8];
                for (int i = 0; i < B_FIELD_BITS / 8; i++) { plain[i] = bField[i] ^ wh[i % 31]; }
                const bool ok = detail::calcXCRC(plain) == (uint8_t)(xField ^ (wh[(B_FIELD_BITS / 8) % 31] >> 4));
                crcVotes[d] = crcVotes[d] * CRC_VOTE_DECAY + (ok ? 1.0f : 0.0f);
            }
        }

        void accumulateAnalysis(const uint8_t* bField, uint8_t frameNumber) {
            for (int d = 0; d < 8; d++) {
                const uint8_t* wh = detail::scrambler(frameNumber + d);
                float cnt[16] = { 0 };
                for (int i = 0; i < B_FIELD_BITS / 8; i++) {
                    uint8_t v = bField[i] ^ wh[i % 31];
                    cnt[v >> 4] += 1.0f;
                    cnt[v & 0xF] += 1.0f;
                }
                for (int k = 0; k < 16; k++) { winHist[d][k] = winHist[d][k] * ANALYSIS_DECAY + cnt[k]; }
            }
            if (analysisFrames < 1000000) { analysisFrames++; }
        }

        static float structureScore(const float* h) {
            float n = 0.0f;
            for (int k = 0; k < 16; k++) { n += h[k]; }
            if (n <= 0.0f) { return 0.0f; }
            float s = 0.0f;
            for (int k = 0; k < 16; k++) {
                float p = h[k] / n - 1.0f / 16.0f;
                s += p * p;
            }
            return 16.0f * s;
        }

        VoiceAnalysis analyseLocked() const {
            VoiceAnalysis a;
            a.signalled = parts[selected].active ? parts[selected].encSignalling : ENC_SIG_NONE;
            a.frames = analysisFrames;

            // X-CRC fails with the frame number in use but passes consistently with another one: decoder desync
            if (crcBursts >= 8.0f && crcVotes[0] < 0.3f * crcBursts) {
                int bd = 1;
                for (int d = 2; d < 8; d++) { if (crcVotes[d] > crcVotes[bd]) { bd = d; } }
                if (crcVotes[bd] >= 0.6f * crcBursts) {
                    a.payload = PAYLOAD_PHASE_MISMATCH;
                    a.bestPhase = bd;
                    return a;
                }
            }
            if (analysisFrames < ANALYSIS_MIN_FRAMES) { return a; }

            a.usedScore = structureScore(winHist[0]);
            for (int d = 1; d < 8; d++) {
                float s = structureScore(winHist[d]);
                if (s > a.bestScore) {
                    a.bestScore = s;
                    a.bestPhase = d;
                }
            }

            if (a.bestScore >= STRUCTURED_MIN && a.bestScore >= MISMATCH_RATIO * a.usedScore) { a.payload = PAYLOAD_PHASE_MISMATCH; }
            else if (a.usedScore >= STRUCTURED_MIN) { a.payload = PAYLOAD_STRUCTURED; }
            else { a.payload = PAYLOAD_RANDOM; }
            return a;
        }

        static bool sameId(const uint8_t* a, const uint8_t* b) {
            return memcmp(a, b, 5) == 0;
        }

        void selectLocked(int rxId) {
            if (rxId < 0 || rxId >= MAX_PARTS || rxId == selected) { return; }
            selected = rxId;
            preferredType = parts[rxId].type;
            g721.reset();          // ADPCM state belongs to one voice stream
            resetAnalysis();
            revision++;
        }

        // Encryption control message (Mt tail, first nibble 0101). Second nibble: bits a12-a13 = command (00 start,
        // 01 stop, 10 start with cipher key index, 11 reserved), bits a14-a15 = 00 request, 01 confirm, 10 grant (11 =
        // reject for "start with key index"). With a key index, bits a16-a31 carry it.
        void decodeEncryptionControl(PartInfo& cur, const uint8_t* f) {
            const uint8_t cmd = f[1] & 0x0F;
            const uint8_t kind = cmd >> 2;
            const uint8_t step = cmd & 3;
            if (kind == 3) { return; }

            const EncSignalling before = cur.encSignalling;
            cur.encCommand = cmd;
            cur.encMsgCount++;
            if (kind == 2) { cur.encKeyIndex = (uint16_t)((f[2] << 8) | f[3]); }

            if (kind == 0 || kind == 2) {                    // start encryption
                if (step == 0) { if (cur.encSignalling != ENC_SIG_ACTIVE) { cur.encSignalling = ENC_SIG_STARTING; } }
                else if (step == 1 || step == 2) { cur.encSignalling = ENC_SIG_ACTIVE; }
                else if (kind == 2) { cur.encSignalling = ENC_SIG_NONE; }          // rejected
            }
            else if (step == 1 || step == 2) {               // stop encryption (confirmed / granted)
                cur.encSignalling = ENC_SIG_STOPPED;
            }
            if (cur.encSignalling != before) { revision++; }
        }

        // Returns true if the A-field CRC was good
        bool decodeAField(PartInfo& cur, const uint8_t* f) {
            uint16_t rcrc = (uint16_t)((uint16_t)f[6] << 8 | f[7]);
            if (detail::calcRCRC(f, 6) != rcrc) {
                cur.aFieldBadCrcCount++;
                return false;
            }

            uint8_t header = f[0];
            switch ((header >> 5) & 0x07) {
            case 3:   // Nt: identities information, contains the part ID
                if (!cur.partIdReceived || memcmp(cur.partId, f + 1, 5) != 0) { revision++; }
                memcpy(cur.partId, f + 1, 5);
                cur.partIdReceived = true;
                break;
            case 4:   // Qt: multiframe synchronisation and system information, sent in frame number 8
                cur.frameNumber = 8;
                cur.qtReceived = true;
                break;
            case 6:   // Mt: MAC control. Only encryption control is interpreted here
                if ((f[1] >> 4) == 0x5) { decodeEncryptionControl(cur, f); }
                break;
            default:
                break;
            }

            // The B-field type bits say whether this slot carries a (full slot, unprotected) voice B-field
            bool voice = (((header >> 1) & 7) == 0);
            if (voice != cur.voicePresent) {
                cur.voicePresent = voice;
                revision++;
            }
            return true;
        }

        std::mutex mtx;
        PartInfo parts[MAX_PARTS];
        int selected = 0;
        G721Decoder g721;
        std::atomic<uint64_t> framesGood{ 0 };
        std::atomic<uint64_t> framesBad{ 0 };
        std::atomic<uint64_t> framesMuted{ 0 };
        std::atomic<bool> muteEncrypted{ false };

        float winHist[8][16];
        int analysisFrames = 0;
    };
}

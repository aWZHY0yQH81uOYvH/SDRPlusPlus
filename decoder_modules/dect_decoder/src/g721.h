#pragma once
// ---------------------------------------------------------------------------
// G.721 / G.726 (32 kbit/s) ADPCM decoder.
//
// DECT's mandatory voice codec (GAP profile) is G.726 at 32 kbit/s, which is
// bit-compatible with the older CCITT G.721. The original gr-dect2 flowgraph
// uses GNU Radio's `vocoder_g721_decode_bs` block for this; SDR++ has no such
// block, so the decoder is implemented here.
//
// The algorithm follows the public-domain reference implementation published
// by Sun Microsystems, Inc. ("provided for unrestricted use"), restricted to
// the 32 kbit/s decoder path and re-expressed as a small self-contained C++
// class with fixed-width integer types.
//
// Input : one 4-bit ADPCM code word per call
// Output: 16-bit linear PCM sample @ 8 kHz
// ---------------------------------------------------------------------------
#include <stdint.h>

namespace dect {
    class G721Decoder {
    public:
        G721Decoder() { reset(); }

        void reset() {
            yl = 34816;
            yu = 544;
            dms = 0;
            dml = 0;
            ap = 0;
            for (int i = 0; i < 2; i++) {
                a[i] = 0;
                pk[i] = 0;
                sr[i] = 32;
            }
            for (int i = 0; i < 6; i++) {
                b[i] = 0;
                dq[i] = 32;
            }
            td = 0;
        }

        // Decode a single 4-bit code word. Returns a 16-bit linear PCM sample.
        int16_t decode(uint8_t code) {
            code &= 0x0F;

            int16_t sezi = predictorZero();
            int16_t sez = sezi >> 1;
            int16_t sei = sezi + predictorPole();
            int16_t se = sei >> 1;                        // estimated signal

            int16_t y = stepSize();                       // quantizer step size

            int16_t dqv = reconstruct(code & 8, dqlnTab[code], y);

            // reconstructed signal
            int16_t srv = (dqv < 0) ? (se - (dqv & 0x3FFF)) : (se + dqv);

            int16_t dqsez = srv - se + sez;               // pole prediction diff.

            update(y, witab[code] * 32, fitab[code], dqv, srv, dqsez);

            return (int16_t)(srv * 4);                    // sr has 14-bit range
        }

    private:
        static const int16_t* power2() {
            static const int16_t t[15] = { 1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800, 0x1000, 0x2000, 0x4000 };
            return t;
        }

        static int quan(int val, const int16_t* table, int size) {
            int i;
            for (i = 0; i < size; i++) {
                if (val < *table++) { break; }
            }
            return i;
        }

        // Floating point multiply used by the predictor
        static int fmult(int an, int srn) {
            int16_t anmag, anexp, anmant;
            int16_t wanexp, wanmant;
            int16_t retval;

            anmag = (an > 0) ? an : ((-an) & 0x1FFF);
            anexp = quan(anmag, power2(), 15) - 6;
            anmant = (anmag == 0) ? 32 : (anexp >= 0) ? (anmag >> anexp) : (anmag << -anexp);
            wanexp = anexp + ((srn >> 6) & 0xF) - 13;

            wanmant = (anmant * (srn & 077) + 0x30) >> 4;
            retval = (wanexp >= 0) ? ((wanmant << wanexp) & 0x7FFF) : (wanmant >> -wanexp);

            return ((an ^ srn) < 0) ? -retval : retval;
        }

        int predictorZero() const {
            int sezi = fmult(b[0] >> 2, dq[0]);
            for (int i = 1; i < 6; i++) {
                sezi += fmult(b[i] >> 2, dq[i]);
            }
            return sezi;
        }

        int predictorPole() const {
            return fmult(a[1] >> 2, sr[1]) + fmult(a[0] >> 2, sr[0]);
        }

        int stepSize() const {
            if (ap >= 256) { return yu; }
            int y = yl >> 6;
            int dif = yu - y;
            int al = ap >> 2;
            if (dif > 0) {
                y += (dif * al) >> 6;
            }
            else if (dif < 0) {
                y += (dif * al + 0x3F) >> 6;
            }
            return y;
        }

        static int reconstruct(int sign, int dqln, int y) {
            int16_t dql = dqln + (y >> 2);                // log of dq magnitude
            if (dql < 0) {
                return sign ? -0x8000 : 0;
            }
            int16_t dex = (dql >> 7) & 15;
            int16_t dqt = 128 + (dql & 127);
            int16_t dqr = (dqt << 7) >> (14 - dex);
            return sign ? (dqr - 0x8000) : dqr;
        }

        void update(int y, int wi, int fi, int dqv, int srv, int dqsez) {
            int cnt;
            int16_t mag, exp;
            int16_t a2p = 0;
            int16_t a1ul;
            int16_t pks1;
            int16_t fa1;
            int8_t tr;
            int16_t ylint, thr2, dqthr;
            int16_t ylfrac, thr1;
            int16_t pk0;

            pk0 = (dqsez < 0) ? 1 : 0;

            mag = dqv & 0x7FFF;                           // prediction difference magnitude

            // TRANS
            ylint = yl >> 15;
            ylfrac = (yl >> 10) & 0x1F;
            thr1 = (32 + ylfrac) << ylint;
            thr2 = (ylint > 9) ? (31 << 10) : thr1;
            dqthr = (thr2 + (thr2 >> 1)) >> 1;            // 0.75 * thr2
            if (td == 0) { tr = 0; }
            else if (mag <= dqthr) { tr = 0; }
            else { tr = 1; }

            // Quantizer scale factor adaptation
            yu = y + ((wi - y) >> 5);
            if (yu < 544) { yu = 544; }
            else if (yu > 5120) { yu = 5120; }

            yl += yu + ((-yl) >> 6);

            // Adaptive predictor coefficients
            if (tr == 1) {
                a[0] = 0;
                a[1] = 0;
                for (cnt = 0; cnt < 6; cnt++) { b[cnt] = 0; }
            }
            else {
                pks1 = pk0 ^ pk[0];

                // update predictor pole a[1]
                a2p = a[1] - (a[1] >> 7);
                if (dqsez != 0) {
                    fa1 = pks1 ? a[0] : -a[0];
                    if (fa1 < -8191) { a2p -= 0x100; }
                    else if (fa1 > 8191) { a2p += 0xFF; }
                    else { a2p += fa1 >> 5; }

                    if (pk0 ^ pk[1]) {
                        // LIMC
                        if (a2p <= -12160) { a2p = -12288; }
                        else if (a2p >= 12416) { a2p = 12288; }
                        else { a2p -= 0x80; }
                    }
                    else if (a2p <= -12416) { a2p = -12288; }
                    else if (a2p >= 12160) { a2p = 12288; }
                    else { a2p += 0x80; }
                }

                a[1] = a2p;

                // update predictor pole a[0]
                a[0] -= a[0] >> 8;
                if (dqsez != 0) {
                    if (pks1 == 0) { a[0] += 192; }
                    else { a[0] -= 192; }
                }

                // LIMD
                a1ul = 15360 - a2p;
                if (a[0] < -a1ul) { a[0] = -a1ul; }
                else if (a[0] > a1ul) { a[0] = a1ul; }

                // update predictor zeros b[6]
                for (cnt = 0; cnt < 6; cnt++) {
                    b[cnt] -= b[cnt] >> 8;                // G.721 / 32 kbit/s
                    if (dqv & 0x7FFF) {
                        if ((dqv ^ dq[cnt]) >= 0) { b[cnt] += 128; }
                        else { b[cnt] -= 128; }
                    }
                }
            }

            for (cnt = 5; cnt > 0; cnt--) { dq[cnt] = dq[cnt - 1]; }

            // FLOAT A: convert dq[0] to 4-bit exp, 6-bit mantissa f.p.
            if (mag == 0) {
                dq[0] = (dqv >= 0) ? 0x20 : (int16_t)0xFC20;
            }
            else {
                exp = quan(mag, power2(), 15);
                dq[0] = (dqv >= 0) ? ((exp << 6) + ((mag << 6) >> exp)) : ((exp << 6) + ((mag << 6) >> exp) - 0x400);
            }

            sr[1] = sr[0];
            // FLOAT B: convert sr to 4-bit exp., 6-bit mantissa f.p.
            if (srv == 0) {
                sr[0] = 0x20;
            }
            else if (srv > 0) {
                exp = quan(srv, power2(), 15);
                sr[0] = (exp << 6) + ((srv << 6) >> exp);
            }
            else if (srv > -32768) {
                mag = -srv;
                exp = quan(mag, power2(), 15);
                sr[0] = (exp << 6) + ((mag << 6) >> exp) - 0x400;
            }
            else {
                sr[0] = (int16_t)0xFC20;
            }

            // DELAY A
            pk[1] = pk[0];
            pk[0] = pk0;

            // TONE
            if (tr == 1) { td = 0; }
            else if (a2p < -11776) { td = 1; }
            else { td = 0; }

            // Adaptation speed control
            dms += (fi - dms) >> 5;
            dml += (((fi << 2) - dml) >> 7);

            if (tr == 1) { ap = 256; }
            else if (y < 1536) { ap += (0x200 - ap) >> 4; }
            else if (td == 1) { ap += (0x200 - ap) >> 4; }
            else if (abs((dms << 2) - dml) >= (dml >> 3)) { ap += (0x200 - ap) >> 4; }
            else { ap += (-ap) >> 4; }
        }

        static int abs(int v) { return v < 0 ? -v : v; }

        // Tables for the 32 kbit/s mode
        static constexpr int16_t dqlnTab[16] = { -2048, 4, 135, 213, 273, 323, 373, 425, 425, 373, 323, 273, 213, 135, 4, -2048 };
        static constexpr int16_t witab[16] = { -12, 18, 41, 64, 112, 198, 355, 1122, 1122, 355, 198, 112, 64, 41, 18, -12 };
        static constexpr int16_t fitab[16] = { 0, 0, 0, 0x200, 0x200, 0x200, 0x600, 0xE00, 0xE00, 0x600, 0x200, 0x200, 0x200, 0, 0, 0 };

        // State
        int32_t yl;     // locked / steady state step size multiplier
        int16_t yu;     // unlocked / non-steady state step size multiplier
        int16_t dms;    // short term energy estimate
        int16_t dml;    // long term energy estimate
        int16_t ap;     // linear weighting coefficient of yl and yu
        int16_t a[2];   // pole predictor coefficients
        int16_t b[6];   // zero predictor coefficients
        int16_t pk[2];  // signs of previous two samples of partially reconstructed signal
        int16_t dq[6];  // previous 6 quantized difference samples (internal float format)
        int16_t sr[2];  // previous 2 reconstructed samples (internal float format)
        int8_t td;      // delayed tone detect
    };
}

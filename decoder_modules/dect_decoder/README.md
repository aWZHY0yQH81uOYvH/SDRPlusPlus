# DECT decoder for SDR++

Decodes **unencrypted DECT (GAP) voice**, shows the parts (base stations and handsets) on air, and tells you when a
call is **encrypted** (so you can tell a broken decode from a ciphered stream). It is a port of the
GNU Radio module [gr-dect2](https://github.com/pavelyazev/gr-dect2) by Pavel Yazev (GPLv3), reworked as a native
SDR++ module.

| gr-dect2 block / flowgraph element      | here                                                        |
|-----------------------------------------|-------------------------------------------------------------|
| resamplers + channel filter             | SDR++ VFO (4.608 MSPS = 4 samples/symbol, 1.38 MHz wide)    |
| `dect2_phase_diff`                      | `dect::PhaseDiff`                                           |
| `dect2_packet_receiver`                 | `dect::PacketReceiver`                                      |
| `dect2_packet_decoder`                  | `dect::PacketDecoder`                                       |
| `vocoder_g721_decode_bs`                | `dect::G721Decoder` (`g721.h`)                              |
| Qt console + "Select Part" chooser      | part table in the module menu, click a row to listen        |
| audio sink                              | SDR++ sink manager stream (8 kHz, resampled to your device) |

## Using it

1. Add a "DECT Decoder" instance in the module manager. A VFO 1.38 MHz wide appears on the waterfall.
2. Pick the band and a carrier and the VFO is tuned to it (or place the VFO yourself with "Manual").
   Europe: carriers 0-9 = 1897.344 MHz down to 1881.792 MHz in 1.728 MHz steps. US/Canada (DECT 6.0): 1921.536 to
   1928.448 MHz.
3. When a handset or base is transmitting, it shows up in the "Parts on air" table. A call appears as an RFP (base)
   and a PP (handset) with the same ID. "Follow active call" picks whichever part is carrying voice; clicking a row
   selects that part manually.
4. A PP can only be decoded while its paired RFP is also received (the RFP defines the multiframe timing).

Parts move between carriers, so finding a call can mean stepping through the carriers.

### Source requirements

* The SDR must tune 1.88-1.93 GHz and deliver at least **4.608 MSPS**. 9.216 or 18.432 MSPS are the cheapest because the
  ratio to 4.608 MSPS is an integer. The module shows a warning when the source rate is too low.
* Radios that tune this band and can run at these rates (HackRF, PlutoSDR, SDRplay, BladeRF, USRP) should work; none
  were tried for this port. RTL-SDR does not reach this band.
* In the synthetic self test the decode rate is ~100% down to about 14 dB SNR in the 1.4 MHz channel and falls off
  quickly below 11 dB. Real-world performance (multipath, drifting oscillators) is not measured.

## Building

Linux/macOS, no extra dependencies. The module is on by default (`-DOPT_BUILD_DECT_DECODER=OFF` to skip it).

macOS, following the main readme (the DECT module needs nothing beyond what SDR++ already needs):

```sh
mkdir build && cd build
cmake .. -DOPT_BUILD_AUDIO_SINK=OFF -DOPT_BUILD_PORTAUDIO_SINK=ON -DOPT_BUILD_NEW_PORTAUDIO_SINK=ON \
         -DUSE_BUNDLE_DEFAULTS=ON -DOPT_BUILD_DECT_DECODER=ON -DCMAKE_BUILD_TYPE=Release
make -j8
cd .. && sh make_macos_bundle.sh ./build ./SDR++.app
```

`make_macos_bundle.sh` copies `dect_decoder.dylib` into `SDR++.app/Contents/Plugins`. For an existing install you can
also just copy `build/decoder_modules/dect_decoder/dect_decoder.dylib` into that `Plugins` folder and restart.
Nothing in the module is architecture specific (plain C++17, no intrinsics), but it has not been built on macOS by the
author of this port; it was compiled and tested on Linux only.

## Self test (no SDR++ or radio needed)

`test/dect_selftest.cpp` synthesises a DECT signal (GFSK with BT 0.5, correct S/A/B/X fields, CRCs and scrambling;
an RFP, its PP and a second RFP), passes it through a frequency offset, a channel filter and noise, and runs the full
receive chain:

```sh
cd decoder_modules/dect_decoder/test
c++ -std=c++17 -O2 -I../src dect_selftest.cpp -o dect_selftest && ./dect_selftest
```

It checks that all parts are found and paired, that the audio of both the RFP and the PP is recovered bit-exactly,
that a part that stops transmitting is released, and prints the decode rate versus SNR.

The G.721 decoder is bit-exact with the reference implementation used by GNU Radio (`gr-vocoder/lib/g7xx`) over
1.8 million random and extreme code words.

## Encryption indicator

Two independent indicators feed the "Enc" column of the parts table and the "Link:" line:

1. **MAC signalling** (EN 300 175-3 clause 7.2.5.7). The A-field Mt tail "encryption control" (first nibble 0101) is
   parsed. START request / confirm / grant, and START with cipher key-index, set the part to *starting* and then
   *ACTIVE*; STOP confirm / grant sets *stopped*. The state of the RFP and its paired PP is shared. This needs the
   handshake to be received, so it fails for a call you join late.
2. **Payload statistics**. Cleartext G.726 codes are strongly non-uniform; ciphertext is uniform. A decayed histogram
   of the ADPCM codes over ~32+ good voice frames gives a structure score. The histogram is kept for all 8 scrambler
   (frame number) phases: if the code distribution is flat under the used phase *and* every other phase, the payload
   is "random" (likely encrypted). If it is flat under the used phase but structured under another one, the
   verdict is "decoder desync" (frame-number offset), which is a decoding problem and not encryption.

"Mute encrypted audio" (off by default) replaces audio flagged as encrypted by silence instead of loud noise.

Caveats, please read:

* Thresholds were calibrated only on synthetic data (speech synthesised with espeak-ng and encoded by one G.726
  encoder, ciphertext simulated by a random keystream). Real handsets, noise, DTMF/silence-heavy calls and music on
  hold have not been tried and may be misclassified; silence alone is structured, not random.
* The simulator assumes the X-CRC covers the data as transmitted after encryption (encryption sits before the
  scrambler, and the X-CRC is calculated on the descrambled B-field). This has not been confirmed on a real encrypted
  capture. If it were wrong, ciphered frames would show up as bad CRC and the payload indicator would stay at
  "Analysing..."; the MAC indicator would still work.
* A scrambler/frame-number error is recognised from the X-CRC: it fails with the frame number in use but passes with
  another offset. The Link line then says "Decoder desync (voice fits frame offset +N)" (base station side only).
* "ACTIVE" is set when the confirm/grant arrives; the exact frame at which ciphering begins is not derived.
  The state stays until a stop message or the part disappears.
* A DECT-compatible but non-standard (proprietary) cipher, or a link that never sends the MAC message in view, is
  only detectable by the payload statistics.

## Findings from a real capture

The 0.4 s capture of a US DECT 6.0 call (base + handset, idle line) showed that the first version of this port checked
the X-CRC wrongly: it is calculated over the *descrambled* B-field and the X-field itself is scrambled (with the
continuing sequence, byte 40 of the 31 byte pattern). With that fixed, every voice burst of the base station passes
(32/32, frame-number sequence matches the Qt multiframe marker) where before only ~1 in 16 did. The self-test now
generates its signal the same way.

**Handset (PP) bursts do not pass the X-CRC** in that capture, although their A-field CRC passes, the eye is wide open
(no bit decision closer than 0.8 rad to the threshold) and the descrambled payload decodes to plausible low level
microphone noise. The cause is not known. For PP bursts the X-CRC therefore does not gate the audio
(`PP_IGNORE_XCRC` in `dect_dsp.h`); a bad PP frame is only protected by the A-field CRC. If you see pops on the handset
side, that is the first place to look.

## Limits

* Narrowband GAP voice (G.726 at 32 kbit/s, full-slot) only. No data, no wideband (G.722) voice, no DECT NG.
* **Encrypted calls are not decoded** (there is no DSC/DSC2 support), but they are flagged, see below. Only
  unencrypted links such as many baby monitors give audio, as with gr-dect2.
* One part is listened to at a time. There is no carrier scanning or frequency tracking.
* The test signal in the self test is produced with the same CRC and scrambler tables as the decoder, which come from
  gr-dect2. They are therefore not independently checked against a real-world capture here.

## Responsible use

Only receive signals you are legally allowed to receive. Listening to other people's telephone calls is illegal in
many countries; the original project says the same. This is meant for your own equipment (baby monitors, your own
handsets), research and protocol analysis.

## Credits

* Pavel Yazev, gr-dect2 (packet receiver/decoder logic, CRC and scrambler tables, with scrambler table corrections by
  Jakub Hruska).
* `g721.h`: the G.721/G.726 32 kbit/s decoder follows the public Sun Microsystems reference implementation ("provided
  for unrestricted use"), as used by GNU Radio.

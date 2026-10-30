# Kestrel flat-six engine bank

The active bank (`engine.json` and `flat6-*.pcm`) uses excerpts from a real
**Porsche 911 SC, 1981, 3.0 litre flat-six**, recorded by **Pole Position Production**.
It is shared by Kestrel and Kestrel Turbo.

- Recording: `Porsche_911_SC_-81_t5_onboard_driving_constant_rpms_exhausts.wav`
- Collection: Sonniss #GameAudioGDC 2016, Pole Position Production — Porsche 911 SC 1981.
- [Producer's recording description](https://pole.se/the-iconic-sound-of-a-porsche-911-sc-from-1981/)
- [Official free bundle](https://sonniss.com/gameaudiogdc/)
- [Individual-file download mirror](https://ftpmirror.your.org/pub/misc/sonniss2016/individual/Pole%20Position%20Production%20-%20Porsche%20911%20SC%201981/Porsche_911_SC_-81_t5_onboard_driving_constant_rpms_exhausts.wav)
- Original SHA-256: `eacab443a8ece1b3f4874604f7dfd7337176648ebb1ab5a5e88cd9fcaf348aee`
- Retrieved: 2026-10-30.
- [Sonniss GDC bundle license](https://sonniss.com/gdc-bundle-license/), version 2.0,
  effective 2026-08-27. Copyright remains with the respective rights holders.

The license permits modification and use in personal/commercial games. These
assets are licensed game content, **not CC0 or part of the code's license**.
Distribute them within this game; do not redistribute them as a sound library,
sample pack, project template, or standalone sound files. The original long take
is kept in ignored `artifacts/audio/source/`, outside the shipped game assets.

## Editing and calibration

`tools/build_engine_audio.py` extracts distinct sections at six RPM regions,
resamples to mono 48 kHz, removes handling rumble below 30 Hz, rolls off above
12 kHz, matches levels with peak headroom, and chooses matching loop seams.
Each runtime PCM is signed 16-bit little-endian without a header. The manifest
records excerpt times, measured firing frequencies, overlap lengths and gains.
No samples from NFS or other games are used.

| Blend anchor | Load excerpt start | Coast excerpt start | Approx. source RPM, load/coast |
| --- | --- | --- | --- |
| 1,150 | 276 s | 276 s | 1,148 / 1,148 |
| 2,000 | 10 s | 100 s | 2,205 / 1,905 |
| 2,600 | 150 s | 134 s | 2,513 / 2,578 |
| 3,500 | 192 s | 235 s | 3,606 / 3,423 |
| 4,400 | 303 s | 363 s | 4,472 / 4,156 |
| 5,100 | 395 s | 450 s | 5,012 / 5,029 |

RPM is estimated from the six-cylinder firing harmonic, not synchronized
tachometer data. Load/coast labels identify heavier/lighter exhaust sections
selected from the same driving take; throttle state was not supplied by the
recordist. Idle shares one excerpt between both states. The last recording is
pitched upward for the game's 6,500 RPM redline; no dedicated 6,500 RPM capture
is claimed. This is a naturally aspirated SC recording, not a recording of a
930 Turbo. The turbo airflow layer is synthesized separately.

To reproduce the bank, download the original to the ignored artifacts folder,
then run from the repository root (Python requires NumPy/SciPy; ffmpeg required):

```sh
python3 tools/build_engine_audio.py artifacts/audio/source/porsche-911-sc-1981.wav
```

The older `engine-source.wav`, `roadster-engine.pcm`, `engine-load-source.wav`,
`engine-load.pcm`, `engine-coast-source.wav` and `engine-coast.pcm` are unused by
the active bank. Their original CC0/CC-BY-SA credits remain in the root README.

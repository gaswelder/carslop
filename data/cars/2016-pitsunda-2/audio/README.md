Engine recording: Mazda 2 1.3 Bifuel Sound & Acceleration
Author: Erwin Lindsen, 9 April 2012
Original video: https://www.youtube.com/watch?v=3AZSUqSdUBI
Audio source: https://commons.wikimedia.org/wiki/File:Mazda_2_-_engine_and_exhaust_sound_%28YouTube_3AZSUqSdUBI%29.opus
License: Creative Commons Attribution 3.0 Unported
https://creativecommons.org/licenses/by/3.0/

Commons lists this as CC BY 3.0, with verification of the original YouTube
license still pending. This older 1.3 Bifuel recording approximates the
2016 Pitsunda's engine.

Changes: excerpts converted to mono 48 kHz signed 16-bit PCM WAV,
high/low-pass filtered, pitch stabilized, level adjusted, and trimmed.
On rebuild, the builder bakes seam smoothing into the exported WAVs.
The revs use a tracked firing-frequency estimate followed by
complex demodulation to recover the actual firing phase. Manually identified
pitch guides keep the tracker on that ridge: louder upper harmonics must not
be mistaken for the firing frequency as the engine slows. Resampling that phase
to a constant rate removes the recorded rev's rise/fall. Loop periods contain
whole four-pulse engine cycles, with a four-pulse crossfade baked on rebuild,
so the seam
does not periodically cancel the engine's harmonics. A slow RMS envelope
reduces the recorded throttle swell while retaining individual pulses.
Complementary frequency bands then receive separate, smoothly varying gain
corrections. Their boundaries follow engine orders, with a 20 ms Gaussian RMS
envelope. This reduces repeating changes in bass/body/hiss that remain audible
even when total RMS is steady. It preserves recorded audio rather than adding
new oscillator tones.
Recordings are named `<rpm>-load.wav` and `<rpm>-coast.wav`, using actual
recorded RPM rounded to an integer. The game discovers them from their filenames
and sorts them by RPM. It loops each complete WAV end-to-start; asset authors
are responsible for smoothing its seam. Four-cylinder RPM is firing frequency
multiplied by 30. Load and coast use separate copies of each excerpt; the runtime
blends neighboring recordings on each axis and applies its throttle-dependent
filter/mix. The highest recording is pitched up to cover redline.

Idle preserves the last 300 ms of the first idle selection (34.55–34.85s),
without pitch stabilization. On rebuild, its exported seam uses a
waveform-matched 35 ms crossfade.
All higher bands use stationary revs rather than the later driving footage.
Source start/duration seconds for the 2400, 3600 and 5250 RPM bands respectively:
21.0/1.2, 19.4/1.1, 26.15/1.25. The highest band uses the crest of a later rev
to minimize the upward pitch shift at redline.

Rebuild with NumPy, SciPy and ffmpeg installed:

    python3 tools/build_engine_audio.py --pitsunda /path/to/original.opus

The builder retains the original recording outside the game assets. Load/coast
labels do not represent measured throttle telemetry; each RPM has two copies
of the same excerpt.

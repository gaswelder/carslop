# Engine recording

`engine-source.wav`: domasx2, **racing car engine sound loops**, `loop_0.wav`.
Source: https://opengameart.org/content/racing-car-engine-sound-loops
Download: https://opengameart.org/sites/default/files/loop_0.wav
License: CC0 1.0, https://creativecommons.org/publicdomain/zero/1.0/
The author describes the current upload as derived from a public-domain car
recording. Retrieved 2026-10-30. The other files in that pack are pitch variants
of the same recording; they are deliberately not presented as separate RPM captures.

`roadster-engine.pcm` is the runtime-loaded mono 48 kHz signed 16-bit little-endian
playback derivative. Recreate with:

```sh
ffmpeg -i data/cars/kestrel/audio/engine-source.wav -af 'highpass=f=28,volume=1.2' -ar 48000 -ac 1 -f s16le data/cars/kestrel/audio/roadster-engine.pcm
```

The Kestrel keeps this sample at idle, blending into separate load and overrun
recordings between 1150 and 2800 RPM. All three loops run continuously, with
interpolated playback and overlapping seams. The target firing pulse follows a
six-cylinder four-stroke engine. These are generic engine recordings, not a
recording of the specific heritage coupe. Dedicated mid-RPM recordings remain
the next improvement; the running loops are pitched down at intermediate RPM.
Muscle and Club retain their distinct procedural voices. Rolling noise and
tire scrub are procedural; rolling follows road speed and scrub resonance is
independent of engine RPM. Playback remains mono.

## Running recordings (separate license)

`engine-load-source.wav` / `engine-coast-source.wav`: `Acc_05570.wav` /
`Dec_05581.wav` from **Generic V8 Engine Sound**.
Sound made by DerMeehdrescher / Meehdrescher Studios.
Source: https://opengameart.org/content/generic-v8-engine-sound
Download: https://opengameart.org/sites/default/files/genericv8sound.zip
License: Creative Commons Attribution-ShareAlike 4.0 International,
https://creativecommons.org/licenses/by-sa/4.0/
Retrieved 2026-10-30. The originals and their modified PCM derivatives are
distributed under that license. Changes: mono downmix, 30 Hz high-pass filter,
raw signed 16-bit little-endian export; runtime pitch/filter/crossfade processing.
The generic V8 character is a temporary stand-in for the six-cylinder Kestrel.

Recreate each derivative with:

```sh
ffmpeg -i data/cars/kestrel/audio/engine-load-source.wav -af highpass=f=30 -ac 1 -ar 48000 -f s16le data/cars/kestrel/audio/engine-load.pcm
ffmpeg -i data/cars/kestrel/audio/engine-coast-source.wav -af highpass=f=30 -ac 1 -ar 48000 -f s16le data/cars/kestrel/audio/engine-coast.pcm
```

To render an offline driving audition (no audio device required):

```sh
mkdir -p artifacts/audio
NFS_AUDIO_PREVIEW=artifacts/audio/driving.wav cargo test recorded_driving_preview -- --nocapture
```

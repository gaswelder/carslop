#!/usr/bin/env python3
"""Build Kestrel's RPM bank from the Sonniss GDC 2016 Porsche 911 SC take.

Requires Python 3, NumPy, SciPy and ffmpeg. The original recording stays outside
the game assets; see README.md for its source/license.
Run: python3 tools/build_engine_audio.py path/to/original.wav
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from scipy import signal

ROOT = Path(__file__).resolve().parents[1]
RATE = 48000
SOURCE_SHA256 = "eacab443a8ece1b3f4874604f7dfd7337176648ebb1ab5a5e88cd9fcaf348aee"
# Nominal blend RPM, then (source start seconds, approximate firing Hz) for
# heavier/lighter exhaust sections. These are distinct excerpts, not copies
# pitched to fabricate extra RPM captures. Load labels are inferred from tone/level,
# not measured throttle telemetry. Idle uses the same excerpt for both states.
EXCERPTS = [
    (1150, (276.0, 57.5), (276.0, 57.5)),
    (2000, (10.0, 110.0), (100.0, 96.0)),
    (2600, (150.0, 125.0), (134.0, 129.0)),
    (3500, (192.0, 180.0), (235.0, 171.0)),
    (4400, (303.0, 223.5), (363.0, 207.0)),
    (5100, (395.0, 251.0), (450.0, 252.0)),
]


def firing_frequency(samples, approximate):
    """Measure the third crank order near an identified six-cylinder pulse."""
    spectrum = abs(np.fft.rfft(samples * np.hanning(len(samples)), RATE * 8))
    frequencies = np.fft.rfftfreq(RATE * 8, 1 / RATE)
    candidates = np.flatnonzero(abs(frequencies - approximate) < approximate * .045)
    peak = candidates[np.argmax(spectrum[candidates])]
    a, b, c = np.log(spectrum[peak - 1:peak + 2] + 1e-15)
    offset = .5 * (a - c) / (a - 2 * b + c)
    return float((peak + offset) / 8)


def extract(samples, start, approximate):
    # Retain the actual exhaust texture; remove DC/handling rumble and frequencies
    # which would alias when the highest band is pitched up toward redline.
    clip = samples[round(start * RATE):round((start + 2.0) * RATE)].astype(np.float64)
    if len(clip) != RATE * 2:
        raise ValueError(f"source is too short for excerpt at {start}s")
    clip = signal.sosfiltfilt(signal.butter(2, 30, 'highpass', fs=RATE, output='sos'), clip)
    clip = signal.sosfiltfilt(signal.butter(4, 12000, fs=RATE, output='sos'), clip)
    pulse = firing_frequency(clip, approximate)
    # One complete four-stroke engine cycle (six firing pulses) is crossfaded at
    # the seam. Search for a tail matching the head to reduce phase cancellation.
    overlap = round(6 * RATE / pulse)
    head = clip[:overlap]
    lo, hi = round(1.35 * RATE), min(round(1.85 * RATE), len(clip) - overlap)
    tails = clip[lo:hi + overlap - 1]
    dot = signal.correlate(tails, head, mode='valid', method='fft')
    energy = np.convolve(tails * tails, np.ones(overlap), mode='valid')
    error = energy + np.dot(head, head) - 2 * dot
    end = lo + int(np.argmin(error))
    clip = clip[:end + overlap]
    # Consistent source level lets crossfades preserve loudness. Keep peak
    # headroom instead of clipping transients to reach an RMS target.
    gain = min(.20 / np.sqrt(np.mean(clip * clip)), .85 / max(abs(clip)))
    pcm = np.rint(clip * gain * 32767).astype('<i2').tobytes()
    return pcm, pulse, overlap, end / RATE, gain


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT / 'data/cars/1964-kestrel/audio')
    args = parser.parse_args()
    digest = hashlib.sha256(args.source.read_bytes()).hexdigest()
    if digest != SOURCE_SHA256:
        parser.error('source SHA-256 does not match the documented Porsche recording')
    decoded = subprocess.run([
        'ffmpeg', '-v', 'error', '-i', str(args.source), '-ar', str(RATE),
        '-ac', '1', '-f', 'f32le', 'pipe:1',
    ], check=True, capture_output=True)
    samples = np.frombuffer(decoded.stdout, dtype='<f4')
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {'version': 1, 'sample_rate': RATE, 'bands': []}
    for rpm, loaded, coast in EXCERPTS:
        band = {'rpm': rpm}
        for state, (start, approximate) in [('load', loaded), ('coast', coast)]:
            pcm, pulse, overlap, duration, gain = extract(samples, start, approximate)
            filename = f'flat6-{rpm}-{state}.pcm'
            (args.output / filename).write_bytes(pcm)
            band[state] = {
                'file': filename, 'pulse_hz': round(pulse, 5), 'overlap': overlap,
                'source_start_seconds': start, 'repeat_seconds': round(duration, 6),
                'source_gain': round(gain, 6),
            }
            print(f'{filename}: measured {pulse:.3f} Hz ({pulse * 20:.0f} RPM), '
                  f'{duration:.3f}s repeat, {len(pcm)} bytes')
        manifest['bands'].append(band)
    (args.output / 'engine.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Build Kestrel's Porsche bank or Pitsunda's Mazda 2 bank.

Requires Python 3, NumPy, SciPy and ffmpeg. The original recording stays outside
the game assets; see README.md for its source/license.
Run: python3 tools/build_engine_audio.py path/to/original.wav
Pitsunda: python3 tools/build_engine_audio.py --pitsunda path/to/original.opus
"""
import argparse
import json
from pathlib import Path
import subprocess
import wave

import numpy
from scipy import ndimage
from scipy import signal

ROOT = Path(__file__).resolve().parents[1]
RATE = 48000


def write_wav(path, pcm):
    with wave.open(str(path), 'wb') as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(RATE)
        output.writeframes(pcm)


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
    spectrum = abs(numpy.fft.rfft(samples * numpy.hanning(len(samples)), RATE * 8))
    frequencies = numpy.fft.rfftfreq(RATE * 8, 1 / RATE)
    candidates = numpy.flatnonzero(abs(frequencies - approximate) < approximate * .045)
    peak = candidates[numpy.argmax(spectrum[candidates])]
    a, b, c = numpy.log(spectrum[peak - 1:peak + 2] + 1e-15)
    offset = .5 * (a - c) / (a - 2 * b + c)
    return float((peak + offset) / 8)


def extract(samples, start, approximate):
    # Retain the actual exhaust texture; remove DC/handling rumble and frequencies
    # which would alias when the highest band is pitched up toward redline.
    clip = samples[round(start * RATE):round((start + 2.0) * RATE)].astype(numpy.float64)
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
    energy = numpy.convolve(tails * tails, numpy.ones(overlap), mode='valid')
    error = energy + numpy.dot(head, head) - 2 * dot
    end = lo + int(numpy.argmin(error))
    clip = clip[:end + overlap]
    # Consistent source level lets crossfades preserve loudness. Keep peak
    # headroom instead of clipping transients to reach an RMS target.
    gain = min(.20 / numpy.sqrt(numpy.mean(clip * clip)), .85 / max(abs(clip)))
    pcm = numpy.rint(clip * gain * 32767).astype('<i2').tobytes()
    return pcm, pulse, overlap, end / RATE, gain


def level_engine_bands(clip, pulse):
    """Reduce repeated changes in tone that a broadband RMS check misses.

    A rev changes its spectral balance as well as its pitch. After removing
    pitch drift, that balance still swells on each loop. Complementary bands
    preserve the recorded waveform before gain correction; their boundaries
    sit between engine orders rather than through a firing harmonic. Smooth
    each band's level without flattening individual combustion pulses.
    """
    components = []
    previous = numpy.zeros_like(clip)
    for order in [.75, 1.25, 1.75, 2.25, 3.25, 4.25, 6.25, 9.25, 15.25, 24.25, 40.25]:
        cutoff = pulse * order
        if cutoff >= 14000:
            break
        low = signal.sosfiltfilt(signal.butter(4, cutoff, fs=RATE, output='sos'), clip)
        components.append(low - previous)
        previous = low
    components.append(clip - previous)
    steady = numpy.zeros_like(clip)
    for component in components:
        envelope = numpy.sqrt(ndimage.gaussian_filter1d(component * component, .02 * RATE))
        correction = numpy.clip(numpy.median(envelope) / numpy.maximum(envelope, 1e-8), .2, 5)
        steady += component * correction
    return steady


def stationary_rev(samples, start, duration, pitch_guide, rpm):
    """Resample a recorded rev onto firing phase, retaining its noisy texture.

    The dominant second crank order is tracked, then complex demodulation
    corrects its phase. Unlike a polynomial pitch fit alone, this keeps adjacent
    RPM banks in tune throughout the loop. Endpoints span complete four-pulse
    engine cycles, so the runtime overlap cannot cancel a fractional cycle.
    """
    clip = samples[round(start * RATE):round((start + duration) * RATE)].astype(numpy.float64)
    clip = signal.sosfiltfilt(signal.butter(2, 30, 'highpass', fs=RATE, output='sos'), clip)
    clip = signal.sosfiltfilt(signal.butter(4, 12000, fs=RATE, output='sos'), clip)
    centers = numpy.arange(.08, duration - .079, .01)
    frequencies = numpy.fft.rfftfreq(32768, 1 / RATE)
    pulses = []
    for center in centers:
        # A rev's upper harmonics can be louder than its firing fundamental.
        # Follow a manually identified ridge instead of allowing the tracker to
        # jump to a 1.5x harmonic as the engine slows at the end of the excerpt.
        expected = numpy.interp(
            center, [point['seconds'] for point in pitch_guide],
            [point['hz'] for point in pitch_guide])
        candidates = numpy.flatnonzero(abs(frequencies - expected) < expected * .10)
        frame = clip[round((center - .07) * RATE):round((center + .07) * RATE)]
        spectrum = abs(numpy.fft.rfft(frame * numpy.hanning(len(frame)), 32768))
        peak = candidates[numpy.argmax(spectrum[candidates])]
        a, b, c = numpy.log(spectrum[peak - 1:peak + 2] + 1e-15)
        offset = .5 * (a - c) / (a - 2 * b + c)
        pulses.append((peak + offset) * RATE / 32768)
    times = numpy.arange(len(clip)) / RATE
    trend = numpy.polynomial.Polynomial.fit(centers, pulses, 4)
    estimated_phase = numpy.cumsum(trend(times)) / RATE
    demodulated = clip * numpy.exp(-2j * numpy.pi * estimated_phase)
    baseband = signal.sosfiltfilt(
        signal.butter(4, 18, fs=RATE, output='sos'), demodulated)
    phase = estimated_phase + numpy.unwrap(numpy.angle(baseband)) / (2 * numpy.pi)
    margin = round(.12 * RATE)
    phase = phase[margin:-margin]
    clip = clip[margin:-margin]
    if numpy.any(numpy.diff(phase) <= 0):
        raise ValueError(f'non-monotonic firing phase at {start}s')
    first_cycle = numpy.ceil(phase[0] / 4) * 4
    cycles = int((phase[-1] - first_cycle) // 4) * 4 - 4
    if cycles < 12:
        raise ValueError('not enough stable engine cycles')
    end = round(cycles * RATE / (rpm / 30))
    pulse = cycles * RATE / end
    overlap = round(4 * RATE / pulse)
    positions = first_cycle + numpy.arange(end + overlap) * pulse / RATE
    steady = numpy.interp(positions, phase, clip)
    # Remove the recorded throttle swell, retaining firing pulses and broadband
    # texture. A slow RMS envelope avoids turning each pulse into a flat wave.
    envelope = numpy.sqrt(ndimage.gaussian_filter1d(steady * steady, .035 * RATE))
    steady *= numpy.median(envelope) / numpy.maximum(envelope, 1e-6)
    steady = level_engine_bands(steady, pulse)
    gain = min(.20 / numpy.sqrt(numpy.mean(steady * steady)), .80 / max(abs(steady)))
    pcm = numpy.rint(steady * gain * 32767).astype('<i2').tobytes()
    return {
        'pcm': pcm, 'pulse_hz': pulse, 'overlap': overlap,
        'repeat_seconds': end / RATE, 'source_gain': float(gain),
        'source_start_seconds': start, 'source_duration_seconds': duration,
        'pitch_stabilized': True, 'firing_cycles': cycles, 'spectral_leveling': True,
        'source_pitch_guide': pitch_guide,
    }


def pitsunda(samples, output):
    output.mkdir(parents=True, exist_ok=True)
    manifest = {'version': 1, 'sample_rate': RATE, 'bands': []}
    # Recover the tail of the original idle loop, before the later near-silent
    # section. Preserve its texture without imposing a guessed harmonic phase.
    start, duration = 34.55, .3
    idle = samples[round(start * RATE):round((start + duration) * RATE)].astype(numpy.float64)
    idle = signal.sosfiltfilt(signal.butter(2, 30, 'highpass', fs=RATE, output='sos'), idle)
    overlap = round(.035 * RATE)
    head = idle[:overlap]
    lo = round(.20 * RATE)
    errors = signal.convolve(idle[lo:] ** 2, numpy.ones(overlap), mode='valid', method='fft')
    errors -= 2 * signal.correlate(idle[lo:], head, mode='valid', method='fft')
    end = lo + int(numpy.argmin(errors))
    idle = idle[:end + overlap]
    gain = min(.20 / numpy.sqrt(numpy.mean(idle * idle)), .80 / max(abs(idle)))
    clips = [{'rpm': 900, 'clip': {
        'pcm': numpy.rint(idle * gain * 32767).astype('<i2').tobytes(),
        'pulse_hz': 30, 'overlap': overlap, 'repeat_seconds': end / RATE,
        'source_gain': float(gain), 'source_start_seconds': start,
        'source_duration_seconds': duration, 'pitch_stabilized': False,
    }}]
    for excerpt in [
        {'rpm': 2400, 'start': 21.0, 'duration': 1.2, 'pitch_guide': [
            {'seconds': 0, 'hz': 115}, {'seconds': 1.2, 'hz': 72},
        ]},
        {'rpm': 3600, 'start': 19.4, 'duration': 1.1, 'pitch_guide': [
            {'seconds': 0, 'hz': 146}, {'seconds': 1.1, 'hz': 100},
        ]},
        {'rpm': 5250, 'start': 26.15, 'duration': 1.25, 'pitch_guide': [
            {'seconds': 0, 'hz': 145}, {'seconds': .5, 'hz': 185},
            {'seconds': 1.25, 'hz': 150},
        ]},
    ]:
        clips.append({'rpm': excerpt['rpm'], 'clip': stationary_rev(samples, **excerpt)})
    for band in clips:
        clip = band['clip']
        name = f"inline4-{band['rpm']}.wav"
        write_wav(output / name, clip.pop('pcm'))
        clip['file'] = name
        manifest['bands'].append({'rpm': band['rpm'], 'load': clip, 'coast': clip})
        print(f"{name}: {clip['repeat_seconds']:.3f}s repeat, {clip['pulse_hz']:.4f} Hz")
    (output / 'engine.json').write_text(json.dumps(manifest, indent=2) + '\n')
    names = {band['load']['file'] for band in manifest['bands']}
    for path in output.glob('inline4-*.wav'):
        if path.name not in names:
            path.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--pitsunda', action='store_true')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.output is None:
        car = '2016-pitsunda-2' if args.pitsunda else '1964-kestrel'
        args.output = ROOT / 'data/cars' / car / 'audio'
    decoded = subprocess.run([
        'ffmpeg', '-v', 'error', '-i', str(args.source), '-ar', str(RATE),
        '-ac', '1', '-f', 'f32le', 'pipe:1',
    ], check=True, capture_output=True)
    samples = numpy.frombuffer(decoded.stdout, dtype='<f4')
    if args.pitsunda:
        pitsunda(samples, args.output)
        return
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {'version': 1, 'sample_rate': RATE, 'bands': []}
    for rpm, loaded, coast in EXCERPTS:
        band = {'rpm': rpm}
        for state, (start, approximate) in [('load', loaded), ('coast', coast)]:
            pcm, pulse, overlap, duration, gain = extract(samples, start, approximate)
            filename = f'flat6-{rpm}-{state}.wav'
            write_wav(args.output / filename, pcm)
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

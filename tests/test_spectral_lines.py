"""Spectral check for the scheduling lines found in the GS load-cell
investigation (4 Hz, 10 Hz and harmonics, about 0.1 kg).

Both pipelines see the same input: white noise plus two worst-case
scheduling artefacts (the same model as tests/acq_engine_sim.c): an offset on
a conversion that directly follows a gain/OSR/channel change, and a
disturbance decaying with a 3 ms time constant after every conversion on the
slow MCLK/4 clock (the die temperature), on every channel.

- Former pipeline (model of origin/dev before this change): continuous
  CH6/CH1/CH0 scan; the 2 ms worker switches to a 10 Hz auxiliary pass and a
  10 Hz temperature pass and back; KG1000 4 ms bins; the latest bin sent in a
  100 Hz weighted interleave (weights 10/10/1/1/1/1/1, a 25-slot = 250 ms
  pattern). The received stream is resampled uniformly, as the investigation
  did.
- New pipeline: the production frame engine (mcp3564r.c) in the host
  simulator, then the published 3-sample median and 20 ms bins.

The former pipeline shows lines at 10 Hz and 4 Hz; the new one shows none
(every frame has the same ADC history, so the artefact is a constant). This is
a model with an assumed artefact, not a bench capture: on hardware, compare
spectra of the published KG1000 stream at rest before and after."""
import math
import random
import unittest

import numpy as np

import acq_sim_support as sim

SECONDS = 120
STEP_ARTEFACT = 50.0    # codes, the conversion right after a reconfiguration
CLOCK_ARTEFACT = 40.0   # codes, decaying (3 ms) after a temperature conversion
NOISE = 10.0            # codes RMS per conversion


def line_snr(times_s, values, frequency, rate=50.0, segment_s=10.0):
    """Power at `frequency` over the median power of nearby bins: uniform
    resampling (as the investigation did), Welch average of 10 s Hann
    segments (0.1 Hz bins)."""
    grid = np.arange(times_s[0], times_s[-1], 1.0 / rate)
    series = np.interp(grid, times_s, values)
    n = int(segment_s * rate)
    window = np.hanning(n)
    power = None
    for start in range(0, len(series) - n + 1, n // 2):
        chunk = series[start:start + n]
        spectrum = np.abs(np.fft.rfft((chunk - chunk.mean()) * window)) ** 2
        power = spectrum if power is None else power + spectrum
    freqs = np.fft.rfftfreq(n, 1.0 / rate)
    index = int(np.argmin(np.abs(freqs - frequency)))
    peak = power[index - 1:index + 2].max()
    around = np.concatenate([power[index - 15:index - 3], power[index + 4:index + 16]])
    return peak / np.median(around)


def former_pipeline(seed=1):
    """Received KG1000 stream (time s, value) of the former pipeline."""
    rng = random.Random(seed)
    conv = 384  # us per channel conversion: OSR256, AZ_MUX, settled SCAN
    t, end = 0, SECONDS * 1_000_000
    mode, mode_since, pending = 'fast', 0, 0
    temp_attempt = aux_attempt = 0
    reconfigured = True
    last_clock = None
    samples = []  # (t_us, value) of CH0
    next_worker = 2000
    order = [6, 1, 0]
    index = 0
    while t < end:
        # The worker (2 ms) changes the scan only at its own ticks.
        while next_worker <= t:
            now = next_worker // 1000
            if mode == 'fast':
                if now - temp_attempt >= 100:
                    # TEMP at MCLK/4, OSR256, AZ_MUX: 2 x 768 DMCLK at 1 MHz.
                    mode, pending, temp_attempt, reconfigured = 'temp', 1536, now, True
                elif now - aux_attempt >= 100:
                    mode, pending, aux_attempt, reconfigured = 'aux', 5 * conv, now, True
                mode_since = next_worker
            elif next_worker - mode_since >= pending:
                if mode == 'temp':
                    last_clock = mode_since + pending
                mode, reconfigured, index = 'fast', True, 0
            next_worker += 2000
        if mode != 'fast':
            t = next_worker  # converting other channels until the next worker tick
            continue
        t += conv
        channel = order[index % 3]
        index += 1
        value = rng.gauss(0, NOISE) + (STEP_ARTEFACT if reconfigured else 0.0)
        if last_clock is not None:
            value += CLOCK_ARTEFACT * math.exp(-(t - last_clock) / 3000.0)
        reconfigured = False
        if channel == 0:
            samples.append((t, value))
    # 4 ms bins on uptime ms, dated by their last sample.
    bins, current = [], None
    for t_us, value in samples:
        b = (t_us // 1000) // 4
        if current and current[0] != b:
            bins.append((current[3], current[1] / current[2]))
            current = None
        current = (b, (current[1] if current else 0.0) + value, (current[2] if current else 0) + 1, t_us)
    # 100 Hz weighted interleave; KG1000 sends the latest completed bin.
    weights = [10, 10, 1, 1, 1, 1, 1]
    credit = [0] * 7
    received, bin_index, last_sent = [], 0, None
    for slot in range(SECONDS * 100):
        now = slot * 10_000
        for i, w in enumerate(weights):
            credit[i] += w
        chosen = max(range(7), key=lambda i: credit[i])
        credit[chosen] -= sum(weights)
        if chosen != 1:
            continue
        while bin_index + 1 < len(bins) and bins[bin_index + 1][0] <= now:
            bin_index += 1
        if bins[bin_index][0] <= now and bin_index != last_sent:
            received.append((bins[bin_index][0] / 1e6, bins[bin_index][1]))
            last_sent = bin_index
    return received


def new_pipeline(channel, seed=2):
    rng = random.Random(seed)
    samples, diag = sim.run(seconds=SECONDS, kg50=1, waveform='artefact')
    values = [(t, code - (100000 + channel * 1000) + rng.gauss(0, NOISE))
              for ch, t, frame, profile, slot, code in samples if ch == channel]
    window, received, current = [], [], None
    for t_us, value in values:
        window = (window + [value])[-3:]
        filtered = sorted(window)[(len(window) - 1) // 2]
        b = t_us // 20000
        if current and current[0] != b:
            received.append(((current[0] * 20000 + 10000) / 1e6, current[1] / current[2]))
            current = None
        current = (b, (current[1] if current else 0.0) + filtered, (current[2] if current else 0) + 1)
    return received, diag


class SpectralLineTests(unittest.TestCase):
    def test_former_schedule_had_4_and_10_hz_lines(self):
        received = former_pipeline()
        times, values = zip(*received)
        # Measured with this model: 10 Hz ~18x, 4 Hz ~9x, 6 Hz ~11x, 20 Hz ~26x
        # the neighbouring noise floor.
        self.assertGreater(line_snr(times, values, 10.0, 40.0), 8.0)
        self.assertGreater(line_snr(times, values, 4.0, 40.0), 4.0)
        self.assertGreater(line_snr(times, values, 20.0, 40.0), 8.0)

    def test_new_schedule_has_no_scheduling_lines(self):
        for channel in (0, 6):
            with self.subTest(channel=channel):
                received, diag = new_pipeline(channel)
                times, values = zip(*received)
                steps = {round((b - a) * 1e6) for a, b in zip(times, times[1:])}
                self.assertEqual(steps, {20000})  # uniform 50 Hz published stream
                for frequency in (2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 20.0):
                    self.assertLess(line_snr(times, values, frequency), 2.5, frequency)
                self.assertEqual(diag['frames_skipped'] + diag['slots_skipped'], 0)


if __name__ == '__main__':
    unittest.main()

"""Fault-scoped publisher refusals retain normal safety and resource bounds."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

from sim.run_full import scope_daq_disconnected_publication_probes

ROOT = Path(__file__).resolve().parents[1]


class DisconnectedPublicationContractTests(unittest.TestCase):
    def layout(self):
        return json.loads((ROOT / 'sim/board.json').read_text())

    def test_only_fault_layout_accepts_io_and_all_safety_bounds_remain(self):
        original = self.layout()
        fault = copy.deepcopy(original)
        scope_daq_disconnected_publication_probes(fault, ROOT, None)
        before = {p['name']: p for p in original['execution']['memory_probes']}
        after = {p['name']: p for p in fault['execution']['memory_probes']}
        self.assertEqual(before['analog_report_failures']['maximum'], 0)
        self.assertEqual(before['publication_io_refusals']['maximum'], 0)
        self.assertGreater(after['analog_report_failures']['maximum'], 0)
        self.assertEqual(after['publication_other_refusals']['maximum'], 0)
        allowed = {'analog_report_failures', 'network_publication_failures',
                   'kg50_network_failures', 'loadcell_network_failures',
                   'publication_attempts', 'publication_io_refusals'}
        for name, probe in before.items():
            if name not in allowed:
                self.assertEqual(after[name], probe, name)
        self.assertEqual(after['can_tx_queue_bytes']['maximum'], 6144)
        self.assertEqual(after['can_tx_pending']['maximum'], 48)
        self.assertEqual(after['sdk_shared_peak']['maximum'], 24576)

    def test_opportunity_limit_uses_actual_build_rate_override(self):
        fault = self.layout()
        fault['execution']['virtual_time_ms'] = 2000
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cache = root / 'build/Checked/CMakeCache.txt'
            cache.parent.mkdir(parents=True)
            cache.write_text('DAQ_NETWORK_RATE_HZ:STRING=10\n')
            scope_daq_disconnected_publication_probes(fault, root, 'Checked')
        probes = {p['name']: p for p in fault['execution']['memory_probes']}
        self.assertEqual(probes['publication_attempts']['maximum'], 21)
        self.assertEqual(probes['publication_io_refusals']['maximum'], 21)

    def test_missing_classifier_and_nonzero_unexpected_bound_fail_closed(self):
        fault = self.layout()
        fault['execution']['memory_probes'] = [
            p for p in fault['execution']['memory_probes']
            if p['name'] != 'publication_other_refusals']
        with self.assertRaisesRegex(RuntimeError, 'Missing'):
            scope_daq_disconnected_publication_probes(fault, ROOT, None)
        fault = self.layout()
        next(p for p in fault['execution']['memory_probes']
             if p['name'] == 'publication_other_refusals')['maximum'] = 1
        with self.assertRaisesRegex(RuntimeError, 'zero bound'):
            scope_daq_disconnected_publication_probes(fault, ROOT, None)

    def test_other_boards_are_not_relaxed(self):
        layout = self.layout()
        layout['name'] = 'OtherBoard'
        original = copy.deepcopy(layout)
        scope_daq_disconnected_publication_probes(layout, ROOT, None)
        self.assertEqual(layout, original)

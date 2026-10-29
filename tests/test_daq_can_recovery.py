"""Recovery must observe accepted fresh data and exact postloss control ACKs."""
import copy
import json
from pathlib import Path
import unittest
import tempfile

from sim.run_full import scope_daq_disconnected_publication_probes

from sim.run_daq_recovery import COMMAND_SAMPLES, configure_recovery, verify_host_recovery

ROOT = Path(__file__).resolve().parents[1]


class DaqCanRecoveryTests(unittest.TestCase):
    def scene(self):
        return dict(virtual_time_ms=40000, sample_count=20, reboots=[], can_ack_events=[],
                    assertions=[], host_log_assertions=[],
                    host_nodes=[dict(name='groundstation', env={})])

    def test_fault_schedule_keeps_healthy_zero_intervals_and_unrelated_safety(self):
        topology = self.scene()
        daq = json.loads((ROOT / 'sim/board.json').read_text())
        before = copy.deepcopy(daq)
        configure_recovery(topology, daq)
        self.assertEqual([(event['after_sample'], event['connected'])
                          for event in topology['can_link_events']], [(3, False), (5, True)])
        probes = {p['name']: p for p in daq['execution']['memory_probes']}
        old = {p['name']: p for p in before['execution']['memory_probes']}
        for name in ['publication_other_refusals', 'sdk_admission_drops', 'sdk_shared_peak',
                     'sd_write_errors', 'sd_line_drops', 'sd_raw_batch_drops', 'panics',
                     'allocation_failures', 'daq_sample_overruns', 'telemetry_stack_remaining']:
            self.assertEqual(probes[name], old[name], name)
        checks = [a for a in topology['assertions']
                  if a['probe'] == 'publication_io_refusals' and 'maximum_gain' in a]
        self.assertEqual([a['to_sample'] for a in checks], [1, 2, *range(7, 20)])
        self.assertTrue(all(a['maximum_gain'] == 0 for a in checks))

    def test_offer_bound_uses_actual_bay_duration_not_standalone_profile(self):
        daq = json.loads((ROOT / "sim/board.json").read_text())
        daq["execution"]["virtual_time_ms"] = 2000
        configure_recovery(self.scene(), daq)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "Core/Inc").mkdir(parents=True)
            (root / "Core/Inc/daq_network_config.h").write_text("#define DAQ_NETWORK_RATE_HZ 100U\n")
            scope_daq_disconnected_publication_probes(daq, root, None)
        probes = {p["name"]: p for p in daq["execution"]["memory_probes"]}
        self.assertEqual(probes["publication_attempts"]["maximum"], 4001)
        self.assertEqual(probes["publication_io_refusals"]["maximum"], 4001)
        self.assertEqual(probes["publication_other_refusals"]["maximum"], 0)

    def healthy_log(self):
        lines = [f'full-bay soak valve command acknowledged: board={board}, state=false, '
                 f'round1/16, sample {sample}, latency 50 ms'
                 for board in ['AB', 'VB'] for sample in COMMAND_SAMPLES]
        # Deliberately false legacy250Hz flag: accepted counts/freshness,
        # measured against configured40Hz, determine the recovery result.
        lines.append('sustained fill delivery failed ' + json.dumps(dict(
            start_ms=20000, elapsed_ms=10000, received=[400, 400, 10, 10, 10],
            max_gap_ms=[25, 25, 1000, 1000, 1000], passed=False)))
        return '\n'.join(lines)

    def test_actual_accepted_data_and_every_fresh_control_ack_are_required(self):
        log = self.healthy_log()
        self.assertEqual(len(verify_host_recovery(log, 40)['windows']), 1)
        lines = log.splitlines()
        del lines[0]
        with self.assertRaisesRegex(ValueError, 'Missing'):
            verify_host_recovery('\n'.join(lines), 40)

    def test_missing_duplicate_late_or_stale_observations_fail(self):
        log = self.healthy_log()
        for broken in [log.replace('latency 50 ms', 'latency 2501 ms', 1),
                       log + '\n' + log.splitlines()[0],
                       log.replace('"received": [400, 400', '"received": [379, 400'),
                       log.replace('"max_gap_ms": [25, 25', '"max_gap_ms": [501, 25'),
                       log[:log.index('sustained fill delivery')]]:
            with self.subTest(broken=broken[-100:]), self.assertRaises(ValueError):
                verify_host_recovery(broken, 40)

    def test_other_faults_or_short_scenes_cannot_qualify_as_recovery(self):
        for field, value in [('reboots', [dict(node='daq', after_sample=5)]),
                             ('sample_count', 6), ('virtual_time_ms', 16000)]:
            scene = self.scene()
            scene[field] = value
            with self.assertRaises(ValueError):
                configure_recovery(scene, json.loads((ROOT / 'sim/board.json').read_text()))

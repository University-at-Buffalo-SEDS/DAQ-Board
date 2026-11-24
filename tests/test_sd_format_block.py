"""SEDS-SD format block: integrity, frozen layout, grammar and label coverage.

The block (Core/Inc/sd_format.h) follows the 13-column header in every DAQ SD
file and part. The ground station accepts a file by (format, version) plus a
schema hash over the declared columns/records, so the declared layout must not
change without a version bump and a matching GroundStation26 parser update.
"""
from pathlib import Path
import hashlib
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]

# version -> schema hash. Never edit an existing entry: a changed layout gets
# a new version and a new entry here, in sd_format.h and in the GS SD parser.
FROZEN = {3: 'ade30ff7', 4: 'bc9e710a'}
BUMP = ("SD record layout changed: bump SD format version, add the frozen hash and "
        "register the version in the GroundStation26 SD parser")

COLUMN_HEADER = ("timestamp_ms,monotonic_ms,sensor,value,raw_adc_code,raw_value,calibrated_value,"
                 "time_source,adc_temperature_c,adc_temperature_code,uptime_ticks,"
                 "uptime_frequency_hz,record_sequence\r\n")
# Labels the source names but never writes as a valid row.
NOT_WRITTEN = {'invalid_adc_channel'}

UNIT = r'[A-Za-z0-9_/.%+-]+(\*[1-9][0-9]*)?'
ENTRY = re.compile(rf'^(?P<label>[A-Za-z0-9_]+\*?)(?P<cols>(:[varc]={UNIT}(;{UNIT})*)+)'
                   r'(@(?P<hz>[1-9][0-9]*))?$')
FORMAT_LINE = re.compile(r'^# seds-sd,format=(?P<format>daq),version=(?P<version>[1-9][0-9]*),'
                         r'board=DAQ,firmware=(?P<firmware>[0-9a-f]{12}(-dirty)?|unknown),'
                         r'tick_hz=(?P<tick>[1-9][0-9]*),schema=(?P<schema>[0-9a-f]{8})\r\n$')


def compiled_header(revision=None):
    """What sd_open_log writes before its '# channel' line, from a host compile."""
    code = r'''
#include <stdio.h>
#include "sd_record_format.h"
#include "sd_format.h"
int main(void) { fputs(SD_CSV_HEADER SD_FORMAT_BLOCK, stdout); fputs("\n" SD_FORMAT_SCHEMA, stdout); return 0; }
'''
    flags = [f'-DSEDS_FIRMWARE_REVISION="{revision}"'] if revision else []
    with tempfile.TemporaryDirectory() as temp:
        binary = Path(temp) / 'sd-format'
        subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', *flags, '-I', str(ROOT / 'Core/Inc'),
                        '-x', 'c', '-', '-o', str(binary)], input=code, text=True, check=True)
        output = subprocess.run([str(binary)], capture_output=True, check=True).stdout.decode('ascii')
    text, schema = output.rsplit('\n', 1)
    return text, schema


def split_lines(text):
    lines = text.split('\r\n')
    assert lines[-1] == '', 'block must end with CRLF'
    return [line + '\r\n' for line in lines[:-1]]


def schema_hash(lines):
    declared = [line for line in lines if line.startswith(('# seds-sd-columns,', '# seds-sd-records,'))]
    return hashlib.sha256(''.join(declared).encode('ascii')).hexdigest()[:8]


def entries(lines):
    return [entry for line in lines if line.startswith('# seds-sd-records,')
            for entry in line[len('# seds-sd-records,'):-2].split(',')]


def declared_labels(lines):
    return {ENTRY.match(entry).group('label'): entry for entry in entries(lines)}


def covered(label, declared):
    return label in declared or any(name.endswith('*') and label.startswith(name[:-1]) for name in declared)


def c_strings(text):
    return re.findall(r'"([^"\\]*)"', text)


def source_labels():
    """Best effort: every string literal handed to an SD record formatter."""
    sd = (ROOT / 'Core/Src/sd_card.c').read_text()
    labels = set()
    scalar = re.search(r'g_sd_scalar_sensors\[\] = \{(.*?)\};', sd, re.S).group(1)
    labels |= set(c_strings(scalar))
    raw = re.search(r'const char \*sensor = (.*?);\n', sd, re.S).group(1)
    labels |= set(c_strings(raw))
    for call in re.finditer(r'sd_format_csv_(?:sensor_|raw_)?record\((.*?)\);', sd, re.S):
        labels |= {s for s in c_strings(call.group(1)) if re.fullmatch(r'[a-z][a-z0-9_]*', s)}
    for path in sorted((ROOT / 'Core/Src').glob('*.c')):
        text = path.read_text()
        for call in re.finditer(r'sd_card_enqueue_csv_row(?:_captured)?\(\s*"([^"]+)"', text):
            labels.add(call.group(1))
        names = re.search(r'k_sd_names\[[^\]]*\]\[[^\]]*\] = \{(.*?)\};', text, re.S)
        if names:
            labels |= set(c_strings(names.group(1)))
    return labels


class SdFormatBlockTests(unittest.TestCase):
    def setUp(self):
        self.text, self.schema = compiled_header('0123456789ab')
        self.lines = split_lines(self.text)

    def test_block_follows_the_column_header(self):
        self.assertEqual(self.lines[0], COLUMN_HEADER)
        self.assertRegex(self.lines[1], FORMAT_LINE)
        self.assertEqual(self.lines[2], '# seds-sd-columns,u64:ms,u64:ms,label,vec,vec,vec,vec,clock,'
                                        'f32:degC,i32:code,u64:tick,u32:Hz,u64:seq\r\n')
        self.assertGreater(len(self.lines), 3)
        self.assertTrue(all(line.startswith('# seds-sd-records,') for line in self.lines[3:]))
        self.assertTrue(self.text.isascii())

    def test_schema_hash_matches_the_compiled_constant_and_the_frozen_table(self):
        fields = FORMAT_LINE.match(self.lines[1]).groupdict()
        computed = schema_hash(self.lines)
        self.assertEqual(fields['schema'], self.schema)
        self.assertEqual(computed, self.schema,
                         f"SD_FORMAT_SCHEMA is {self.schema}, declared lines hash to {computed}. {BUMP}")
        version = int(fields['version'])
        self.assertEqual(FROZEN.get(version), computed, f"version {version} hashes to {computed}. {BUMP}")
        self.assertEqual(fields['tick'], '1000')

    def test_firmware_revision_is_informational(self):
        text, schema = compiled_header()
        lines = split_lines(text)
        self.assertEqual(FORMAT_LINE.match(lines[1]).group('firmware'), 'unknown')
        self.assertEqual(schema_hash(lines), schema)
        self.assertEqual(lines[2:], self.lines[2:])
        dirty, _ = compiled_header('0123456789ab-dirty')
        self.assertRegex(split_lines(dirty)[1], FORMAT_LINE)

    def test_cmake_injects_the_short_revision_only_into_the_sd_writer(self):
        cmake = (ROOT / 'cmake/firmware_revision.cmake').read_text()
        self.assertIn('rev-parse --short=12 HEAD', cmake)
        self.assertIn('set(SEDS_FIRMWARE_REVISION "unknown")', cmake)
        self.assertIn('Core/Src/sd_card.c', cmake)
        self.assertIn('include(cmake/firmware_revision.cmake)', (ROOT / 'CMakeLists.txt').read_text())

    def test_record_entries_follow_the_grammar(self):
        columns = self.lines[2][:-2].split(',')[1:]
        self.assertEqual(len(columns), COLUMN_HEADER.count(',') + 1)
        seen = set()
        for entry in entries(self.lines):
            match = ENTRY.match(entry)
            self.assertIsNotNone(match, entry)
            self.assertNotIn(match.group('label'), seen, entry)
            seen.add(match.group('label'))
            cols = [part.split('=')[0] for part in match.group('cols').split(':')[1:]]
            self.assertEqual(len(cols), len(set(cols)), entry)
            for forbidden in ' ,;=@':
                self.assertNotIn(forbidden, match.group('label'))
        declared = declared_labels(self.lines)
        # Spot-check clocks: MCP3564R conversions in us, everything else in ms.
        self.assertTrue(declared['kg1000_v'].endswith('@1000000'))
        self.assertTrue(declared['chamber_pressure_v'].endswith(':c=psig@1000000'))
        self.assertNotIn('@', declared['input_voltage_v'])
        self.assertNotIn('@', declared['valve_pressure_psi'])

    def test_every_written_label_is_declared(self):
        declared = declared_labels(self.lines)
        found = source_labels() - NOT_WRITTEN
        self.assertGreater(len(found), 60)
        missing = sorted(label for label in found if not covered(label, declared))
        self.assertEqual(missing, [], f"undeclared SD labels {missing}. {BUMP}")

    def test_every_file_open_writes_the_block_after_the_column_header(self):
        sd = (ROOT / 'Core/Src/sd_card.c').read_text()
        sources = ''.join(path.read_text() for path in sorted((ROOT / 'Core/Src').glob('*.c')))
        # One column header in the firmware, immediately followed by the block.
        self.assertEqual(sources.count('SD_CSV_HEADER'), 1)
        self.assertRegex(sd, r'static const char header\[\] =\s*SD_CSV_HEADER\s*SD_FORMAT_BLOCK\s*"# channel=')
        # Log files are created and opened for writing only in sd_open_log; new
        # parts (FAT32 limit) and the launch log re-enter it.
        self.assertEqual(sd.count("sd_create_log_file("), 2)
        opener = sd[sd.index('static UINT sd_open_log('):]
        opener = opener[:opener.index('\n}\n')]
        self.assertIn('sd_create_log_file(', opener)
        self.assertLess(opener.index('FX_OPEN_FOR_WRITE'), opener.index('fx_file_write(&log->file, (VOID *)header'))
        self.assertEqual(opener.count('fx_file_write'), 3)
        # The only other file written is the provisioning marker (not a log).
        self.assertEqual(opener.count('FX_OPEN_FOR_WRITE'), 1)
        others = [sd[max(0, m.start() - 200):m.start()] for m in re.finditer('FX_OPEN_FOR_WRITE', sd)]
        others = [text for text in others if 'SD_PROVISION_MARKER' not in text]
        self.assertEqual(len(others), 1, others)
        self.assertEqual(sd.count('fx_file_create('), 2)
        self.assertIn('if (!log->open && sd_open_log(log, key, calibration) != FX_SUCCESS) return NULL;', sd)


if __name__ == '__main__':
    unittest.main()

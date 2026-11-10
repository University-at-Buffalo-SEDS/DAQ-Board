"""Tests must not depend on the name of the checkout directory.

test_ota_lifecycle_native selected the DAQ fixture with ROOT.name ==
'DAQ-Board', so any worktree or clone under another name failed to link
(persistent_store_try_enter undefined). Board identity comes from the
layout (sim/board.json), which travels with the sources.
"""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


class CheckoutNameIndependenceTests(unittest.TestCase):
    def test_no_test_selects_behaviour_by_directory_name(self):
        offenders = [path.name for path in sorted((ROOT / 'tests').glob('test_*.py'))
                     if path.name != Path(__file__).name and re.search(r"ROOT\.name\s*(==|in)\s*\[?['\"]DAQ-Board", path.read_text())]
        self.assertEqual(offenders, [])


if __name__ == '__main__':
    unittest.main()

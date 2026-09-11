import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


class SdFloatFormatTests(unittest.TestCase):
    def test_csv_preserves_binary32_samples(self):
        root = Path(__file__).resolve().parents[1]
        compiler = shutil.which("cc")
        self.assertIsNotNone(compiler, "Install a C compiler to run SD format tests")
        with tempfile.TemporaryDirectory(prefix="daq-float-test-") as tmp:
            binary = str(Path(tmp) / "test_sd_float_format")
            subprocess.run([compiler, "-std=c11", "-O2", "-Wall", "-Wextra",
                            "-Werror", "-I", str(root / "Core/Inc"),
                            str(root / "tests/test_sd_float_format.c"),
                            "-o", binary], check=True)
            subprocess.run([binary], check=True)

import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from armctl.cli import main


class TestScript(unittest.TestCase):
    def test_script_runs(self):
        fd, path = tempfile.mkstemp(suffix=".txt")
        try:
            with os.fdopen(fd, "w") as f:
                f.write("# comment\n")
                f.write("move base 45\n")
                f.write("wait 10\n")
                f.write("M0:90\n")
                f.write("\n")
            rc = main(["--sim", "script", path])
            self.assertEqual(rc, 0)
        finally:
            os.remove(path)


if __name__ == "__main__":
    unittest.main()
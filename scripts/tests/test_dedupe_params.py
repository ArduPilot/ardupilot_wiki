"""Tests for scripts/dedupe_params.py.

    python3 scripts/tests/test_dedupe_params.py
"""
import os
import re
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dedupe_params import (  # noqa: E402
    dedupe_old_rangefinder_parameters,
    dedupe_periph_net_parameters,
    dedupe_rst_parameter_sections,
)


def header(n):
    return (f".. _parameters_RNGFND{n}_:\n\nRNGFND{n} Parameters\n"
            f"------------------\n\n").encode()


def item(n, name, text="text"):
    return f".. _RNGFND{n}_{name}:\n\nRNGFND{n}_{name}: {text}\n\n".encode()


NET_HEADER = b".. _parameters_NET_:\n\nNET\\_ Parameters\n----------------\n\n\n"
NET_PPP = b".. _NET_PPP_PORT:\n\nNET_PPP_PORT: PPP port\n\n"
NET_ENABLE = b".. _NET_ENABLE:\n\nNET_ENABLE: Enable\n"


class DedupeTestCase(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = Path(tmp.name) / "params.rst"

    def run_dedupe(self, data, func=dedupe_old_rangefinder_parameters):
        self.path.write_bytes(data)
        func(str(self.path))
        return self.path.read_bytes()


class TestRangefinder(DedupeTestCase):
    def test_duplicate_header_is_removed(self):
        data = header(1) + item(1, "TYPE") + header(1) + item(1, "MIN")
        self.assertEqual(self.run_dedupe(data),
                         header(1) + item(1, "TYPE") + item(1, "MIN"))

    def test_file_without_duplicates_is_not_rewritten(self):
        data = header(1) + item(1, "TYPE") + header(2) + item(2, "TYPE")
        self.path.write_bytes(data)
        os.utime(self.path, ns=(10**9, 10**9))
        dedupe_old_rangefinder_parameters(str(self.path))
        self.assertEqual(self.path.read_bytes(), data)
        self.assertEqual(self.path.stat().st_mtime_ns, 10**9)

    def test_different_numbers_are_not_merged(self):
        data = header(1) + item(1, "A") + header(2) + item(2, "A") + header("A") + item("A", "A")
        self.assertEqual(self.run_dedupe(data), data)

    def test_duplicate_block_stops_at_next_section_anchor(self):
        data = header(1) + item(1, "A") + header(1) + header(2) + item(2, "A")
        self.assertEqual(self.run_dedupe(data),
                         header(1) + item(1, "A") + header(2) + item(2, "A"))

    def test_duplicate_header_at_end_of_file(self):
        data = header(1) + item(1, "A") + header(1)
        self.assertEqual(self.run_dedupe(data), header(1) + item(1, "A"))

    def test_anchor_matching_is_case_insensitive(self):
        data = header(1) + item(1, "A") + header(1).replace(b"RNGFND", b"rngfnd", 1) + item(1, "B")
        self.assertEqual(self.run_dedupe(data), header(1) + item(1, "A") + item(1, "B"))

    def test_crlf_is_normalised_to_lf_when_rewritten(self):
        data = header(1) + item(1, "A") + header(1) + item(1, "B")
        out = self.run_dedupe(data.replace(b"\n", b"\r\n"))
        self.assertEqual(out, header(1) + item(1, "A") + item(1, "B"))

    def test_invalid_utf8_is_reported_and_file_left_untouched(self):
        bad_item = item(1, "A").replace(b": text", b": x\xff")
        data = header(1) + bad_item + header(1) + item(1, "B")
        self.assertIn(b"\xff", data)
        self.path.write_bytes(data)
        with self.assertLogs(level="ERROR"):
            dedupe_old_rangefinder_parameters(str(self.path))
        self.assertEqual(self.path.read_bytes(), data)


class TestPeriphNet(DedupeTestCase):
    def test_duplicate_net_header_is_removed(self):
        data = NET_HEADER + NET_PPP + NET_HEADER + NET_ENABLE
        out = self.run_dedupe(data, dedupe_periph_net_parameters)
        self.assertEqual(out, NET_HEADER + NET_PPP + NET_ENABLE)

    def test_single_net_header_is_unchanged(self):
        data = NET_HEADER + NET_PPP + NET_ENABLE
        self.assertEqual(self.run_dedupe(data, dedupe_periph_net_parameters), data)


class TestGeneric(DedupeTestCase):
    def test_key_group_zero_uses_whole_match(self):
        anchor = re.compile(r"^\.\. _sec_[0-9]:$")
        nxt = re.compile(r"^\.\. _item")
        data = (b".. _sec_1:\n\nT\n\n.. _item1:\n\nx\n"
                b".. _sec_2:\n\nT\n\n.. _item2:\n\ny\n"
                b".. _sec_1:\n\nT\n\n.. _item3:\n\nz\n")
        self.path.write_bytes(data)
        dedupe_rst_parameter_sections(str(self.path), anchor, nxt)
        self.assertEqual(self.path.read_bytes(),
                         b".. _sec_1:\n\nT\n\n.. _item1:\n\nx\n"
                         b".. _sec_2:\n\nT\n\n.. _item2:\n\ny\n"
                         b".. _item3:\n\nz\n")

    def test_missing_file_logs_warning_and_creates_nothing(self):
        with self.assertLogs(level="WARNING"):
            dedupe_old_rangefinder_parameters(str(self.path))
        self.assertFalse(self.path.exists())

    def test_unreadable_path_logs_error(self):
        self.path.mkdir()  # a directory exists but cannot be read as a file
        with self.assertLogs(level="ERROR"):
            dedupe_old_rangefinder_parameters(str(self.path))

    @unittest.skipIf(hasattr(os, "geteuid") and os.geteuid() == 0,
                     "root ignores file permissions")
    def test_write_failure_logs_error_and_keeps_file(self):
        data = header(1) + item(1, "A") + header(1) + item(1, "B")
        self.path.write_bytes(data)
        os.chmod(self.path, 0o444)
        with self.assertLogs(level="ERROR"):
            dedupe_old_rangefinder_parameters(str(self.path))
        self.assertEqual(self.path.read_bytes(), data)


if __name__ == "__main__":
    unittest.main()

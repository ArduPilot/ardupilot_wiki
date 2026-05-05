#!/usr/bin/env python3
"""Tests for check_ref_directives.py, check_rst_literal_blocks.py and check_youtube_timestamps.py."""

import pathlib
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import check_youtube_timestamps  # noqa: E402


def _rst(tmp: pathlib.Path, name: str, content: str) -> pathlib.Path:
    path = tmp / name
    path.write_text(content, encoding="utf-8")
    return path


class TestCheckYoutubeTimestamps(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = pathlib.Path(self._tmp.name)

    def tearDown(self):
        self._tmp.cleanup()

    def test_valid_no_timestamp(self):
        path = _rst(self.tmp, "valid.rst", ".. youtube:: dQw4w9WgXcQ\n")
        self.assertEqual(check_youtube_timestamps.check_file(path), [])

    def test_valid_url_parameters_option(self):
        content = ".. youtube:: dQw4w9WgXcQ\n    :url_parameters: ?start=42\n"
        path = _rst(self.tmp, "valid_option.rst", content)
        self.assertEqual(check_youtube_timestamps.check_file(path), [])

    def test_timestamp_t_with_s_suffix(self):
        path = _rst(self.tmp, "bad_ts.rst", ".. youtube:: dQw4w9WgXcQ?t=42s\n")
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)
        self.assertIn("url_parameters", errors[0])

    def test_url_parameters_with_s_suffix(self):
        content = ".. youtube:: dQw4w9WgXcQ\n    :url_parameters: ?start=42s\n"
        path = _rst(self.tmp, "bad_suffix.rst", content)
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)
        self.assertIn("42s", errors[0])
        self.assertIn("plain integer", errors[0])

    def test_url_parameters_plain_integer_no_false_positive(self):
        # Ensure multi-digit plain integers like ?start=190 are not flagged
        content = ".. youtube:: _iyTo9H7HAk\n    :url_parameters: ?start=190\n"
        path = _rst(self.tmp, "valid_multidigit.rst", content)
        self.assertEqual(check_youtube_timestamps.check_file(path), [])

    def test_timestamp_with_question_mark_t(self):
        path = _rst(self.tmp, "bad_qt.rst", ".. youtube:: dQw4w9WgXcQ?t=42\n")
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)
        self.assertIn("url_parameters", errors[0])

    def test_timestamp_with_ampersand_t(self):
        path = _rst(self.tmp, "bad_at.rst", ".. youtube:: dQw4w9WgXcQ&t=42\n")
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)
        self.assertIn("url_parameters", errors[0])

    def test_timestamp_with_question_mark_start(self):
        path = _rst(self.tmp, "bad_qs.rst", ".. youtube:: dQw4w9WgXcQ?start=103\n")
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)
        self.assertIn("url_parameters", errors[0])

    def test_timestamp_with_ampersand_start(self):
        path = _rst(self.tmp, "bad_as.rst", ".. youtube:: dQw4w9WgXcQ&start=103\n")
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)
        self.assertIn("url_parameters", errors[0])

    def test_directive_with_leading_spaces(self):
        path = _rst(self.tmp, "indented.rst", "   ..  youtube:: dQw4w9WgXcQ&t=5\n")
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)

    def test_negative_video_id_no_timestamp(self):
        # Negative video IDs (starting with -) with no timestamp are valid
        path = _rst(self.tmp, "neg_id.rst", ".. youtube:: -Db4u8LJE5w\n")
        self.assertEqual(check_youtube_timestamps.check_file(path), [])

    def test_negative_video_id_with_timestamp(self):
        path = _rst(self.tmp, "neg_id_ts.rst", ".. youtube:: -Db4u8LJE5w&t=103\n")
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)
        self.assertIn("url_parameters", errors[0])

    def test_main_valid(self):
        path = _rst(self.tmp, "valid.rst", ".. youtube:: dQw4w9WgXcQ\n")
        with patch("sys.argv", ["check_youtube_timestamps", str(path)]):
            self.assertEqual(check_youtube_timestamps.main(), 0)

    def test_main_invalid(self):
        path = _rst(self.tmp, "invalid.rst", ".. youtube:: dQw4w9WgXcQ&t=42\n")
        with patch("sys.argv", ["check_youtube_timestamps", str(path)]):
            self.assertEqual(check_youtube_timestamps.main(), 1)

    def test_non_rst_skipped(self):
        path = self.tmp / "video.txt"
        path.write_text(".. youtube:: dQw4w9WgXcQ&t=42\n", encoding="utf-8")
        with patch("sys.argv", ["check_youtube_timestamps", str(path)]):
            self.assertEqual(check_youtube_timestamps.main(), 0)

    def test_unicode_error(self):
        path = self.tmp / "bad.rst"
        path.write_bytes(b".. youtube:: \xff\n")
        errors = check_youtube_timestamps.check_file(path)
        self.assertEqual(len(errors), 1)
        self.assertIn("UnicodeDecodeError", errors[0])


if __name__ == "__main__":
    unittest.main()

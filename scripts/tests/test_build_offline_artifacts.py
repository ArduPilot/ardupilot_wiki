"""Unit checks on the build side of the parameter deltas: which versions are
carried, how a delta is written and headed, and that it rebuilds.

    python3 -m unittest scripts.tests.test_build_offline_artifacts
"""

import contextlib
import hashlib
import io
import os
import pathlib
import sys
import tarfile
import tempfile
import unittest

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

from scripts import build_offline_artifacts as build  # noqa: E402


def version(channel, number):
    return {"file": f"docs/parameters-Copter-{channel}-V{number}.html",
            "channel": channel, "version": number, "label": number, "bytes": 1}


class TestCarriedVersions(unittest.TestCase):
    def test_only_4x_stables_and_the_newest_beta(self):
        built = [version("stable", "4.7.1"), version("beta", "4.7.1"), version("stable", "4.6.0"),
                 version("beta", "4.6.0"), version("stable", "3.6.12"), version("beta", "3.6.12"),
                 version("stable", "4.5.7")]
        carried = build.carried_param_versions(built)
        self.assertEqual([v["version"] for v in carried], ["4.7.1", "4.7.1", "4.6.0", "4.5.7"])
        self.assertEqual([v["channel"] for v in carried], ["stable", "beta", "stable", "stable"])

    def test_the_newest_stable_is_the_base_and_only_it(self):
        carried = build.carried_param_versions(
            [version("stable", "4.5.0"), version("stable", "4.7.0"), version("beta", "4.8.0")])
        bases = [v for v in carried if v.get("default")]
        self.assertEqual([v["version"] for v in bases], ["4.7.0"])
        self.assertEqual(carried[0]["version"], "4.8.0")   # newest first, beta above

    def test_nothing_from_before_4x(self):
        self.assertEqual(build.carried_param_versions([version("stable", "3.6.12"), version("beta", "3.7.0")]), [])

    def test_a_10x_release_counts_as_4x_or_later(self):
        carried = build.carried_param_versions([version("stable", "10.0.0"), version("stable", "4.7.0")])
        self.assertEqual([v["version"] for v in carried], ["10.0.0", "4.7.0"])
        self.assertTrue(carried[0]["default"])

    def test_parsing_a_page_name(self):
        info = build.param_version_of(pathlib.Path("docs/parameters-Plane-stable-V4.6.3.html"))
        self.assertEqual((info["channel"], info["version"], info["label"]), ("stable", "4.6.3", "4.6.3"))
        beta = build.param_version_of(pathlib.Path("docs/parameters-Plane-beta-V4.7.0.html"))
        self.assertEqual(beta["label"], "4.7.0 beta")
        self.assertIsNone(build.param_version_of(pathlib.Path("docs/parameters.html")))
        self.assertIsNone(build.param_version_of(pathlib.Path("other/parameters-Plane-stable-V4.6.3.html")))


class TestDeltaWriting(unittest.TestCase):
    def setUp(self):
        try:
            import zstandard  # noqa: F401
        except ImportError:
            self.skipTest("zstandard not installed")
        self.base = (b"<!DOCTYPE html><html><body>" +
                     b"".join(b"<p>PARAM_%d: value %d</p>" % (i, i) for i in range(2000)) +
                     b"</body></html>")
        self.page = self.base.replace(b"PARAM_100: value 100", b"PARAM_100: value 101")

    def test_a_delta_is_small_and_rebuilds_the_page(self):
        import zstandard
        delta = build.delta_against(self.base, self.page)
        self.assertLess(len(delta), len(self.page) // 50)
        dec = zstandard.ZstdDecompressor(dict_data=zstandard.ZstdCompressionDict(
            self.base, dict_type=zstandard.DICT_TYPE_RAWCONTENT))
        self.assertEqual(dec.decompress(delta), self.page)

    def test_a_delta_carries_a_checksum_and_its_size(self):
        import zstandard
        delta = build.delta_against(self.base, self.page)
        params = zstandard.get_frame_parameters(delta)
        self.assertTrue(params.has_checksum)
        self.assertEqual(params.content_size, len(self.page))

    def test_a_change_at_the_far_end_of_a_large_page_still_rebuilds(self):
        # The reference from the page's end back into the base's start spans
        # both pages; the prefix dictionary must reach that far.
        import zstandard
        big_base = self.base * 40   # about 2 MB
        big_page = big_base[:-20] + b"<p>changed</p></body></html>"
        delta = build.delta_against(big_base, big_page)
        self.assertLess(len(delta), 4096)
        dec = zstandard.ZstdDecompressor(dict_data=zstandard.ZstdCompressionDict(
            big_base, dict_type=zstandard.DICT_TYPE_RAWCONTENT))
        self.assertEqual(dec.decompress(delta), big_page)

    def test_archive_entries_and_manifest(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            docs = root / "wiki" / "build" / "html" / "docs"
            docs.mkdir(parents=True)
            (docs / "parameters-Copter-stable-V4.7.0.html").write_bytes(self.base)
            (docs / "parameters-Copter-stable-V4.6.0.html").write_bytes(self.page)
            (docs / "parameters-Copter-stable-V3.6.0.html").write_bytes(self.page)
            out = root / "out"
            out.mkdir()
            files = {}
            cwd = pathlib.Path.cwd()
            try:
                import os
                os.chdir(root)
                with build.reproducible_tar(out / "wiki-offline.tar.gz") as tar:
                    versions = build.add_param_versions(
                        tar, "wiki", root / "wiki" / "build" / "html", out, {}, {"wiki"}, files)
            finally:
                os.chdir(cwd)
            self.assertEqual([v["version"] for v in versions], ["4.7.0", "4.6.0"])
            with tarfile.open(out / "wiki-offline.tar.gz") as tar:
                names = {m.name: tar.extractfile(m).read() for m in tar if m.isfile()}
            self.assertEqual(set(names), {"wiki/docs/parameters-Copter-stable-V4.7.0.html",
                                          "wiki/docs/parameters-Copter-stable-V4.6.0.html"})
            base_entry = names["wiki/docs/parameters-Copter-stable-V4.7.0.html"]
            delta_entry = names["wiki/docs/parameters-Copter-stable-V4.6.0.html"]
            self.assertTrue(base_entry.startswith(b"<!DOCTYPE"))
            head, _, frame = delta_entry.partition(b"\n")
            self.assertEqual(head, b"APDELTA1 parameters-Copter-stable-V4.7.0.html " +
                             hashlib.sha256(self.page).hexdigest()[:16].encode())
            self.assertEqual(versions[1]["bytes"], len(delta_entry))
            self.assertEqual(files["wiki/docs/parameters-Copter-stable-V4.6.0.html"],
                             build.content_hash(delta_entry))
            # Published loose for the differential update, gzipped.
            self.assertTrue((out / "files" / "wiki" / "docs" /
                             "parameters-Copter-stable-V4.6.0.html.gz").is_file())


class TestVersionsThatCannotBeCarried(unittest.TestCase):
    """Neither case stops the build, so both have to say so in the log."""

    def carry(self, tmp, names):
        root = pathlib.Path(tmp)
        docs = root / "wiki" / "build" / "html" / "docs"
        docs.mkdir(parents=True)
        for name in names:
            (docs / name).write_bytes(b"<!DOCTYPE html><html><body>p</body></html>")
        out = root / "out"
        out.mkdir()
        cwd = pathlib.Path.cwd()
        try:
            os.chdir(root)
            with build.reproducible_tar(out / "wiki-offline.tar.gz") as tar:
                return build.add_param_versions(
                    tar, "wiki", root / "wiki" / "build" / "html", out, {}, {"wiki"}, {})
        finally:
            os.chdir(cwd)

    def test_versions_found_but_no_zstandard_is_an_error_on_stderr(self):
        said = io.StringIO()
        with tempfile.TemporaryDirectory() as tmp:
            held = sys.modules.get("zstandard", False)
            sys.modules["zstandard"] = None     # what an uninstalled module looks like
            try:
                with contextlib.redirect_stderr(said):
                    versions = self.carry(tmp, ["parameters-Copter-stable-V4.7.0.html",
                                                "parameters-Copter-stable-V4.6.0.html"])
            finally:
                if held is False:
                    del sys.modules["zstandard"]
                else:
                    sys.modules["zstandard"] = held
        self.assertEqual(versions, [])
        self.assertIn("ERROR", said.getvalue())
        self.assertIn("zstandard", said.getvalue())
        self.assertIn("2 parameter versions", said.getvalue())

    def test_no_stable_version_to_build_against_is_logged(self):
        try:
            import zstandard  # noqa: F401
        except ImportError:
            self.skipTest("zstandard not installed")
        said = io.StringIO()
        with tempfile.TemporaryDirectory() as tmp:
            with contextlib.redirect_stdout(said):
                versions = self.carry(tmp, ["parameters-Copter-beta-V4.8.0.html"])
        self.assertEqual(versions, [])
        self.assertIn("no stable parameter version", said.getvalue())


if __name__ == "__main__":
    unittest.main()

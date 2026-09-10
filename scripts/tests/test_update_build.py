"""Regression tests for incomplete Sphinx builds reaching publication."""

import multiprocessing
import os
import pathlib
import signal
import subprocess
import sys
import tempfile
import unittest
from contextlib import ExitStack
from types import SimpleNamespace
from unittest.mock import Mock, patch

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))

import update  # noqa: E402


def kill_build(wiki, fast):
    os.kill(os.getpid(), signal.SIGKILL)


class TestBuildPublication(unittest.TestCase):
    def test_child_exit_codes(self):
        for code in (0, 1, 2, 3, -9, -11):
            with self.subTest(code=code), patch.object(update, 'error'):
                proc = Mock(exitcode=code)
                proc.name = 'build_one_plane'
                procs = [proc]
                failed = update._reap_finished_procs(procs)
                self.assertEqual(failed, [] if code in (0, 2) else ['plane'])
                self.assertEqual(procs, [])
                proc.join.assert_called_once_with()

    @unittest.skipUnless('fork' in multiprocessing.get_all_start_methods(), 'requires fork and SIGKILL')
    def test_killed_build_does_not_cache_backup_or_publish(self):
        updater = update.WikiUpdater.__new__(update.WikiUpdater)
        updater.args = SimpleNamespace(site='plane', parallel=1, fast=True,
                                       clean_common=False, paramversioning=True,
                                       enablebackups=True, destdir='/unused', backupdestdir='/unused')
        with ExitStack() as stack:
            for name in ('check_imports', 'check_ref_directives', 'create_features_pages',
                         'copy_static_html_sites', 'copy_common_source_files'):
                stack.enter_context(patch.object(update, name))
            protected = [stack.enter_context(patch.object(update, name)) for name in
                         ('put_cached_parameters_files_in_sites', 'cache_parameters_files',
                          'make_backup', 'delete_old_wiki_backups', 'copy_build')]
            stack.enter_context(patch.object(update, 'build_one', kill_build))
            stack.enter_context(patch.object(update.multiprocessing, 'Process',
                                             multiprocessing.get_context('fork').Process))
            with self.assertRaises(SystemExit) as exc:
                updater.run()
            self.assertEqual(exc.exception.code, 1)
            for operation in protected:
                operation.assert_not_called()

    def test_sphinx_failure_status_is_not_treated_as_warnings(self):
        app = Mock(statuscode=1, _warncount=1)
        with tempfile.TemporaryDirectory() as tmp, patch.object(update, 'Sphinx', return_value=app):
            with self.assertRaises(SystemExit) as exc:
                update.build_one(tmp, False)
            self.assertEqual(exc.exception.code, 1)

    def test_required_build_artifacts(self):
        required = ('index.html', 'searchindex.js', 'objects.inv')
        for missing in required:
            with self.subTest(missing=missing), tempfile.TemporaryDirectory() as tmp:
                output = pathlib.Path(tmp, 'build', 'html')
                output.mkdir(parents=True)
                for name in required:
                    if name != missing:
                        (output / name).write_text('test', encoding='utf-8')
                with patch.object(update, 'ALL_WIKIS', [tmp]), self.assertRaises(SystemExit):
                    update.check_build(tmp)
                (output / missing).write_text('test', encoding='utf-8')
                with patch.object(update, 'ALL_WIKIS', [tmp]):
                    update.check_build(tmp)


class TestSphinxOutput(unittest.TestCase):
    def build(self, conf_extra='', guide='Guide\n=====\n'):
        with tempfile.TemporaryDirectory() as tmp:
            source = pathlib.Path(tmp, 'plane', 'source')
            source.mkdir(parents=True)
            (source / 'conf.py').write_text(
                "project = 'Build test'\nmaster_doc = 'index'\nexclude_patterns = ['excluded.rst']\n" + conf_extra,
                encoding='utf-8')
            (source / 'index.rst').write_text('Test\n====\n\n.. toctree::\n\n   guide\n', encoding='utf-8')
            (source / 'guide.rst').write_text(guide, encoding='utf-8')
            (source / 'excluded.rst').write_text('Excluded\n========\n', encoding='utf-8')
            # Each build gets a fresh Sphinx extension registry.
            return subprocess.run(
                [sys.executable, '-c',
                 'import os, sys, update; os.chdir(sys.argv[1]); '
                 'update.build_one("plane", False); update.check_build("plane")', tmp],
                cwd=REPO, capture_output=True, text=True, timeout=60)

    def test_complete_build_respects_excluded_sources(self):
        result = self.build()
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_warning_only_build_retains_warning_exit_code(self):
        result = self.build(guide='Guide\n=====\n\n:ref:`missing-label`\n')
        self.assertEqual(result.returncode, 2, result.stdout + result.stderr)

    def test_missing_page_after_successful_sphinx_build_is_fatal(self):
        result = self.build(conf_extra=(
            'from pathlib import Path\n'
            'def remove_page(app, exception):\n'
            '    Path(app.builder.get_outfilename("guide")).unlink()\n'
            'def setup(app):\n'
            '    app.connect("build-finished", remove_page)\n'))
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('Missing HTML output for 1 documents: guide', result.stderr)


if __name__ == '__main__':
    unittest.main()

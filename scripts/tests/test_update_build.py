"""Regression tests for incomplete Sphinx builds reaching publication."""

import json
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
                    (output / name).write_text('<html>test</html>', encoding='utf-8')
                manifest = {name: update.output_hash(output / name) for name in required}
                (output.parent / 'output-manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
                (output / missing).unlink()
                with patch.object(update, 'ALL_WIKIS', [tmp]), self.assertRaises(SystemExit):
                    update.check_build(tmp)
                (output / missing).write_text('<html>test</html>', encoding='utf-8')
                with patch.object(update, 'ALL_WIKIS', [tmp]):
                    update.check_build(tmp)


class TestParameterCache(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = pathlib.Path(tmp.name, 'repo')
        self.cache = pathlib.Path(tmp.name, 'old_params_mversion', 'ArduPlane')
        self.cache.mkdir(parents=True)
        self.output = self.root / 'plane/build/html/docs'
        self.output.mkdir(parents=True)
        self.name = 'parameters-Plane-stable-V4.6.0.html'
        self.cached = self.cache / self.name
        self.target = self.output / self.name
        self.valid = b'<html>Complete parameters</html>\n'
        cwd = patch.object(update.os, 'getcwd', return_value=str(self.root))
        cwd.start()
        self.addCleanup(cwd.stop)

    def test_corrupt_cache_does_not_overwrite_fresh_output(self):
        self.cached.write_bytes(b'')
        self.target.write_bytes(self.valid)
        update.put_cached_parameters_files_in_sites('plane')
        self.assertEqual(self.target.read_bytes(), self.valid)

    def test_corrupt_cache_cannot_fill_gap(self):
        for content in (b'', b'<html>Truncated parameters'):
            with self.subTest(content=content):
                self.cached.write_bytes(content)
                with self.assertRaises(SystemExit):
                    update.put_cached_parameters_files_in_sites('plane')
                self.assertFalse(self.target.exists())

    def test_valid_cache_fills_gap(self):
        self.cached.write_bytes(self.valid)
        update.put_cached_parameters_files_in_sites('plane')
        self.assertEqual(self.target.read_bytes(), self.valid)

    def test_failed_cache_copy_is_fatal_and_leaves_no_partial_page(self):
        self.cached.write_bytes(self.valid)

        def partial_copy(source, target):
            target.write_bytes(b'<html>Partial')
            raise OSError('injected copy error')

        with patch.object(update.shutil, 'copy2', side_effect=partial_copy), self.assertRaises(SystemExit):
            update.put_cached_parameters_files_in_sites('plane')
        self.assertEqual(list(self.output.iterdir()), [])


class TestSphinxOutput(unittest.TestCase):
    def build(self, conf_extra='', guide='Guide\n=====\n', after_build=''):
        with tempfile.TemporaryDirectory() as tmp:
            source = pathlib.Path(tmp, 'plane', 'source')
            source.mkdir(parents=True)
            (source / 'conf.py').write_text(
                "project = 'Build test'\nmaster_doc = 'index'\nexclude_patterns = ['excluded.rst']\n" + conf_extra,
                encoding='utf-8')
            (source / 'index.rst').write_text('Test\n====\n\n.. toctree::\n\n   guide\n', encoding='utf-8')
            (source / 'guide.rst').write_text(guide, encoding='utf-8')
            (source / 'excluded.rst').write_text('Excluded\n========\n', encoding='utf-8')
            (source / 'figure.svg').write_text(
                '<svg xmlns="http://www.w3.org/2000/svg" width="1" height="1"></svg>', encoding='utf-8')
            # Each build gets a fresh Sphinx extension registry.
            return subprocess.run(
                [sys.executable, '-c',
                 'import os, sys, update; os.chdir(sys.argv[1]); '
                 'update.build_one("plane", False); ' + after_build + 'update.check_build("plane")', tmp],
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

    def test_html_write_error_is_fatal(self):
        # Patch both Sphinx 7's open() and Sphinx 8's Path.write_text().
        conf = '''
import builtins
from pathlib import Path
import sphinx.builders.html as html
real_open = builtins.open
real_write_text = Path.write_text
class FailedWrite:
    def __init__(self, file): self.file = file
    def __enter__(self): return self
    def __exit__(self, *args): self.file.close()
    def write(self, data):
        self.file.write(data[:20])
        raise OSError(5, 'injected HTML write error')
def failing_open(path, mode='r', *args, **kwargs):
    file = real_open(path, mode, *args, **kwargs)
    if str(path).endswith('/guide.html') and mode == 'w':
        return FailedWrite(file)
    return file
def failing_write_text(path, data, *args, **kwargs):
    if path.name == 'guide.html':
        real_write_text(path, data[:20], *args, **kwargs)
        raise OSError(5, 'injected HTML write error')
    return real_write_text(path, data, *args, **kwargs)
html.open = failing_open
Path.write_text = failing_write_text
'''
        for data in ('data[:20]', 'data'):
            # Even a late I/O error after writing the closing tag is fatal.
            with self.subTest(data=data):
                result = self.build(conf_extra=conf.replace('data[:20]', data))
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn('Sphinx build exception for plane: error writing file', result.stderr)

    def test_image_copy_error_is_fatal(self):
        result = self.build(guide='Guide\n=====\n\n.. image:: figure.svg\n', conf_extra='''
import sphinx.builders.html as html
real_copyfile = html.copyfile
def failing_copyfile(src, dst, *args, **kwargs):
    if str(src).endswith('figure.svg'):
        raise OSError(5, 'injected image copy error')
    return real_copyfile(src, dst, *args, **kwargs)
html.copyfile = failing_copyfile
''')
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('injected image copy error', result.stderr)

    def test_truncated_page_without_warning_is_fatal(self):
        result = self.build(conf_extra='''
from pathlib import Path
def truncate(app, exception):
    Path(app.builder.get_outfilename('guide')).write_text('<html>Truncated')
def setup(app):
    app.connect('build-finished', truncate)
''')
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('Incomplete HTML output', result.stderr)

    def test_changed_output_after_build_is_fatal(self):
        result = self.build(after_build=(
            'from pathlib import Path; '
            'Path("plane/build/html/guide.html").write_text("<html>Wrong page</html>"); '))
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('Build output changed after Sphinx completed', result.stdout + result.stderr)

    def test_failed_incremental_build_invalidates_previous_manifest(self):
        result = self.build(after_build='''
from pathlib import Path
from unittest.mock import patch
try:
    with patch.object(update, 'Sphinx', side_effect=RuntimeError('injected')):
        update.build_one('plane', True)
except SystemExit:
    pass
assert not Path('plane/build/output-manifest.json').exists()
''')
        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn('output-manifest.json', result.stdout + result.stderr)

    def test_incremental_retry_regenerates_missing_images(self):
        result = self.build(guide='Guide\n=====\n\n.. image:: figure.svg\n', after_build='''
from pathlib import Path
Path('plane/build/html/_images/figure.svg').unlink()
try:
    update.build_one('plane', True)
except SystemExit as exc:
    assert exc.code == 2  # Sphinx warns about registration in a second app.
assert Path('plane/build/html/_images/figure.svg').is_file()
''')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('previous output is incomplete or unverified', result.stdout)


if __name__ == '__main__':
    unittest.main()

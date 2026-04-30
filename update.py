#!/usr/bin/env python3
"""

This script updates and rebuilds wiki sources from Github and from
parameters on the test server.

It is intended to be run on the main wiki server or
locally within the project's Vagrant environment.

Build notes:

* First step is always a fetch and pull from git (master).
  * Default is just a normal fetch and pull from master

  * Common topics are copied from /common/source/docs.
  * If the --clean-common option is "True" then common files will be recopied (instead of just copying changed files).
    This is useful if you want to make sure all common files are updated,
    but it will cause a full rebuild of all wikis (instead of an incremental build).

  * Topics are copied based on information in the copywiki shortcode.
    For example a topic marked as below would only be copied to copter
     and plane wikis:
    [copywiki destination="copter,plane"]
  * Topics that don't have a [copywiki] will be copied to wikis
    the DEFAULT_COPY_WIKIS list
  * Copied topics are stripped of the 'copywiki' shortcode in the destination.
  * Copied topics are stripped of any content not marked for the target wiki
    using the "site" shortcode:
    [site wiki="plane,rover"]conditional content[/site]

Parameters files are fetched from autotest using requests

"""

import argparse
import errno
import filecmp
import glob
import gzip
import hashlib
import json
import logging
import multiprocessing
import os
import platform
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, TimeoutError
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional
from urllib.parse import urlparse

import requests
from sphinx.application import Sphinx

import rst_table
from frontend.scripts import get_discourse_posts
from scripts.dedupe_params import dedupe_periph_net_parameters

if sys.version_info < (3, 8):
    print("Minimum python version is 3.8")
    sys.exit(1)


# Configure logging
class ColoredFormatter(logging.Formatter):
    """Simple ANSI-coloured formatter for terminal output."""
    COLORS = {
        logging.DEBUG: '\033[36m',      # cyan
        logging.INFO: '\033[32m',       # green
        logging.WARNING: '\033[33m',    # yellow
        logging.ERROR: '\033[31m',      # red
        logging.CRITICAL: '\033[1;31m', # bold red
    }
    RESET = '\033[0m'

    def format(self, record):
        # Apply colour only when output is a tty
        if hasattr(sys.stdout, 'isatty') and sys.stdout.isatty() \
                and not os.environ.get('CI') and not os.environ.get('GITHUB_ACTIONS'):
            color = self.COLORS.get(record.levelno, '')
            record.levelname = f"{color}{record.levelname}{self.RESET}"
        return super().format(record)


class ErrorStoreHandler(logging.Handler):
    """Allow to store errors for later usage."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.error_messages = []

    def emit(self, record):
        self.error_messages.append(record.getMessage())


# The StreamHandler logs to the console
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setFormatter(ColoredFormatter('[update.py]: [%(levelname)s]: %(message)s'))
stream_handler.setLevel(logging.INFO)
logging.basicConfig(level=logging.INFO, handlers=[stream_handler])
logger = logging.getLogger(__name__)

# The ErrorStoreHandler stores the messages
error_store_handler = ErrorStoreHandler()
error_store_handler.setLevel(logging.ERROR)
logger.addHandler(error_store_handler)

# Keep noisy third-party network logs quiet by default
logging.getLogger('urllib3').setLevel(logging.WARNING)


DEFAULT_COPY_WIKIS = ['copter', 'plane', 'rover', 'sub']
ALL_WIKIS = [
    'copter',
    'plane',
    'rover',
    'sub',
    'antennatracker',
    'dev',
    'planner',
    'planner2',
    'ardupilot',
    'mavproxy',
    'frontend',
    'blimp',
]
COMMON_DIR = 'common'

WIKI_NAME_TO_VEHICLE_NAME = {
    'copter': 'Copter',
    'plane': 'Plane',
    'rover': 'Rover',
    'sub': 'Sub',
    'blimp': 'Blimp',
}

PARAMETER_SITE = {
    'rover': 'APMrover2',
    'copter': 'ArduCopter',
    'plane': 'ArduPlane',
    'sub': 'ArduSub',
    'antennatracker': 'AntennaTracker',
    'AP_Periph': 'AP_Periph',
    'blimp': 'Blimp',
}
LOGMESSAGE_SITE = {
    'rover': 'Rover',
    'copter': 'Copter',
    'plane': 'Plane',
    'sub': 'Sub',
    'antennatracker': 'Tracker',
    'blimp': 'Blimp',
}

N_BACKUPS_RETAIN = 10

# Global HTTP session for connection reuse and caching
_http_session = None


def info(str_to_print: str) -> None:
    """Info output."""
    logger.info(str_to_print)


def debug(str_to_print: str) -> None:
    """Debug output if verbose is set."""
    logger.debug(str_to_print)


def error(str_to_print) -> None:
    """Show and count the errors."""
    logger.error(f"{str_to_print}")


def fatal(str_to_print) -> None:
    """Show and exit on errors."""
    logger.critical(f"{str_to_print}")
    sys.exit(1)


def get_http_session():
    """Get or create a persistent HTTP session with connection pooling"""
    global _http_session
    if _http_session is None:
        _http_session = requests.Session()
        _http_session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; ArduPilotWikiUpdater/1.0)',
            'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8',
            'Connection': 'keep-alive'
        })
        # Add retry logic for better reliability
        from requests.adapters import HTTPAdapter
        from urllib3.util.retry import Retry

        retries = Retry(
            total=3,
            backoff_factor=1,
            status_forcelist=[429, 500, 502, 503, 504],
            allowed_methods=["HEAD", "GET", "OPTIONS"]
        )
        adapter = HTTPAdapter(max_retries=retries)
        _http_session.mount("https://", adapter)
        _http_session.mount("http://", adapter)

    return _http_session


def remove_if_exists(filepath):
    try:
        os.remove(filepath)
    except OSError as e:
        if e.errno != errno.ENOENT:
            raise e


def fetch_and_rename(fetchurl: str, target_file: str, new_name: str) -> None:
    # Fetch into a temporary filename (new_name) and only replace the
    # real target if content actually changed. This avoids touching
    # mtimes when the fetched content is identical and prevents
    # unnecessary Sphinx rebuilds.
    fetch_url(fetchurl, fpath=new_name, verbose=False)

    try:
        # If target exists and is identical, remove fetched temp and skip replace
        if os.path.exists(target_file) and filecmp.cmp(new_name, target_file, shallow=False):
            debug(f"No change for {target_file} (fetched content identical)")
            os.remove(new_name)
            return
    except OSError as e:
        debug(f"Failed to compare fetched file and target: {e}")
    info(f"Renaming {new_name} to {target_file}")
    os.replace(new_name, target_file)


def fetch_url(fetchurl: str, fpath: Optional[str] = None, verbose: bool = True) -> None:
    """Fetches content at url and puts it in a file corresponding to the filename in the URL"""
    info(f"Fetching {fetchurl}")
    # For larger files or when cache fails, use streaming download with progress
    session = get_http_session()

    total_size = 0

    if verbose:
        total_size = get_request_file_size(fetchurl)

    response = session.get(fetchurl, stream=True, timeout=30)
    response.raise_for_status()

    filename = fpath or os.path.basename(urlparse(fetchurl).path)

    downloaded_size = 0
    chunk_size = 64 * 1024  # Increased chunk size for better performance

    with open(filename, 'wb') as out_file:
        if verbose and total_size > 0:
            print("[update.py]: Completed : 0%", end='', file=sys.stdout)  # intentionally use of print for formatting
        completed_last = 0
        for chunk in response.iter_content(chunk_size=chunk_size):
            out_file.write(chunk)
            downloaded_size += len(chunk)

            # progress bar
            if verbose and total_size > 0:
                completed = downloaded_size * 100 // total_size
                if completed - completed_last > 10 or completed == 100:
                    print(f"..{completed}%", end='')
                    completed_last = completed
        if verbose:
            print()  # Newline to correct the console cursor position


def get_request_file_size(url: str) -> int:
    """Get file size from URL using HEAD request with session reuse"""

    session = get_http_session()
    headers = {'Accept-Encoding': 'identity'}  # needed as request use compression by default
    hresponse = session.head(url, headers=headers, timeout=30)

    if 'Content-Length' in hresponse.headers:
        size = int(hresponse.headers['Content-Length'])
        return size

    return 0


def fetchparameters(site: Optional[str] = None, cache: Optional[str] = None) -> None:
    dataname = "Parameters"
    fetch_ardupilot_generated_data(
        PARAMETER_SITE,
        f"https://autotest.ardupilot.org/{dataname}", # noqa: E231
        f"{dataname}.rst",
        f"{dataname.lower()}.rst",
        site,
        cache,
    )


def fetchlogmessages(site: Optional[str] = None, cache: Optional[str] = None) -> None:
    dataname = "LogMessages"
    fetch_ardupilot_generated_data(
        LOGMESSAGE_SITE,
        f"https://autotest.ardupilot.org/{dataname}",  # noqa: E231
        f"{dataname}.rst",
        f"{dataname.lower()}.rst",
        site,
        cache,
    )


def fetch_ardupilot_generated_data(site_mapping: Dict, base_url: str, sub_url: str, document_name: str,
                                   site: Optional[str] = None, cache: Optional[str] = None) -> None:
    """Fetches the data for all the sites from the test server and
    copies them to the correct location.

    This is always run as part of a build (i.e. no checking to see if
    parameters or logmessage have changed.)

    """
    urls: List[str] = []
    targetfiles: List[str] = []
    names: List[str] = []

    for key, value in site_mapping.items():
        fetchurl = f'{base_url}/{value}/{sub_url}'
        targetfile = f'./{key}/source/docs/{document_name}'
        if key == 'AP_Periph':
            targetfile = f'./dev/source/docs/AP_Periph-{sub_url}'
        if cache:
            if not os.path.exists(targetfile):
                raise Exception(f"Asked to use cached files, but {targetfile} does not exist")
            continue
        if site == key or site is None or (site == 'dev' and key == 'AP_Periph'):
            urls.append(fetchurl)
            targetfiles.append(targetfile)
            names.append(f"{value}_{document_name}")

    with ThreadPoolExecutor(max_workers=4) as executor:  # Limit concurrent downloads
        tasks = []
        for url, target, name in zip(urls, targetfiles, names):
            task = executor.submit(fetch_and_rename, url, target, name)
            tasks.append(task)

        # Wait for all downloads to complete
        for task in tasks:
            try:
                task.result(timeout=5*60)
            except (TimeoutError, OSError, requests.RequestException) as e:
                error(f"Download failed: {e}")


def build_one(wiki, fast):
    """build one wiki"""
    info(f'build_one: {wiki}')

    source_dir = os.path.join(wiki, 'source')
    output_dir = os.path.join(wiki, 'build')
    html_dir = os.path.join(output_dir, 'html')
    doctree_dir = os.path.join(output_dir, 'doctrees')

    # This will fail if there's no folder to clean, so we check first
    if not fast and os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    try:
        app = Sphinx(
            buildername='html',
            confdir=source_dir,
            doctreedir=doctree_dir,
            outdir=html_dir,
            parallel=2,
            srcdir=source_dir,
        )
        app.build()
    except Exception as exc:
        print(f"[update.py]: [ERROR]: Sphinx build exception for {wiki}: {exc}", file=sys.stderr)
        sys.exit(1)

    if app._warncount > 0:
        sys.exit(2)


def _reap_finished_procs(procs):
    """Join any finished child processes and report their exit status."""
    for p in procs[:]:
        if p.exitcode is not None:
            wiki_name = "_".join(p.name.split("_")[2:])
            p.join()
            procs.remove(p)
            if p.exitcode == 1:
                error(f"Sphinx build error for {wiki_name}")
            elif p.exitcode == 2:
                error(f"Sphinx warnings were emitted for {wiki_name}")


def sphinx_make(site, parallel, fast):
    """
    Calls 'make html' to build each site
    """
    done = set()
    wikis = set(ALL_WIKIS[:])
    procs = []

    while len(done) != len(wikis):
        wiki = list(wikis.difference(done))[0]
        done.add(wiki)
        if site == 'common' or site == 'frontend':
            continue
        if wiki == 'frontend':
            continue
        if site is not None and not site == wiki:
            continue
        p = multiprocessing.Process(name=f"build_one_{wiki}", target=build_one, args=(wiki, fast))
        p.start()
        procs.append(p)
        while parallel != -1 and len(procs) >= parallel:
            _reap_finished_procs(procs)
            time.sleep(0.1)
    while len(procs) > 0:
        _reap_finished_procs(procs)
        time.sleep(0.1)


def check_build(site):
    """
    check that build was successful
    """
    if platform.system() == "Windows":
        debug("Skipping check_build on windows")
        return
    for wiki in ALL_WIKIS:
        if site is not None and site != wiki:
            continue
        if wiki in ['common', 'frontend']:
            continue
        index_html = os.path.join(wiki, "build", "html", "index.html")
        if not os.path.exists(index_html):
            fatal(f"{wiki} site not built - missing {index_html}")


def copy_build(site, destdir):
    """
    Copies each site into the target location
    """
    for wiki in ALL_WIKIS:
        if site == 'common':
            continue
        if site is not None and site != wiki:
            continue
        if wiki == 'frontend':
            continue
        debug(f'Copy: {wiki}')
        targetdir = os.path.join(destdir, wiki)
        debug("Creating backup")
        olddir = os.path.join(destdir, 'old')
        debug(f'Recreating {olddir}')
        if os.path.exists(olddir):
            shutil.rmtree(olddir)
        os.makedirs(olddir)
        if os.path.exists(targetdir):
            debug(f'Moving {targetdir} into {olddir}')
            shutil.move(targetdir, olddir)
        # copy new dir to targetdir
        # progress("DEBUG: targetdir: %s" % targetdir)
        # sourcedir='./%s/build/html/*' % wiki
        sourcedir = f'./{wiki}/build/html/'
        # progress("DEBUG: sourcedir: %s" % sourcedir)
        # progress('DEBUG: mv %s %s' % (sourcedir, destdir) )

        html_moved_dir = os.path.join(destdir, 'html')
        try:
            shutil.move(sourcedir, html_moved_dir)
            # Rename move! (single move to html/* failed)
            shutil.move(html_moved_dir, targetdir)
            debug(f"Moved to {targetdir}")
        except shutil.Error:
            error(f"FAIL moving output to {targetdir}")

        # copy jquery
        os.makedirs(os.path.join(targetdir, '_static'), exist_ok=True)

        # delete the old directory
        debug(f'Removing {olddir}')
        shutil.rmtree(olddir)


def make_backup(building_time, site, destdir, backupdestdir):
    """
    backup current site
    """
    for wiki in ALL_WIKIS:
        if site == 'common':
            continue
        if site is not None and site != wiki:
            continue
        if wiki == 'frontend':
            continue
        debug(f'Backing up: {wiki}')

        targetdir = os.path.join(destdir, wiki)
        os.makedirs(targetdir, exist_ok=True)

        if not os.path.exists(targetdir):
            fatal(f"FAIL backup when looking for folder {targetdir}")

        bkdir = os.path.join(backupdestdir, f"{building_time}-wiki-bkp", str(wiki))
        debug(f'Checking {bkdir}')
        os.makedirs(bkdir, exist_ok=True)
        debug(f'Copying {targetdir} into {bkdir}')
        try:
            subprocess.check_call(["rsync", "-a", "--delete", f"{targetdir}/", bkdir])
        except subprocess.CalledProcessError as ex:
            error(ex)
            fatal(f"Failed to backup {wiki}")


def delete_old_wiki_backups(folder, n_to_keep):
    try:
        debug(f'Checking number of backups in folder {folder}')
        backup_folders = glob.glob(f"{folder}/*-wiki-bkp/")
        backup_folders.sort()
        if len(backup_folders) > n_to_keep:
            for i in range(0, len(backup_folders) - n_to_keep):
                if '-wiki-bkp' in str(backup_folders[i]):
                    debug(f'Deleting folder {str(backup_folders[i])}')
                    shutil.rmtree(str(backup_folders[i]))
                else:
                    debug(f'Ignoring folder {str(backup_folders[i])} because it does not look like a auto generated wiki backup folder')  # noqa: E501
        else:
            debug(f'No old backups to delete in {folder}')
    except Exception as e:
        error(f'Error on deleting some previous wiki backup folders: {e}')


def copy_common_source_files(start_dir=COMMON_DIR, clean_common=False):
    """
    copies files common to all Wikis to the source directories for each Wiki

    Args:
        start_dir: Directory containing common source files
        clean_common: If True, delete and recopy all common files (old behavior).
                     If False, only copy files that have changed (faster incremental builds).
    """

    # Create destination folders that might be needed (if don't exist)
    for wiki in ALL_WIKIS:
        os.makedirs(f'{wiki}/source/docs', exist_ok=True)
        os.makedirs(f'{wiki}/source/_static', exist_ok=True)

    # Build a set of expected common files per wiki (to detect stale files)
    # Format: {wiki: set of filenames that should exist}
    expected_common_files = {wiki: set() for wiki in ALL_WIKIS}

    # First pass: determine which files should exist in each wiki
    for root, dirs, files in os.walk(start_dir):
        for file in files:
            if file.endswith(".rst"):
                source_file_path = os.path.join(root, file)
                with open(source_file_path, 'r', encoding='utf-8') as f:
                    source_content = f.read()
                targets = get_copy_targets(source_content)
                for wiki in targets:
                    expected_common_files[wiki].add(file)

    # Remove stale common files (files that exist but shouldn't)
    files_removed = 0
    for wiki in ALL_WIKIS:
        existing_common_files = glob.glob(f'{wiki}/source/docs/common-*.rst')
        for filepath in existing_common_files:
            filename = os.path.basename(filepath)
            if filename not in expected_common_files[wiki]:
                debug(f'Removing stale common file: {filepath}')
                os.remove(filepath)
                files_removed += 1

    if clean_common:
        # Clean all existing common topics for full rebuild
        for wiki in ALL_WIKIS:
            files = glob.glob(f'{wiki}/source/docs/common-*.rst')
            for f in files:
                debug(f'Remove existing common: {f}')
                os.remove(f)

    debug("Copying common source files to each Wiki")
    files_copied = 0
    files_skipped = 0

    for root, dirs, files in os.walk(start_dir):
        for file in files:
            source_file_path = Path(root) / file
            if file.endswith(".rst"):
                # debug("  FILE: %s" % file)
                source_content = source_file_path.read_text(encoding='utf-8')
                targets = get_copy_targets(source_content)
                for wiki in targets:
                    content = strip_content(source_content, wiki)
                    targetfile = Path(wiki) / "source" / "docs" / file

                    # Only write if content has changed (preserves timestamps for unchanged files)
                    # Compare against the file content after stripping copywiki shortcodes.
                    if not clean_common and targetfile.exists():
                        if targetfile.read_text(encoding='utf-8') == content:
                            files_skipped += 1
                            continue

                    targetfile.write_text(content, encoding='utf-8')
                    files_copied += 1
            elif file.endswith(".css"):
                for wiki in ALL_WIKIS:
                    targetfile = Path(wiki) / "source" / "_static" / file
                    # Only copy if different
                    if not clean_common and targetfile.exists() and filecmp.cmp(source_file_path, targetfile, shallow=False):
                        continue
                    shutil.copy2(source_file_path, targetfile)
            elif file.endswith(".js"):
                source_content = source_file_path.read_text(encoding='utf-8')
                targets = get_copy_targets(source_content)
                for wiki in targets:
                    content = strip_content(source_content, wiki)
                    targetfile = Path(wiki) / "source" / "_static" / file

                    # Only write if content has changed
                    if not clean_common and targetfile.exists():
                        if targetfile.read_text(encoding='utf-8') == content:
                            continue

                    targetfile.write_text(content, encoding='utf-8')

    info(f"Common files: {files_copied} copied, {files_skipped} unchanged, {files_removed} removed")


def get_copy_targets(content):
    p = re.compile(r'\[copywiki.*?destination\=\"(.*?)\".*?\]', flags=re.DOTALL)
    m = p.search(content)
    targetset = set()
    if m:
        targets = m.group(1).split(',')
        for item in targets:
            targetset.add(item.strip())
    else:
        targetset = set(DEFAULT_COPY_WIKIS)
    return targetset


def strip_content(content, site):
    """
    Strips the copywiki shortcode. Removes content for other sites and
    the [site] shortcode itself.
    """

    def fix_copywiki_shortcode(matchobj):
        """
        Strip the copywiki shortcode if found (just return "nothing" to
        result of re)
        """
        # logmatch_code(matchobj, 'STRIP')
        # progress("STRIPPED")
        return ''

    # Remove the copywiki from content
    newText = re.sub(r'\[copywiki.*?\]',
                     fix_copywiki_shortcode,
                     content,
                     flags=re.MULTILINE)

    def fix_site_shortcode(matchobj):
        # logmatch_code(matchobj, 'SITESC_')
        sitelist = matchobj.group(1)
        # progress("SITES_BLOCK: %s" % sitelist)
        if site not in sitelist:
            # progress("NOT")
            return ''
        else:
            # progress("YES")
            return matchobj.group(2)
    # Remove the site shortcode from content
    newText = re.sub(r'\[site\s.*?wiki\=\"(.*?)\".*?\](.*?)\[\/site\]',
                     fix_site_shortcode,
                     newText,
                     flags=re.DOTALL)

    return newText


def logmatch_code(matchobj, prefix):

    for i in range(9):
        try:
            info(f"{prefix} m{i}: {matchobj.group(i)}")
        except IndexError:  # The object has less groups than expected
            error(f"{prefix}: except m{i}")


def is_the_same_file(file1, file2):
    """ Compare two files using their SHA256 hashes"""
    def file_hash(path, algo="sha256", chunk_size=8192):
        h = hashlib.new(algo)
        with open(path, "rb") as f:
            chunk = f.read(chunk_size)
            while chunk:
                h.update(chunk)
                chunk = f.read(chunk_size)
        return h.hexdigest()
    return file_hash(file1) == file_hash(file2)


def cleanup_versioned_parameters(site=None):
    """
    It removes all versioned parameters files and JSON files in order to
    prepare the wiki for a new build with the new versioned parameters.
    """

    for key, value in PARAMETER_SITE.items():

        if site == key or site is None:

            # Remove old versioned param files
            if 'antennatracker' in key.lower():  # To main the original script approach instead of the build_parameters.py approach.  # noqa: E501
                old_parameters_mask = "./AntennaTracker/source/docs/parameters-AntennaTracker-"
            else:
                old_parameters_mask = f"./{key}/source/docs/parameters-{key.title()}-"

            old_parameters_files = [
                f for f in glob.glob(f"{old_parameters_mask}*.rst")]
            for filename in old_parameters_files:
                debug(f"Erasing rst {filename}")
                remove_if_exists(filename)

            # Remove old json file
            if 'antennatracker' in key.lower():  # To main the original script approach instead of the build_parameters.py approach.  # noqa: E501
                target_json_file = './AntennaTracker/source/_static/parameters-AntennaTracker.json'
            else:
                target_json_file = f'./{value}/source/_static/parameters-{key.title()}.json'
            debug(f"Erasing json {target_json_file}")
            remove_if_exists(target_json_file)


def fetch_versioned_parameters(site=None):
    """
    It relies on "build_parameters.py" be executed before the "update.py"

    Once the generated files are on ../new_params_mversion it, put all
    parameters and JSON files in their destinations.
    """

    for key, value in PARAMETER_SITE.items():

        if key == 'AP_Periph': # workaround until create a versioning for AP_Periph in firmware server
            fetchurl = f'https://autotest.ardupilot.org/Parameters/{value}/Parameters.rst'
            targetfile = './dev/source/docs/AP_Periph-Parameters.rst'
            fetch_and_rename(fetchurl, targetfile, 'Parameters.rst')

        else: # regular versioning

            if site == key or site is None:
                # Remove old param single file
                single_param_file = f'./{key}/source/docs/parameters.rst'
                debug(f"Erasing {single_param_file}")
                remove_if_exists(single_param_file)

                cleanup_versioned_parameters(key)

                # Moves the updated JSON file
                if 'antennatracker' in key.lower():  # To main the original script approach instead of the build_parameters.py approach.  # noqa: E501
                    vehicle_json_file = f"{os.getcwd()}/../new_params_mversion/AntennaTracker/parameters-AntennaTracker.json"  # noqa: E501
                else:
                    vehicle_json_file = f"{os.getcwd()}/../new_params_mversion/{value}/parameters-{key.title()}.json"
                new_file = f"{key}/source/_static/{vehicle_json_file[vehicle_json_file.rfind('/') + 1:]}"
                try:
                    debug(f"Moving {vehicle_json_file}")
                    # os.rename(vehicle_json_file, new_file)
                    shutil.copy2(vehicle_json_file, new_file)
                except Exception as e:
                    error(e)
                    pass

                # Copy all parameter files to vehicle folder IFF it is new
                try:
                    new_parameters_folder = f"{os.getcwd()}/../new_params_mversion/{value}/"
                    new_parameters_files = [
                        f for f in glob.glob(f"{new_parameters_folder}*.rst")
                    ]
                except Exception as e:
                    error(e)
                    pass
                for filename in new_parameters_files:
                    # Check possible cached version
                    try:
                        new_file = f"{key}/source/docs/{filename[filename.rfind('/') + 1:]}"
                        if not os.path.isfile(new_file):
                            debug(f"Copying {filename} to {new_file} (target file does not exist)")
                            shutil.copy2(filename, new_file)
                        elif os.path.isfile(filename.replace("new_params_mversion", "old_params_mversion")): # The cached file exists?  # noqa: E501

                            # Temporary debug messages to help with cache tasks.
                            debug("Check cache: {} against {}".format(filename, filename.replace("new_params_mversion", "old_params_mversion")))  # noqa: E501
                            debug("Check cache with filecmp.cmp: {}".format(filecmp.cmp(filename, filename.replace("new_params_mversion", "old_params_mversion"))))  # noqa: E501
                            debug("Check cache with sha256: {}".format(is_the_same_file(filename, filename.replace("new_params_mversion", "old_params_mversion"))))  # noqa: E501

                            if ("parameters.rst" in filename) or (not filecmp.cmp(filename, filename.replace("new_params_mversion", "old_params_mversion"))):    # It is different?  OR is this one the latest. | Latest file must be built every time in order to enable Sphinx create the correct references across the wiki.  # noqa: E501
                                debug(f"Overwriting {filename} to {new_file}")
                                shutil.copy2(filename, new_file)
                            else:
                                debug(f"It will reuse the last build of {new_file}")
                        else:   # If not cached, copy it anyway.
                            debug(f"Copying {filename} to {new_file}")
                            shutil.copy2(filename, new_file)

                    except Exception as e:
                        error(e)
                        pass


def cache_parameters_files(site=None):
    """
    For each vechile: put new_params_mversion/ content in
    old_params_mversion/ folders and .html built files as well.
    """
    for key, value in PARAMETER_SITE.items():
        if (site == key or site is None) and (key != 'AP_Periph'):  # and (key != 'AP_Periph') workaround until create a versioning for AP_Periph in firmware server # noqa: E501
            try:
                old_parameters_folder = f"{os.getcwd()}/../old_params_mversion/{value}/"
                old_parameters_files = [
                    f for f in glob.glob(f"{old_parameters_folder}*.*")
                ]
                for file in old_parameters_files:
                    debug(f"Removing {file}")
                    os.remove(file)

                new_parameters_folder = f"{os.getcwd()}/../new_params_mversion/{value}/"
                new_parameters_files = [
                    f for f in glob.glob(f"{new_parameters_folder}parameters-*.rst")
                ]
                for filename in new_parameters_files:
                    debug(f"Copying {filename} to {old_parameters_folder}")
                    shutil.copy2(filename, old_parameters_folder)

                built_folder = f"{os.getcwd()}/{key}/build/html/docs/"
                built_parameters_files = [
                    f for f in glob.glob(f"{built_folder}parameters-*.html")
                ]
                for built in built_parameters_files:
                    debug(f"Copying {built} to {old_parameters_folder}")
                    shutil.copy2(built, old_parameters_folder)

            except Exception as e:
                error(e)
                pass


def put_cached_parameters_files_in_sites(site=None):
    """
    For each vechile: put built .html files in site folder

    """
    for key, value in PARAMETER_SITE.items():
        if (site == key or site is None) and (key != 'AP_Periph'): # and (key != 'AP_Periph') workaround until create a versioning for AP_Periph in firmware server # noqa: E501
            try:
                built_folder = f"{os.getcwd()}/../old_params_mversion/{value}/"
                built_parameters_files = [
                    f for f in glob.glob(f"{built_folder}parameters-*.html")
                ]
                vehicle_folder = f"{os.getcwd()}/{key}/build/html/docs/"
                debug(f"Site {site} getting previously built files from {built_folder}")
                for built in built_parameters_files:
                    if "latest" not in built:  # latest parameters files must be built every time
                        debug(f"Reusing built {built} in {vehicle_folder} ")
                        shutil.copy(built, vehicle_folder)
            except Exception as e:
                error(e)
                pass


def update_frontend_json():
    """
    Frontend get posts from Forum server and insert it into JSON
    """
    debug('Running script to get last posts from forum server.')
    try:
        get_discourse_posts.main()
    except Exception as e:
        error(e)
        pass


def copy_static_html_sites(site, destdir):
    """
    Copy pure HTML folder the same way that Sphinx builds it
    """
    if (site in ['frontend', None]) and (destdir is not None):
        debug('Copying static sites (only frontend so far).')
        update_frontend_json()
        folder = 'frontend'
        try:
            site_folder = f"{os.getcwd()}/{folder}"
            targetdir = os.path.join(destdir, folder)
            shutil.rmtree(targetdir, ignore_errors=True)
            shutil.copytree(site_folder, targetdir)
        except Exception as e:
            error(e)
            pass


def check_imports():
    '''check key imports work'''
    import importlib.metadata
    # package names to check the versions of. Note that these can be different than the string used to import the package
    required_packages = ["sphinx_rtd_theme>=1.3.0", "sphinxcontrib.youtube>=1.2.0", "sphinx>=7.1.2", "docutils<0.19"]
    for package in required_packages:
        debug(f"Checking for {package}")
        try:
            importlib.metadata.version(package.split("<")[0].split(">=")[0])
        except importlib.metadata.PackageNotFoundError as ex:
            error(ex)
            fatal(f'Require {package}\nPlease run the wiki build setup script "Sphinxsetup"')
    debug("Imports OK")


def check_ref_directives():
    '''check formatting around ref directive that sphinx does not warn about'''
    character_before_ref_tag = re.compile(r"[a-zA-Z0-9_:]:ref:")
    character_after_ref_tag = re.compile(r"(:ref:`.*?`[_]{0,2}) ([\.,:])")

    # don't check "common="" files in vehicle wikis
    skipped_files = set()
    for wiki in ALL_WIKIS:
        skipped_files.update(glob.glob(f'{wiki}/source/docs/common-*.rst'))
    wiki_glob = set(glob.glob("**/*.rst", recursive=True))
    files_to_check = wiki_glob.difference(skipped_files)
    for f in files_to_check:
        with open(f, "r", encoding='utf-8') as file:
            try:
                for i, line in enumerate(file.readlines()):
                    if character_before_ref_tag.search(line):
                        error(f'Remove character before ref directive in "{f}" on line number {i+1}')
                    if character_after_ref_tag.search(line):
                        error(f'Remove character after ref directive in "{f}" on line number {i+1}')
            except UnicodeDecodeError as ex:
                print(f"UnicodeError in {f}: ", ex)
                sys.exit(1)


def create_features_pages(site):
    '''for each vehicle, write out a page containing features for each
    supported board'''

    debug("Creating features pages")

    # grab build_options which allows us to map from define to name
    # and description.  Create a convenience hash for it
    remove_if_exists("build_options.py")
    fetch_url("https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/scripts/build_options.py")
    import build_options
    build_options_by_define = {}
    for feature in build_options.BUILD_OPTIONS:
        build_options_by_define[feature.define] = feature

    # fetch and load most-recently-built features.json
    remove_if_exists("features.json.gz")
    fetch_url("https://firmware.ardupilot.org/features.json.gz")
    features_json = json.load(gzip.open("features.json.gz"))
    if features_json["format-version"] != "1.0.0":
        error("bad format version")
        return
    features = features_json["features"]

    # progress("features: (%s)" % str(features))
    for wiki in WIKI_NAME_TO_VEHICLE_NAME.keys():
        debug(wiki)
        if site is not None and site != wiki:
            continue
        if wiki not in WIKI_NAME_TO_VEHICLE_NAME:
            continue
        vehicletype = WIKI_NAME_TO_VEHICLE_NAME[wiki]
        content = create_features_page(features, build_options_by_define, vehicletype)
        if wiki == "AP_Periph":
            destination_filepath = "dev/source/docs/periph-binary-features.rst"
        else:
            destination_filepath = f"{wiki}/source/docs/binary-features.rst"
        # make .../docs/ directory if it doesn't already exist
        os.makedirs(os.path.dirname(destination_filepath), exist_ok=True)
        with open(destination_filepath, "w") as f:
            f.write(content)


def reference_for_board(board):
    '''return a string suitable for creating an anchor in RST to make
    board's feature table linkable'''
    return f"FEATURE_{board}"


def create_features_page(features, build_options_by_define, vehicletype):
    features_by_platform = {}
    for build in features:
        # progress("build: (%s)" % str(build))
        if build["vehicletype"] != vehicletype:
            continue
        features_by_platform[build["platform"]] = build["features"]

    column_headings = ["Category", "Feature", "Included", "Description"]
    all_tables = ""
    for platform_key in sorted(features_by_platform.keys(), key=lambda x : x.lower()):
        rows = []
        platform_features = features_by_platform[platform_key]
        sorted_platform_features_in = []
        sorted_platform_features_not_in = []
        features_in = {}
        for feature in platform_features:
            feature_in = not feature.startswith("!")
            if not feature_in:
                feature = feature[1:]
            features_in[feature] = feature_in
            try:
                build_options = build_options_by_define[feature]
            except KeyError:
                # mismatch between build_options.py and features.json
                error(f"feature {feature} ({platform_key},{vehicletype}) not in build_options.py")
                continue
            if feature_in:
                some_list = sorted_platform_features_in
            else:
                some_list = sorted_platform_features_not_in
            some_list.append((build_options.category, feature))

        sorted_platform_features = (
            sorted(sorted_platform_features_not_in, key=lambda x : x[0] + x[1]) +
            sorted(sorted_platform_features_in, key=lambda x : x[0] + x[1]))

        for (category, feature) in sorted_platform_features:
            build_options = build_options_by_define[feature]
            row = [category, build_options.label]
            if features_in[feature]:
                row.append("Yes")
            else:
                row.append("No")
            row.append(build_options.description)
            if not features_in[feature]:
                # for now, do not include features that are on the
                # board, just those that aren't, per Henry's request:
                rows.append(row)
        if len(rows) == 0:
            t = ""
        else:
            t = rst_table.tablify(rows, headings=column_headings)
        underline = "-" * len(platform_key)
        all_tables += (f'''
.. _{reference_for_board(platform_key)}:

{platform_key}
{underline}

{t}
''')

    index = ""
    for board in sorted(features_by_platform.keys(), key=lambda x : x.lower()):
        index += f'- :ref:`{board}<{reference_for_board(board)}>`\n\n'

    all_features_rows = []
    for feature in sorted(build_options_by_define.values(), key=lambda x : (x.category + x.label).lower()):
        all_features_rows.append([feature.category, feature.label, feature.description])
    all_features = rst_table.tablify(all_features_rows, headings=["Category", "Feature", "Description"])

    return f'''
.. _binary-features:

=====================================
List of Firmware Limitations by Board
=====================================

**Dynamically generated by update.py.  Do not edit.**

{vehicletype} Omitted features by board type in "latest" builds from build server


Board Index
===========

{index}

.. _all-features:

All Features
============

{all_features}

Boards
======

{all_tables}
'''

#######################################################################


class WikiUpdater:
    def __init__(self) -> None:
        if platform.system() == "Windows":
            multiprocessing.freeze_support()

        # Set up option parsing to get connection string
        parser = argparse.ArgumentParser(
            description="Copy Common Files as needed, stripping out non-relevant wiki content",
        )
        parser.add_argument(
            "--site",
            help="If you just want to copy to one site, you can do this. Otherwise will be copied.",
        )
        parser.add_argument(
            "--clean-common",
            action="store_true",
            help="Force clean and copy common files into wikis directories.",
        )
        parser.add_argument(
            "--cached-parameter-files",
            action="store_true",
            help="Do not re-download parameter files",
        )
        parser.add_argument(
            "--parallel",
            type=int,
            help="limit parallel builds, -1 for unlimited",
            default=1,
        )
        parser.add_argument(
            "--destdir",
            default=None,
            help="Destination directory for compiled docs",
        )
        parser.add_argument(
            "--enablebackups",
            action="store_true",
            default=False,
            help="Enable several backups up to const N_BACKUPS_RETAIN in --backupdestdir folder",
        )
        parser.add_argument(
            "--backupdestdir",
            default="/var/sites/wiki-backup/web",
            help="Destination directory for compiled docs",
        )
        parser.add_argument(
            "--paramversioning",
            action="store_true",
            default=False,
            help="Build multiple parameters pages for each vehicle based on its firmware repo.",
        )
        parser.add_argument(
            "--verbose",
            dest="verbose",
            action="store_true",
            default=False,
            help="show debugging output",
        )
        parser.add_argument(
            "--fast",
            dest="fast",
            action="store_true",
            default=False,
            help=("Incremental build using already downloaded parameters, "
                  "log messages, and video thumbnails rather than cleaning "
                  "before build."),
        )

        self.args = parser.parse_args()
        self.verbose: bool = self.args.verbose

        logging_level = logging.DEBUG if self.verbose else logging.INFO
        logger.setLevel(logging_level)
        logging.getLogger('scripts.dedupe_params').setLevel(logging_level)
        stream_handler.setLevel(logging_level)

    def run(self) -> None:

        tstart = time.time()
        now = datetime.now()
        building_time = now.strftime("%Y-%m-%d-%H-%M-%S")

        check_imports()
        check_ref_directives()

        info("=== Step 1: Creating features pages ===")
        info(f"Time elapsed so far: {time.time() - tstart:.2f} seconds")
        create_features_pages(self.args.site)

        info("=== Step 2: Fetching parameters and log messages in parallel ===")
        info(f"Time elapsed so far: {time.time() - tstart:.2f} seconds")
        if not self.args.fast:
            if self.args.paramversioning:
                # Parameters for all versions available on firmware.ardupilot.org:
                fetch_versioned_parameters(self.args.site)
            else:
                # Single parameters file. Just present the latest parameters:
                cleanup_versioned_parameters(self.args.site)
                fetchparameters(self.args.site, self.args.cached_parameter_files)

            dedupe_periph_net_parameters('./dev/source/docs/AP_Periph-Parameters.rst')
            # Fetch most recent LogMessage metadata from autotest:
            fetchlogmessages(self.args.site, self.args.cached_parameter_files)

        info("=== Step 3: Processing static sites ===")
        info(f"Time elapsed so far: {time.time() - tstart:.2f} seconds")
        copy_static_html_sites(self.args.site, self.args.destdir)

        # Use clean_common=True for clean builds, False for fast/incremental builds
        info("=== Step 4: Copying common source files ===")
        info(f"Time elapsed so far: {time.time() - tstart:.2f} seconds")
        copy_common_source_files(clean_common=self.args.clean_common)

        info("=== Step 5: Building documentation with Sphinx ===")
        info(f"Time elapsed so far: {time.time() - tstart:.2f} seconds")
        sphinx_make(self.args.site, self.args.parallel, self.args.fast)

        if self.args.paramversioning:
            put_cached_parameters_files_in_sites(self.args.site)
            cache_parameters_files(self.args.site)

        check_build(self.args.site)

        if self.args.enablebackups:
            make_backup(building_time, self.args.site, self.args.destdir, self.args.backupdestdir)
            delete_old_wiki_backups(self.args.backupdestdir, N_BACKUPS_RETAIN)

        if self.args.destdir:
            copy_build(self.args.site, self.args.destdir)

        # To navigate locally and view versioning script for parameters
        # working is necessary run Chrome as "chrome
        # --allow-file-access-from-files". Otherwise it will appear empty
        # locally and working once is on the server.

        error_count = len(error_store_handler.error_messages)
        total_time = time.time() - tstart
        info(f"Total execution time: {total_time:.2f} seconds ({total_time / 60:.1f} minutes)")

        if error_count > 0:
            # cannot use logger here to not infinitely recurse on error.
            print("[update.py][\033[1;31merror\033[0m]: Reprinting error messages:", file=sys.stderr)
            for error_msg in error_store_handler.error_messages:
                print(f"[update.py][\033[1;31merror\033[0m]: {error_msg}", file=sys.stderr)
        else:
            logger.info("Build completed without errors")

        sys.exit(0)


def main():
    updater = WikiUpdater()
    updater.run()


if __name__ == "__main__":
    main()

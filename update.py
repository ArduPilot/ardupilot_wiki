#!/usr/bin/env python3
"""

This script updates and rebuilds wiki sources from Github and from
parameters on the test server.

It is intended to be run on the main wiki server or
locally within the project's Vagrant environment.

Build notes:

* First step is always a fetch and pull from git (master).
  * Default is just a normal fetch and pull from master
  * If the --clean option is "True" then git will reset to head

* Common topics are copied from /common/source/docs.
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
import io
import json
import glob
import gzip
import hashlib
import multiprocessing
import os
import platform
import re
import shutil
import subprocess
import sys
import time
import requests
from urllib.parse import urlparse
from concurrent.futures import ThreadPoolExecutor
from typing import Optional, Dict, List, Union


from sphinx.application import Sphinx
import rst_table

from codecs import open
from datetime import datetime

from frontend.scripts import get_discourse_posts
import logging


class ErrorStoreHandler(logging.Handler):
    """Allow to store errors for later usage."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.error_messages = []

    def emit(self, record):
        self.error_messages.append(record.getMessage())


logger = logging.getLogger(__name__)

# The ErrorStoreHandler stores the messages
error_store_handler = ErrorStoreHandler()
error_store_handler.setLevel(logging.ERROR)
logger.addHandler(error_store_handler)

# The StreamHandler logs to the console
stream_handler = logging.StreamHandler(sys.stdout)
stream_handler.setLevel(logging.ERROR)
logger.addHandler(stream_handler)

# Global HTTP session for connection reuse and caching
_http_session = None
_cache_dir = ".cache"

if sys.version_info < (3, 8):
    print("Minimum python version is 3.8")
    sys.exit(1)

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

# GIT_REPO = ''

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


def info(str_to_print: str) -> None:
    """Show and count the errors."""
    logging.info(str_to_print)


def debug(str_to_print: str) -> None:
    """Debug output if verbose is set."""
    logging.debug(str_to_print)


def error(str_to_print: Union[str, Exception]) -> None:
    """Show and count the errors."""
    logging.error(f"{str_to_print}")


def fatal(str_to_print: Union[str, Exception]) -> None:
    """Show and count the errors."""
    logging.critical(f"{str_to_print}")
    sys.exit(1)


def remove_if_exists(filepath: str) -> None:
    try:
        os.remove(filepath)
    except OSError as e:
        if e.errno != errno.ENOENT:
            raise e


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


def get_cached_url_content(url, cache_dir=None, max_age_hours=1):
    """Get URL content with caching to avoid repeated downloads"""
    if cache_dir is None:
        cache_dir = _cache_dir

    os.makedirs(cache_dir, exist_ok=True)

    # Create cache filename from URL
    cache_filename = re.sub(r'[^\w\-_.]', '_', url) + '.cache'
    cache_path = os.path.join(cache_dir, cache_filename)

    # Check if cached version exists and is recent
    if os.path.exists(cache_path):
        cache_age = time.time() - os.path.getmtime(cache_path)
        if cache_age < (max_age_hours * 3600):
            debug(f"Using cached content for {url}")
            with open(cache_path, 'rb') as f:
                return f.read()

    # Fetch fresh content
    try:
        debug(f"Fetching fresh content from {url}")
        session = get_http_session()
        response = session.get(url, timeout=30)
        response.raise_for_status()
        content = response.content

        # Cache the content
        with open(cache_path, 'wb') as f:
            f.write(content)

        return content
    except Exception as e:
        error(f"Failed to fetch {url}: {e}")
        # Try to use old cached version if available
        if os.path.exists(cache_path):
            debug(f"Using stale cached content for {url}")
            with open(cache_path, 'rb') as f:
                return f.read()
        raise


def run_command_with_timeout(cmd, cwd=None, timeout=300, check=True):
    """Run command with better error handling and timeout"""
    debug(f"Running command: {cmd}")
    try:
        if isinstance(cmd, str):
            cmd = cmd.split()

        result = subprocess.run(
            cmd,
            cwd=cwd,
            capture_output=True,
            text=True,
            check=check,
            timeout=timeout
        )

        if result.stderr:
            debug(f"Command stderr: {result.stderr}")

        return result.stdout
    except subprocess.CalledProcessError as e:
        error(f"Command failed: {' '.join(cmd)}")
        error(f"Error: {e.stderr}")
        if check:
            raise
    except subprocess.TimeoutExpired:
        error(f"Command timed out: {' '.join(cmd)}")
        if check:
            raise


def fetch_and_rename(fetchurl: str, target_file: str, new_name: str) -> None:
    fetch_url(fetchurl, fpath=new_name, verbose=False)
    info(f"Renaming {new_name} to {target_file}")
    os.replace(new_name, target_file)


def fetch_url(fetchurl: str, fpath: Optional[str] = None, verbose: bool = True) -> None:
    """Fetches content at url and puts it in a file corresponding to the filename in the URL"""
    info(f"Fetching {fetchurl}")

    filename = fpath or os.path.basename(urlparse(fetchurl).path)

    # Try to get from cache first for non-streaming downloads
    if not verbose:  # Non-verbose usually means smaller files, use cache
        try:
            content = get_cached_url_content(fetchurl)
            with open(filename, 'wb') as out_file:
                out_file.write(content)
            return
        except Exception:
            # Fall back to streaming download
            pass

    # For larger files or when cache fails, use streaming download with progress
    session = get_http_session()
    total_size = 0

    if verbose:
        # Get file size for progress bar
        total_size = get_request_file_size(fetchurl)

    response = session.get(fetchurl, stream=True, timeout=30)
    response.raise_for_status()

    downloaded_size = 0
    chunk_size = 64 * 1024  # Increased chunk size for better performance

    with open(filename, 'wb') as out_file:
        if verbose and total_size > 0:
            info("Completed : 0%")
        completed_last = 0
        for chunk in response.iter_content(chunk_size=chunk_size):
            out_file.write(chunk)
            downloaded_size += len(chunk)

            # progress bar
            if verbose and total_size > 0:
                completed = downloaded_size * 100 // total_size
                if completed - completed_last > 10 or completed == 100:
                    info(f"..{completed}%")
                    completed_last = completed
        if verbose:
            info("")  # Newline to correct the console cursor position


def get_request_file_size(url: str) -> int:
    """Get file size from URL using HEAD request with session reuse"""
    try:
        session = get_http_session()
        headers = {'Accept-Encoding': 'identity'}  # needed as request use compression by default
        hresponse = session.head(url, headers=headers, timeout=30)

        if 'Content-Length' in hresponse.headers:
            size = int(hresponse.headers['Content-Length'])
            return size
    except Exception as e:
        debug(f"Failed to get file size for {url}: {e}")
    return 0


def fetchparameters(site: Optional[str] = None, cache: Optional[str] = None) -> None:
    dataname = "Parameters"
    fetch_ardupilot_generated_data(PARAMETER_SITE, f'https://autotest.ardupilot.org/{dataname}', f'{dataname}.rst',
                                   f'{dataname.lower()}.rst', site, cache)


def fetchlogmessages(site: Optional[str] = None, cache: Optional[str] = None) -> None:
    dataname = "LogMessages"
    fetch_ardupilot_generated_data(LOGMESSAGE_SITE, f'https://autotest.ardupilot.org/{dataname}', f'{dataname}.rst',
                                   f'{dataname.lower()}.rst', site, cache)


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
        futures = []
        for url, target, name in zip(urls, targetfiles, names):
            future = executor.submit(fetch_and_rename, url, target, name)
            futures.append(future)

        # Wait for all downloads to complete
        for future in futures:
            try:
                future.result(timeout=5*60)
            except Exception as e:
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

    app = Sphinx(
        buildername='html',
        confdir=source_dir,
        doctreedir=doctree_dir,
        outdir=html_dir,
        parallel=2,
        srcdir=source_dir,
    )
    app.build()


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
        p = multiprocessing.Process(target=build_one, args=(wiki, fast))
        p.start()
        procs.append(p)
        while parallel != -1 and len(procs) >= parallel:
            for p in procs:
                if p.exitcode is not None:
                    p.join()
                    procs.remove(p)
                    if p.exitcode != 0:
                        error('Error making sphinx(1)')
            time.sleep(0.1)
    while len(procs) > 0:
        for p in procs[:]:
            if p.exitcode is not None:
                p.join()
                procs.remove(p)
                if p.exitcode != 0:
                    error('Error making sphinx(2)')
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


def copy_build(site, destdir) -> None:
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
        debug('Copy: %s' % wiki)
        targetdir = os.path.join(destdir, wiki)
        debug("Creating backup")
        olddir = os.path.join(destdir, 'old')
        debug('Recreating %s' % olddir)
        if os.path.exists(olddir):
            shutil.rmtree(olddir)
        os.makedirs(olddir)
        if os.path.exists(targetdir):
            debug(f'Moving {targetdir} into {olddir}')
            shutil.move(targetdir, olddir)
        # copy new dir to targetdir
        # progress("DEBUG: targetdir: %s" % targetdir)
        # sourcedir='./%s/build/html/*' % wiki
        sourcedir = './%s/build/html/' % wiki
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
        debug('Removing %s' % olddir)
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
        debug('Backing up: %s' % wiki)

        targetdir = os.path.join(destdir, wiki)
        os.makedirs(targetdir, exist_ok=True)  # Replace distutils.dir_util.mkpath

        if not os.path.exists(targetdir):
            fatal("FAIL backup when looking for folder %s" % targetdir)

        bkdir = os.path.join(backupdestdir, str(building_time + '-wiki-bkp'), str(wiki))
        debug('Checking %s' % bkdir)
        os.makedirs(bkdir, exist_ok=True)  # Replace distutils.dir_util.mkpath
        debug(f'Copying {targetdir} into {bkdir}')
        try:
            run_command_with_timeout(["rsync", "-a", "--delete", targetdir + "/", bkdir], timeout=600)
        except subprocess.CalledProcessError as ex:
            error(ex)
            fatal("Failed to backup %s" % wiki)


def delete_old_wiki_backups(folder, n_to_keep):
    try:
        debug('Checking number of backups in folder %s' % folder)
        backup_folders = glob.glob(folder + "/*-wiki-bkp/")
        backup_folders.sort()
        if len(backup_folders) > n_to_keep:
            for i in range(0, len(backup_folders) - n_to_keep):
                if '-wiki-bkp' in str(backup_folders[i]):
                    debug('Deleting folder %s' % str(backup_folders[i]))
                    shutil.rmtree(str(backup_folders[i]))
                else:
                    debug('Ignoring folder %s because it does not look like a auto generated wiki backup folder' %
                          str(backup_folders[i]))
        else:
            debug('No old backups to delete in %s' % folder)
    except Exception as e:
        error('Error on deleting some previous wiki backup folders: %s' % e)


def create_dir_if_not_exists(dir_path: str) -> None:
    try:
        os.mkdir(dir_path)
    except FileExistsError:  # Catching specific exception
        pass


def copy_common_source_files(start_dir=COMMON_DIR):
    """
    copies files common to all Wikis to the source directories for each Wiki
    """

    # Clean existing common topics (easiest way to guarantee old ones
    # are removed)
    # Cost is that these will have to be rebuilt even if not changed
    import glob
    for wiki in ALL_WIKIS:
        files = glob.glob('%s/source/docs/common-*.rst' % wiki)
        for f in files:
            debug('Remove existing common: %s' % f)
            os.remove(f)

    # Create destination folders that might be needed (if don't exist)
    for wiki in ALL_WIKIS:
        create_dir_if_not_exists(wiki)
        create_dir_if_not_exists(f'{wiki}/source')
        create_dir_if_not_exists(f'{wiki}/source/docs')
        create_dir_if_not_exists(f'{wiki}/source/_static')

    debug("Copying common source files to each Wiki")
    for root, dirs, files in os.walk(start_dir):
        for file in files:
            if file.endswith(".rst"):
                debug("  FILE: %s" % file)
                source_file_path = os.path.join(root, file)
                source_file = open(source_file_path, 'r', 'utf-8')
                source_content = source_file.read()
                source_file.close()
                targets = get_copy_targets(source_content)
                # progress(targets)
                for wiki in targets:
                    # progress("CopyTarget: %s" % wiki)
                    content = strip_content(source_content, wiki)
                    targetfile = f'{wiki}/source/docs/{file}'
                    debug(targetfile)
                    destination_file = open(targetfile, 'w', 'utf-8')
                    destination_file.write(content)
                    destination_file.close()
            elif file.endswith(".css"):
                for wiki in ALL_WIKIS:
                    shutil.copy2(os.path.join(root, file),
                                 '%s/source/_static/' % wiki)
            elif file.endswith(".js"):
                source_file_path = os.path.join(root, file)
                source_file = open(source_file_path, 'r', 'utf-8')
                source_content = source_file.read()
                source_file.close()
                targets = get_copy_targets(source_content)
                # progress("JS: " + str(targets))
                for wiki in targets:
                    content = strip_content(source_content, wiki)
                    targetfile = f'{wiki}/source/_static/{file}'
                    debug(targetfile)
                    destination_file = open(targetfile, 'w', 'utf-8')
                    destination_file.write(content)
                    destination_file.close()


def get_copy_targets(content):
    p = re.compile(r'\[copywiki.*?destination\=\"(.*?)\".*?\]', flags=re.S)
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
                     flags=re.M)

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
                     flags=re.S)

    return newText


def logmatch_code(matchobj, prefix):

    for i in range(9):
        try:
            info("%s m%d: %s" % (prefix, i, matchobj.group(i)))
        except IndexError:  # The object has less groups than expected
            error("%s: except m%d" % (prefix, i))


def is_the_same_file(file1, file2):
    """ Compare two files using their SHA256 hashes"""
    digests = []
    for filename in [file1, file2]:
        hasher = hashlib.sha256()
        with open(filename, 'rb') as f:  # Open in binary mode
            buf = f.read()
            hasher.update(buf)
            a = hasher.hexdigest()
            digests.append(a)

    return digests[0] == digests[1]


def fetch_versioned_parameters(site=None):
    """
    It relies on "build_parameters.py" be executed before the "update.py"

    Once the generated files are on ../new_params_mversion it tut all
    parameters and JSON files in their destinations.
    """

    for key, value in PARAMETER_SITE.items():

        if key == 'AP_Periph': # workaround until create a versioning for AP_Periph in firmware server
            fetchurl = 'https://autotest.ardupilot.org/Parameters/%s/Parameters.rst' % value
            targetfile = './dev/source/docs/AP_Periph-Parameters.rst'
            fetch_and_rename(fetchurl, targetfile, 'Parameters.rst')

        else: # regular versining

            if site == key or site is None:
                # Remove old param single file
                single_param_file = './%s/source/docs/parameters.rst' % key
                debug("Erasing " + single_param_file)
                remove_if_exists(single_param_file)

                # Remove old versioned param files
                if 'antennatracker' in key.lower():  # To main the original script approach instead of the build_parameters.py approach.  # noqa: E501
                    old_parameters_mask = (os.getcwd() +
                                           '/%s/source/docs/parameters-%s-' %
                                           ("AntennaTracker", "AntennaTracker"))
                else:
                    old_parameters_mask = (os.getcwd() +
                                           '/%s/source/docs/parameters-%s-' %
                                           (key, key.title()))
                try:
                    old_parameters_files = [
                        f for f in glob.glob(old_parameters_mask + "*.rst")]
                    for filename in old_parameters_files:
                        debug("Erasing rst " + filename)
                        os.remove(filename)
                except Exception as e:
                    error(e)
                    pass

                # Remove old json file
                if 'antennatracker' in key.lower():  # To main the original script approach instead of the build_parameters.py approach.  # noqa: E501
                    target_json_file = ('./%s/source/_static/parameters-%s.json' %
                                        ("AntennaTracker", "AntennaTracker"))
                else:
                    target_json_file = ('./%s/source/_static/parameters-%s.json' %
                                        (value, key.title()))
                debug("Erasing json " + target_json_file)
                remove_if_exists(target_json_file)

                # Moves the updated JSON file
                if 'antennatracker' in key.lower():  # To main the original script approach instead of the build_parameters.py approach.  # noqa: E501
                    vehicle_json_file = os.getcwd() + '/../new_params_mversion/{}/parameters-{}.json'.format("AntennaTracker", "AntennaTracker")  # noqa: E501
                else:
                    vehicle_json_file = os.getcwd() + f'/../new_params_mversion/{value}/parameters-{key.title()}.json'
                new_file = (
                    key +
                    "/source/_static/" +
                    vehicle_json_file[str(vehicle_json_file).rfind("/")+1:])
                try:
                    debug("Moving " + vehicle_json_file)
                    # os.rename(vehicle_json_file, new_file)
                    shutil.copy2(vehicle_json_file, new_file)
                except Exception as e:
                    error(e)
                    pass

                # Copy all parameter files to vehicle folder IFF it is new
                new_parameters_files = []
                try:
                    new_parameters_folder = (os.getcwd() +
                                             '/../new_params_mversion/%s/' % value)
                    new_parameters_files = [
                        f for f in glob.glob(new_parameters_folder + "*.rst")
                    ]
                except Exception as e:
                    error(e)
                    pass
                for filename in new_parameters_files:
                    # Check possible cached version
                    try:
                        new_file = (key +
                                    "/source/docs/" +
                                    filename[str(filename).rfind("/")+1:])
                        if not os.path.isfile(new_file):
                            debug(f"Copying {filename} to {new_file} (target file does not exist)")
                            shutil.copy2(filename, new_file)
                        elif os.path.isfile(filename.replace("new_params_mversion", "old_params_mversion")): # The cached file exists?  # noqa: E501

                            # Temporary debug messages to help with cache tasks.
                            debug("Check cache: {} against {}".format(filename, filename.replace("new_params_mversion", "old_params_mversion")))  # noqa: E501
                            debug("Check cache with filecmp.cmp: %s" % filecmp.cmp(filename, filename.replace("new_params_mversion", "old_params_mversion")))  # noqa: E501
                            debug("Check cache with sha256: %s" % is_the_same_file(filename, filename.replace("new_params_mversion", "old_params_mversion")))  # noqa: E501

                            if ("parameters.rst" in filename) or (not filecmp.cmp(filename, filename.replace("new_params_mversion", "old_params_mversion"))):    # It is different?  OR is this one the latest. | Latest file must be built everytime in order to enable Sphinx create the correct references across the wiki.  # noqa: E501
                                debug(f"Overwriting {filename} to {new_file}")
                                shutil.copy2(filename, new_file)
                            else:
                                debug("It will reuse the last build of " + new_file)
                        else:   # If not cached, copy it anyway.
                            debug(f"Copying {filename} to {new_file}")
                            shutil.copy2(filename, new_file)

                    except Exception as e:
                        error(e)
                        pass


def create_latest_parameter_redirect(default_param_file, vehicle):
    """
    For a given vehicle create a file called parameters.rst that
    redirects to the latest parameters file.(Create to maintaim retro
    compatibility.)
    """
    out_line = "======================\nParameters List (Full)(\n======================\n"
    out_line += "\n.. raw:: html\n\n"
    out_line += "   <script>location.replace(\"" + default_param_file[:-3] + "html" + "\")</script>"
    out_line += "\n\n"

    filename = vehicle + "/source/docs/parameters.rst"
    with open(filename, "w") as text_file:
        text_file.write(out_line)

    debug("Created html automatic redirection from parameters.html to %shtml" %
          default_param_file[:-3])


def cache_parameters_files(site=None):
    """
    For each vechile: put new_params_mversion/ content in
    old_params_mversion/ folders and .html built files as well.
    """
    for key, value in PARAMETER_SITE.items():
        if (site == key or site is None) and (key != 'AP_Periph'):  # and (key != 'AP_Periph') workaround until create a versioning for AP_Periph in firmware server # noqa: E501
            try:
                old_parameters_folder = (os.getcwd() +
                                         '/../old_params_mversion/%s/' % value)
                old_parameters_files = [
                    f for f in glob.glob(old_parameters_folder + "*.*")
                ]
                for file in old_parameters_files:
                    debug("Removing %s" % file)
                    os.remove(file)

                new_parameters_folder = (os.getcwd() +
                                         '/../new_params_mversion/%s/' % value)
                new_parameters_files = [
                    f for f in glob.glob(new_parameters_folder +
                                         "parameters-*.rst")
                ]
                for filename in new_parameters_files:
                    debug("Copying %s to %s" %
                          (filename, old_parameters_folder))
                    shutil.copy2(filename, old_parameters_folder)

                built_folder = os.getcwd() + "/" + key + "/build/html/docs/"
                built_parameters_files = [
                    f for f in glob.glob(built_folder + "parameters-*.html")
                ]
                for built in built_parameters_files:
                    debug("Copying %s to %s" %
                          (built, old_parameters_folder))
                    shutil.copy2(built, old_parameters_folder)

            except Exception as e:
                error(e)
                pass


def put_cached_parameters_files_in_sites(site=None):
    """
    For each vehicle: put built .html files in site folder

    """
    for key, value in PARAMETER_SITE.items():
        if (site == key or site is None) and (key != 'AP_Periph'): # and (key != 'AP_Periph') workaround until create a versioning for AP_Periph in firmware server # noqa: E501
            try:
                built_folder = (os.getcwd() +
                                '/../old_params_mversion/%s/' % value)
                built_parameters_files = [
                    f for f in glob.glob(built_folder + "parameters-*.html")
                ]
                vehicle_folder = os.getcwd() + "/" + key + "/build/html/docs/"
                debug("Site %s getting previously built files from %s" %
                      (site, built_folder))
                for built in built_parameters_files:
                    if "latest" not in built:  # latest parameters files must be built every time
                        debug("Reusing built %s in %s " %
                              (built, vehicle_folder))
                        shutil.copy(built, vehicle_folder)
            except Exception as e:
                error(str(e))
                pass


def update_frontend_json():
    """
    Frontend get posts from Forum server and insert it into JSON
    """
    debug('Running script to get last posts from forum server.')
    try:
        get_discourse_posts.main()
    except Exception as e:
        error(str(e))
        pass


def copy_static_html_sites(site, destdir):
    """
    Copy pure HMTL folder the same way that Sphinx builds it
    """
    if (site in ['frontend', None]) and (destdir is not None):
        debug('Copying static sites (only frontend so far).')
        update_frontend_json()
        folder = 'frontend'
        try:
            site_folder = os.getcwd() + "/" + folder
            targetdir = os.path.join(destdir, folder)
            shutil.rmtree(targetdir, ignore_errors=True)
            shutil.copytree(site_folder, targetdir)
        except Exception as e:
            error(str(e))
            pass


def check_imports():
    """check key imports work"""
    import importlib.metadata
    # package names to check the versions of. Note that these can be different than the string used to import the package
    required_packages = ["sphinx_rtd_theme>=1.3.0", "sphinxcontrib.youtube>=1.2.0", "sphinx>=7.1.2", "docutils<0.19"]
    for package in required_packages:
        debug("Checking for %s" % package)
        try:
            importlib.metadata.version(package.split("<")[0].split(">=")[0])
        except importlib.metadata.PackageNotFoundError as ex:
            error(ex)
            fatal("Require %s\nPlease run the wiki build setup script \"Sphinxsetup\"" % package)
    debug("Imports OK")


def check_ref_directives():
    """check formatting around ref directive that sphinx does not warn about"""
    character_before_ref_tag = re.compile(r"[a-zA-Z0-9_:]:ref:")
    character_after_ref_tag = re.compile(r"(:ref:`.*?`[_]{0,2}) ([\.,:])")

    # don't check "common="" files in vehicle wikis
    skipped_files = set()
    for wiki in ALL_WIKIS:
        skipped_files.update(glob.glob(f'{wiki}/source/docs/common-*.rst'))
    wiki_glob = set(glob.glob("**/*.rst", recursive=True))
    files_to_check = wiki_glob.difference(skipped_files)
    for f in files_to_check:
        with open(f, "r", "utf-8") as file:
            for i, line in enumerate(file.readlines()):
                if character_before_ref_tag.search(line):
                    error(f"Remove character before ref directive in \"{f}\" on line number {i+1}")
                if character_after_ref_tag.search(line):
                    error(f"Remove character after ref directive in \"{f}\" on line number {i+1}")


def load_build_options():
    """Load build options with caching"""
    try:
        # Use cached download for build_options.py
        content = get_cached_url_content(
            "https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/scripts/build_options.py",
            max_age_hours=6  # Cache for 6 hours
        )
        # Import module from content without writing temporary file
        import importlib.util

        # Create module from source code
        spec = importlib.util.spec_from_loader("build_options", loader=None)
        if spec is None:
            raise ImportError("Failed to create module spec")
        build_options = importlib.util.module_from_spec(spec)

        # Decode content if it's bytes
        source_code = content.decode('utf-8') if isinstance(content, bytes) else content
        exec(source_code, build_options.__dict__)

        build_options_by_define = {}
        for f in build_options.BUILD_OPTIONS:
            build_options_by_define[f.define] = f
        return build_options_by_define
    except Exception as e:
        error(f"Failed to load build options: {e}")
        return {}


def load_features_data():
    """Load features data with caching"""
    try:
        # Use cached download for features.json.gz
        content = get_cached_url_content(
            "https://firmware.ardupilot.org/features.json.gz",
            max_age_hours=6  # Cache for 6 hours
        )

        # Process in-memory without writing to filesystem
        with gzip.open(io.BytesIO(content), 'rt') as f:
            features_json = json.load(f)

        if features_json["format-version"] != "1.0.0":
            error("bad format version")
            return []
        return features_json["features"]
    except Exception as e:
        error(f"Failed to load features data: {e}")
        return []


def process_platform_features(platform_features, build_options_by_define, platform_key, vehicletype):
    """Process features for a single platform"""
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
            error("feature %s (%s,%s) not in build_options.py" %
                  (feature, platform_key, vehicletype))
            continue

        if feature_in:
            sorted_platform_features_in.append((build_options.category, feature))
        else:
            sorted_platform_features_not_in.append((build_options.category, feature))

    # Sort and combine features
    sorted_platform_features = (
        sorted(sorted_platform_features_not_in, key=lambda x: x[0] + x[1]) +
        sorted(sorted_platform_features_in, key=lambda x: x[0] + x[1])
    )

    return sorted_platform_features, features_in


def create_platform_table(sorted_platform_features, features_in, build_options_by_define, platform_key):
    """Create table for a single platform"""
    rows = []
    column_headings = ["Category", "Feature", "Included", "Description"]

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
        return ""

    table = rst_table.tablify(rows, headings=column_headings)
    underline = "-" * len(platform_key)

    return """
.. _{}:

{}
{}

{}
""".format(reference_for_board(platform_key), platform_key, underline, table)


def create_features_pages(site):
    """for each vehicle, write out a page containing features for each
    supported board"""

    debug("Creating features pages")

    # Load build options and features data with caching
    build_options_by_define = load_build_options()
    if not build_options_by_define:
        error("Failed to load build options, skipping features pages")
        return

    features = load_features_data()
    if not features:
        error("Failed to load features data, skipping features pages")
        return

    # Process each vehicle
    for wiki in WIKI_NAME_TO_VEHICLE_NAME.keys():
        debug(f"Processing features for {wiki}")
        if site is not None and site != wiki:
            continue
        if wiki not in WIKI_NAME_TO_VEHICLE_NAME:
            continue

        vehicletype = WIKI_NAME_TO_VEHICLE_NAME[wiki]
        content = create_features_page(features, build_options_by_define, vehicletype)

        if wiki == "AP_Periph":
            destination_filepath = "dev/source/docs/periph-binary-features.rst"
        else:
            destination_filepath = "%s/source/docs/binary-features.rst" % wiki

        # make .../docs/ directory if it doesn't already exist
        os.makedirs(os.path.dirname(destination_filepath), exist_ok=True)
        with open(destination_filepath, "w") as f:
            f.write(content)


def reference_for_board(board):
    """return a string suitable for creating an anchor in RST to make
    board's feture table linkable"""
    return "FEATURE_%s" % board


def create_features_page(features, build_options_by_define, vehicletype):
    """Create features page content for a vehicle type"""
    # Group features by platform
    features_by_platform = {}
    for build in features:
        if build["vehicletype"] != vehicletype:
            continue
        features_by_platform[build["platform"]] = build["features"]

    # Generate tables for each platform
    all_tables = ""
    for platform_key in sorted(features_by_platform.keys(), key=str.lower):
        platform_features = features_by_platform[platform_key]
        sorted_platform_features, features_in = process_platform_features(
            platform_features, build_options_by_define, platform_key, vehicletype
        )
        platform_table = create_platform_table(
            sorted_platform_features, features_in, build_options_by_define, platform_key
        )
        all_tables += platform_table

    # Generate board index
    index = ""
    for board in sorted(features_by_platform.keys(), key=str.lower):
        index += f'- :ref:`{board}<{reference_for_board(board)}>`\n\n'

    # Generate all features table
    all_features_rows = []
    for feature in sorted(build_options_by_define.values(),
                          key=lambda x: (x.category + x.label).lower()):
        all_features_rows.append([feature.category, feature.label, feature.description])
    all_features = rst_table.tablify(all_features_rows,
                                     headings=["Category", "Feature", "Description"])

    return f"""
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
"""

#######################################################################


class WikiUpdater:
    def __init__(self) -> None:
        if platform.system() == "Windows":
            multiprocessing.freeze_support()

        # Set up option parsing to get connection string
        parser = argparse.ArgumentParser(
            description='Copy Common Files as needed, stripping out non-relevant wiki content',
        )
        parser.add_argument(
            '--site',
            help="If you just want to copy to one site, you can do this. Otherwise will be copied.",
        )
        parser.add_argument(
            '--clean',
            action='store_true',
            help="Does a very clean build - resets git to master head (and TBD cleans up any duplicates in the output).",
        )
        parser.add_argument(
            '--cached-parameter-files',
            action='store_true',
            help="Do not re-download parameter files",
        )
        parser.add_argument(
            '--parallel',
            type=int,
            help="limit parallel builds, -1 for unlimited",
            default=1,
        )
        parser.add_argument(
            '--destdir',
            default=None,
            help="Destination directory for compiled docs",
        )
        parser.add_argument(
            '--enablebackups',
            action='store_true',
            default=False,
            help="Enable several backups up to const N_BACKUPS_RETAIN in --backupdestdir folder",
        )
        parser.add_argument(
            '--backupdestdir',
            default="/var/sites/wiki-backup/web",
            help="Destination directory for compiled docs",
        )
        parser.add_argument(
            '--paramversioning',
            action='store_true',
            default=False,
            help="Build multiple parameters pages for each vehicle based on its firmware repo.",
        )
        parser.add_argument(
            '--verbose',
            dest='verbose',
            action='store_true',
            default=False,
            help="show debugging output",
        )
        parser.add_argument(
            '--fast',
            dest='fast',
            action='store_true',
            default=False,
            help=("Incremental build using already downloaded parameters, "
                  "log messages, and video thumbnails rather than cleaning "
                  "before build."),
        )

        # Get the directory where this script is located for cache directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        default_cache_dir = os.path.join(script_dir, '.cache')

        parser.add_argument(
            '--cache-dir',
            dest='cache_dir',
            default=default_cache_dir,
            help="Directory to cache HTTP responses for faster subsequent builds",
        )
        parser.add_argument(
            '--cache-max-age',
            dest='cache_max_age',
            type=int,
            default=1,
            help="Maximum age of cached files in hours (default: 1)",
        )
        self.args, unknown = parser.parse_known_args()
        self.verbose: bool = self.args.verbose

        logging_level = logging.DEBUG if self.verbose else logging.INFO
        logging.basicConfig(level=logging_level, format='[update.py]: [%(levelname)s]: %(message)s')

    def run(self) -> None:
        global _cache_dir
        _cache_dir = self.args.cache_dir
        tstart = time.time()

        # Create cache directory
        os.makedirs(_cache_dir, exist_ok=True)
        info(f"Using cache directory: {_cache_dir}")

        now = datetime.now()
        building_time = now.strftime("%Y-%m-%d-%H-%M-%S")

        check_imports()
        check_ref_directives()

        info("=== Step 1: Creating features pages ===")
        info(f"Time eslapsed so far: {time.time() - tstart:.2f} seconds")
        create_features_pages(self.args.site)

        if not self.args.fast:
            info("=== Step 2: Fetching parameters and log messages in parallel ===")
            info(f"Time eslapsed so far: {time.time() - tstart:.2f} seconds")
            # Run parameters and log messages fetching in parallel for better performance
            with ThreadPoolExecutor(max_workers=2) as executor:
                # Submit both tasks based on paramversioning
                if self.args.paramversioning:
                    info("Using versioned parameters")
                    param_future = executor.submit(fetch_versioned_parameters, self.args.site)
                else:
                    info("Using regular parameters")
                    param_future = executor.submit(fetchparameters, self.args.site, self.args.cached_parameter_files)

                log_future = executor.submit(fetchlogmessages, self.args.site, self.args.cached_parameter_files)

                # Wait for both to complete and handle any errors
                try:
                    param_future.result(timeout=15*60)  # 15 minute timeout for versioned params
                    info("Parameters fetching completed")
                except Exception as e:
                    error(f"Parameters fetching failed: {e}")

                try:
                    log_future.result(timeout=10*60)  # 10 minute timeout for log messages
                    info("Log messages fetching completed")
                except Exception as e:
                    error(f"Log messages fetching failed: {e}")

        info("=== Step 3: Processing static sites ===")
        info(f"Time eslapsed so far: {time.time() - tstart:.2f} seconds")
        copy_static_html_sites(self.args.site, self.args.destdir)

        info("=== Step 4: Copying common source files ===")
        info(f"Time eslapsed so far: {time.time() - tstart:.2f} seconds")
        copy_common_source_files()

        info("=== Step 5: Building documentation with Sphinx ===")
        info(f"Time eslapsed so far: {time.time() - tstart:.2f} seconds")
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
        if error_count > 0:
            error("Reprinting error messages:")
            for error_msg in error_store_handler.error_messages:
                error(f"\033[1;31m[update.py][error]: {error_msg}\033[0m")
            fatal(f"{error_count} errors during Wiki build")
        else:
            info("Build completed without errors")

        info(f"Total execution time: {total_time:.2f} seconds ({total_time/60:.1f} minutes)")
        sys.exit(0)


def main():
    updater = WikiUpdater()
    updater.run()


if __name__ == "__main__":
    main()

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
from __future__ import print_function, unicode_literals

import argparse
import distutils
import errno
import filecmp
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
from typing import Optional, Dict, List


import rst_table

from codecs import open
from datetime import datetime
# while flake8 says this is unused, distutils.dir_util.mkpath fails
# without the following import on old versions of Python:
from distutils import dir_util  # noqa: F401

from frontend.scripts import get_discourse_posts

if sys.version_info < (3, 8):
    print("Minimum python version is 3.8")
    sys.exit(1)

DEFAULT_COPY_WIKIS = ['copter', 'plane', 'rover']
ALL_WIKIS = [
    'copter',
    'plane',
    'rover',
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
    'blimp': 'Blimp',
}

# GIT_REPO = ''

PARAMETER_SITE = {
    'rover': 'APMrover2',
    'copter': 'ArduCopter',
    'plane': 'ArduPlane',
    'antennatracker': 'AntennaTracker',
    'AP_Periph': 'AP_Periph',
    'blimp': 'Blimp',
}
LOGMESSAGE_SITE = {
    'rover': 'Rover',
    'copter': 'Copter',
    'plane': 'Plane',
    'antennatracker': 'Tracker',
    'blimp': 'Blimp',
}
error_log = list()
N_BACKUPS_RETAIN = 10

VERBOSE = False


def debug(str_to_print):
    """Debug output if verbose is set."""
    if VERBOSE:
        print(f"[update.py]: {str_to_print}")


def error(str_to_print):
    """Show and count the errors."""
    global error_log
    error_log.append(str_to_print)
    print(f"[update.py][error]: {str_to_print}", file=sys.stderr)


def fatal(str_to_print):
    """Show and count the errors."""
    error(str_to_print)
    sys.exit(1)


def remove_if_exists(filepath):
    try:
        os.remove(filepath)
    except OSError as e:
        if e.errno != errno.ENOENT:
            raise e


def fetch_and_rename(fetchurl: str, target_file: str, new_name: str) -> None:
    fetch_url(fetchurl, fpath=new_name, verbose=False)
    print(f"Renaming {new_name} to {target_file}")
    os.replace(new_name, target_file)


def fetch_url(fetchurl: str, fpath: Optional[str] = None, verbose: bool = True) -> None:
    """Fetches content at url and puts it in a file corresponding to the filename in the URL"""
    print(f"Fetching {fetchurl}")

    if verbose:
        total_size = get_request_file_size(fetchurl)

    response = requests.get(fetchurl, stream=True)
    response.raise_for_status()

    filename = fpath or os.path.basename(urlparse(fetchurl).path)

    downloaded_size = 0
    chunk_size = 10 * 1024

    with open(filename, 'wb') as out_file:
        if verbose:
            print(f"Completed : 0%", end='')
        completed_last = 0
        for chunk in response.iter_content(chunk_size=chunk_size):
            out_file.write(chunk)
            downloaded_size += len(chunk)

            # progress bar
            if verbose:
                completed = downloaded_size * 100 // total_size
                if completed - completed_last > 10 or completed == 100:
                    print(f"..{completed}%", end='')
                    completed_last = completed
        if verbose:
            print()  # Newline to correct the console cursor position


def get_request_file_size(url: str) -> int:
    headers = {'Accept-Encoding': 'identity'}  # needed as request use compression by default
    hresponse = requests.head(url, headers=headers)

    if 'Content-Length' in hresponse.headers:
        size = int(hresponse.headers['Content-Length'])
        return size
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

    with ThreadPoolExecutor() as executor:
        executor.map(fetch_and_rename, urls, targetfiles, names, timeout=5*60)


def build_one(wiki, fast):
    '''build one wiki'''
    print(f'Using make for sphinx: {wiki}')
    if platform.system() == "Windows":
        # This will fail if there's no folder to clean, so no check_call here
        if not fast:
            subprocess.run(["make.bat", "clean"], cwd=wiki, shell=True)
        subprocess.check_call(["make.bat", "html"], cwd=wiki, shell=True)
    else:
        if not fast:
            subprocess.check_call(["nice", "make", "clean"], cwd=wiki)
        subprocess.check_call(["nice", "make", "html"], cwd=wiki)


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
            fatal("%s site not built - missing %s" % (wiki, index_html))


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
        debug('Copy: %s' % wiki)
        targetdir = os.path.join(destdir, wiki)
        debug("Creating backup")
        olddir = os.path.join(destdir, 'old')
        debug('Recreating %s' % olddir)
        if os.path.exists(olddir):
            shutil.rmtree(olddir)
        os.makedirs(olddir)
        if os.path.exists(targetdir):
            debug('Moving %s into %s' % (targetdir, olddir))
            shutil.move(targetdir, olddir)
        # copy new dir to targetdir
        # print("DEBUG: targetdir: %s" % targetdir)
        # sourcedir='./%s/build/html/*' % wiki
        sourcedir = './%s/build/html/' % wiki
        # print("DEBUG: sourcedir: %s" % sourcedir)
        # print('DEBUG: mv %s %s' % (sourcedir, destdir) )

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


def make_backup(site, destdir, backupdestdir):
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
        distutils.dir_util.mkpath(targetdir)

        if not os.path.exists(targetdir):
            fatal("FAIL backup when looking for folder %s" % targetdir)

        bkdir = os.path.join(backupdestdir, str(building_time + '-wiki-bkp'), str(wiki))
        debug('Checking %s' % bkdir)
        distutils.dir_util.mkpath(bkdir)
        debug('Copying %s into %s' % (targetdir, bkdir))
        try:
            subprocess.check_call(["rsync", "-a", "--delete", targetdir + "/", bkdir])
        except subprocess.CalledProcessError as ex:
            print(ex)
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


def generate_copy_dict(start_dir=COMMON_DIR):
    """
    This creates a dict which indexes copy targets for all common docs.
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

    for root, dirs, files in os.walk(start_dir):
        for file in files:
            if file.endswith(".rst"):
                debug("FILE: %s" % file)
                source_file_path = os.path.join(root, file)
                source_file = open(source_file_path, 'r', 'utf-8')
                source_content = source_file.read()
                source_file.close()
                targets = get_copy_targets(source_content)
                # print(targets)
                for wiki in targets:
                    # print("CopyTarget: %s" % wiki)
                    content = strip_content(source_content, wiki)
                    targetfile = '%s/source/docs/%s' % (wiki, file)
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
                # print("JS: " + str(targets))
                for wiki in targets:
                    content = strip_content(source_content, wiki)
                    targetfile = '%s/source/_static/%s' % (wiki, file)
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
        # print("STRIPPED")
        return ''

    # Remove the copywiki from content
    newText = re.sub(r'\[copywiki.*?\]',
                     fix_copywiki_shortcode,
                     content,
                     flags=re.M)

    def fix_site_shortcode(matchobj):
        # logmatch_code(matchobj, 'SITESC_')
        sitelist = matchobj.group(1)
        # print("SITES_BLOCK: %s" % sitelist)
        if site not in sitelist:
            # print("NOT")
            return ''
        else:
            # print("YES")
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
            print("%s m%d: %s" % (prefix, i, matchobj.group(i)))
        except IndexError:  # The object has less groups than expected
            print("%s: except m%d" % (prefix, i))


def is_the_same_file(file1, file2):
    """ Compare two files using their SHA256 hashes"""
    digests = []
    for filename in [file1, file2]:
        hasher = hashlib.sha256()
        with open(filename, 'rb') as f:
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
                    vehicle_json_file = os.getcwd() + '/../new_params_mversion/%s/parameters-%s.json' % ("AntennaTracker", "AntennaTracker")  # noqa: E501
                else:
                    vehicle_json_file = os.getcwd() + '/../new_params_mversion/%s/parameters-%s.json' % (value, key.title())
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
                            debug("Copying %s to %s (target file does not exist)" % (filename, new_file))
                            shutil.copy2(filename, new_file)
                        elif os.path.isfile(filename.replace("new_params_mversion", "old_params_mversion")): # The cached file exists?  # noqa: E501

                            # Temporary debug messages to help with cache tasks.
                            debug("Check cache: %s against %s" % (filename, filename.replace("new_params_mversion", "old_params_mversion")))  # noqa: E501
                            debug("Check cache with filecmp.cmp: %s" % filecmp.cmp(filename, filename.replace("new_params_mversion", "old_params_mversion")))  # noqa: E501
                            debug("Check cache with sha256: %s" % is_the_same_file(filename, filename.replace("new_params_mversion", "old_params_mversion")))  # noqa: E501

                            if ("parameters.rst" in filename) or (not filecmp.cmp(filename, filename.replace("new_params_mversion", "old_params_mversion"))):    # It is different?  OR is this one the latest. | Latest file must be built everytime in order to enable Sphinx create the correct references across the wiki.  # noqa: E501
                                debug("Overwriting %s to %s" % (filename, new_file))
                                shutil.copy2(filename, new_file)
                            else:
                                debug("It will reuse the last build of " + new_file)
                        else:   # If not cached, copy it anyway.
                            debug("Copying %s to %s" % (filename, new_file))
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
    For each vechile: put built .html files in site folder

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
                    if ("latest" not in built):  # latest parameters files must be built every time
                        debug("Reusing built %s in %s " %
                              (built, vehicle_folder))
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
            error(e)
            pass


def check_imports():
    '''check key imports work'''
    import pkg_resources
    # package names to check the versions of. Note that these can be different than the string used to import the package
    requires = ["sphinx_rtd_theme>=1.3.0", "sphinxcontrib.youtube>=1.2.0", "sphinx>=7.1.2", "docutils<0.19"]
    for r in requires:
        debug("Checking for %s" % r)
        try:
            pkg_resources.require(r)
        except pkg_resources.ResolutionError as ex:
            print(ex)
            fatal("Require %s\nPlease run the wiki build setup script \"Sphinxsetup\"" % r)
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
        with open(f, "r", "utf-8") as file:
            for i, line in enumerate(file.readlines()):
                if character_before_ref_tag.search(line):
                    error(f"Remove character before ref directive in \"{f}\" on line number {i+1}")
                if character_after_ref_tag.search(line):
                    error(f"Remove character after ref directive in \"{f}\" on line number {i+1}")


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
    for f in build_options.BUILD_OPTIONS:
        build_options_by_define[f.define] = f

    # fetch and load most-recently-built features.json
    remove_if_exists("features.json.gz")
    fetch_url("https://firmware.ardupilot.org/features.json.gz")
    features_json = json.load(gzip.open("features.json.gz"))
    if features_json["format-version"] != "1.0.0":
        print("bad format version")
        return
    features = features_json["features"]

    # print("features: (%s)" % str(features))
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
            destination_filepath = "%s/source/docs/binary-features.rst" % wiki
        with open(destination_filepath, "w") as f:
            f.write(content)


def reference_for_board(board):
    '''return a string suitable for creating an anchor in RST to make
    board's feture table linkable'''
    return "FEATURE_%s" % board


def create_features_page(features, build_options_by_define, vehicletype):
    features_by_platform = {}
    for build in features:
        # print("build: (%s)" % str(build))
        if build["vehicletype"] != vehicletype:
            continue
        features_by_platform[build["platform"]] = build["features"]
    rows = []
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
                print("feature %s (%s,%s) not in build_options.py" %
                      (feature, platform_key, vehicletype))
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
        all_tables += ('''
.. _%s:

%s
%s

%s
''' % (reference_for_board(platform_key), platform_key, underline, t))

    index = ""
    for board in sorted(features_by_platform.keys(), key=lambda x : x.lower()):
        index += '- :ref:`%s<%s>`\n\n' % (board, reference_for_board(board))

    all_features_rows = []
    for feature in sorted(build_options_by_define.values(), key=lambda x : (x.category + x.label).lower()):
        all_features_rows.append([feature.category, feature.label, feature.description])
    all_features = rst_table.tablify(all_features_rows, headings=["Category", "Feature", "Description"])

    return '''
.. _binary-features:

=====================================
List of Firmware Limitations by Board
=====================================

**Dynamically generated by update.py.  Do not edit.**

%s Omitted features by board type in "latest" builds from build server


Board Index
===========

%s

.. _all-features:

All Features
============

%s

Boards
======

%s
''' % (vehicletype, index, all_features, all_tables)

#######################################################################


if __name__ == "__main__":

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
        help=("Incremental build using already downloaded parameters, log messages, and video thumbnails rather than cleaning "
              "before build."),
    )

    args = parser.parse_args()
    # print(args.site)
    # print(args.clean)

    VERBOSE = args.verbose

    now = datetime.now()
    building_time = now.strftime("%Y-%m-%d-%H-%M-%S")

    check_imports()
    check_ref_directives()
    create_features_pages(args.site)

    if not args.fast:
        if args.paramversioning:
            # Parameters for all versions availble on firmware.ardupilot.org:
            fetch_versioned_parameters(args.site)
        else:
            # Single parameters file. Just present the latest parameters:
            fetchparameters(args.site, args.cached_parameter_files)

        # Fetch most recent LogMessage metadata from autotest:
        fetchlogmessages(args.site, args.cached_parameter_files)

    copy_static_html_sites(args.site, args.destdir)
    generate_copy_dict()
    sphinx_make(args.site, args.parallel, args.fast)

    if args.paramversioning:
        put_cached_parameters_files_in_sites(args.site)
        cache_parameters_files(args.site)

    check_build(args.site)

    if args.enablebackups:
        make_backup(args.site, args.destdir, args.backupdestdir)
        delete_old_wiki_backups(args.backupdestdir, N_BACKUPS_RETAIN)

    if args.destdir:
        copy_build(args.site, args.destdir)

    # To navigate locally and view versioning script for parameters
    # working is necessary run Chrome as "chrome
    # --allow-file-access-from-files". Otherwise it will appear empty
    # locally and working once is on the server.

    error_count = len(error_log)
    if error_count > 0:
        print("Reprinting error messages:", file=sys.stderr)
        for msg in error_log:
            print(f"\033[1;31m[update.py][error]: {msg}\033[0m", file=sys.stderr)
        fatal(f"{error_count} errors during Wiki build")
    else:
        print("Build completed without errors")

    sys.exit(0)

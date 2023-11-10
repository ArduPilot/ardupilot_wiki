#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    This script aims to provide multiple parameters source files for each vehicle based on the versions available at
    https://firmware.ardupilot.org

    It is intended to be run on the main wiki server. TO-DO: run locally within the project's Vagrant environment.

    Build notes:

    * Before start:
        * Folder and file management tested only in linux;
        * It is supposed to have the wiki repo in a same level of an ArduPilot repo and two other folders
        named new_params_mvesion/ and old_params_mversion/

    * First step is go to each vehicle on firmware.ardupilot.org and get all available versions namely as stable/beta/latest.
        * For each version it gets a board and get git_version.txt file;
        * It parsers that to get the version and commit hash;
        * It creates a dictionary with vehicles, versions and commit hashes.

    * Second step is use the dict to navigates on a ArduPilot Repo, changing checkouts to desidered hashes.
        * Relies on ArduPilotRepoFolder/Tools/autotest/param_metadata/param_parse.py to generate the parameters files;
        * It renames the anchors for all files, except for latest versions.

    * Third step: create the json files and move all generated files.

"""

import os
import shutil
import sys
import time
from html.parser import HTMLParser
import re
import glob
import argparse
import subprocess
import requests
from concurrent.futures import ThreadPoolExecutor


parser = argparse.ArgumentParser(description="python3 build_parameters.py [options]")
parser.add_argument("--verbose", dest='verbose', action='store_false', default=True, help="show debugging output")
parser.add_argument("--ardupilotRepoFolder", dest='gitFolder', default="../ardupilot", help="Ardupilot git folder. ")
parser.add_argument("--destination", dest='destFolder', default="../../../../new_params_mversion", help="Parameters*.rst destination folder.")  # noqa: E501
parser.add_argument('--vehicle', dest='single_vehicle', help="If you just want to copy to one vehicle, you can do this. Otherwise it will work for all vehicles (Copter, Plane, Rover, AntennaTracker, Sub, Blimp)")  # noqa: E501
parser.add_argument('--parallel-vehicles', dest='parallel_vehicles', action='store_true', default=True,
                    help="Enable parallel processing of vehicles (default: enabled)")
parser.add_argument('--no-parallel-vehicles', dest='parallel_vehicles', action='store_false',
                    help="Disable parallel processing of vehicles")

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))
default_cache_dir = os.path.join(script_dir, '.cache')

parser.add_argument("--cache-dir", dest='cache_dir', default=default_cache_dir, help="Directory to cache HTTP responses")
args = parser.parse_args()

error_count = 0

# Global session for HTTP requests with connection pooling
session = requests.Session()
session.headers.update({
    'User-Agent': 'Mozilla/5.0 (compatible; ArduPilotWikiBuilder/1.0)',
    'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8',
    'Connection': 'keep-alive'
})


def get_cached_url(url, cache_dir=None):
    """Get URL content with caching to avoid repeated downloads"""
    if cache_dir is None:
        cache_dir = args.cache_dir

    os.makedirs(cache_dir, exist_ok=True)

    # Create cache filename from URL
    cache_filename = re.sub(r'[^\w\-_.]', '_', url) + '.cache'
    cache_path = os.path.join(cache_dir, cache_filename)

    # Check if cached version exists and is recent (less than 6 hours old)
    if os.path.exists(cache_path):
        cache_age = time.time() - os.path.getmtime(cache_path)
        if cache_age < 6 * 3600:  # 6 hours
            debug(f"Using cached content for {url}")
            with open(cache_path, 'r', encoding='utf-8') as f:
                return f.read()

    # Fetch fresh content
    try:
        debug(f"Fetching fresh content from {url}")
        response = session.get(url, timeout=30)
        response.raise_for_status()
        content = response.text

        # Cache the content
        with open(cache_path, 'w', encoding='utf-8') as f:
            f.write(content)

        return content
    except Exception as e:
        error(f"Failed to fetch {url}: {e}")
        # Try to use old cached version if available
        if os.path.exists(cache_path):
            debug(f"Using stale cached content for {url}")
            with open(cache_path, 'r', encoding='utf-8') as f:
                return f.read()
        raise


def cleanup_git_locks(repo_path):
    """
    Clean up any stale git lock files that might prevent git operations
    """
    lock_files = [
        '.git/index.lock',
        '.git/HEAD.lock',
        '.git/config.lock',
        '.git/refs/heads/master.lock'
    ]

    cleaned = 0
    for lock_file in lock_files:
        lock_path = os.path.join(repo_path, lock_file)
        if os.path.exists(lock_path):
            try:
                os.remove(lock_path)
                debug(f"Removed stale git lock: {lock_file}")
                cleaned += 1
            except OSError as e:
                debug(f"Could not remove git lock {lock_file}: {e}")

    if cleaned > 0:
        debug(f"Cleaned up {cleaned} git lock file(s)")
    return cleaned


def run_git_command_with_retry(cmd, cwd=None, check=True, max_retries=3):
    """Run git command with retry logic for lock conflicts"""
    if cwd is None:
        cwd = os.getcwd()

    for attempt in range(max_retries):
        try:
            debug(f"Running git command (attempt {attempt + 1}): {cmd}")
            result = subprocess.run(
                cmd.split(),
                cwd=cwd,
                capture_output=True,
                text=True,
                check=check,
                timeout=300  # 5 minute timeout
            )
            if result.stderr:
                debug(f"Git stderr: {result.stderr}")
            return result.stdout

        except subprocess.CalledProcessError as e:
            # Check if it's a lock file issue
            if 'index.lock' in str(e.stderr) or 'Unable to create' in str(e.stderr):
                debug(f"Git lock detected on attempt {attempt + 1}, cleaning up...")
                cleanup_git_locks(cwd)
                if attempt < max_retries - 1:
                    import time
                    time.sleep(1)  # Wait a second before retry
                    continue

            error(f"Git command failed: {cmd}")
            error(f"Error: {e.stderr}")
            if check:
                raise

        except subprocess.TimeoutExpired:
            error(f"Git command timed out: {cmd}")
            if check:
                raise

    # If we get here, all retries failed
    error(f"Git command failed after {max_retries} attempts: {cmd}")
    if check:
        raise subprocess.CalledProcessError(1, cmd)


# Parameters
COMMITFILE = "git-version.txt"
BASEURL = "https://firmware.ardupilot.org/"
ALLVEHICLES = ["AntennaTracker", "Copter", "Plane", "Rover", "Sub", "Blimp"]
VEHICLES = ALLVEHICLES

BASEPATH = ""

# Dicts for name replacing
vehicle_new_to_old_name = { # Used because "param_parse.py" args expect old names
    "Rover": "APMrover2",
    "Sub": "ArduSub",
    "Copter": "ArduCopter",
    "Plane": "ArduPlane",
    "AntennaTracker": "AntennaTracker",  # firmware server calls Tracker as AntennaTracker
    "Blimp": "Blimp",
}

vehicle_old_to_new_name = { # Used because git-version.txt use APMVersion with old names
    "APMrover2": "Rover",
    "ArduRover": "Rover",
    "ArduSub": "Sub",
    "ArduCopter": "Copter",
    "ArduPlane": "Plane",
    "AntennaTracker": "AntennaTracker",  # firmware server calls Tracker/atennatracker as AntennaTracker
    "Blimp": "Blimp",
}


def progress(str_to_print):
    print(f"[build_parameters.py] {str_to_print}")


def debug(str_to_print):
    """Debug output if verbose is set."""
    if args.verbose:
        progress(str_to_print)


def error(str_to_print):
    """Show and count the errors."""
    global error_count
    error_count += 1
    progress("[error]: " + str(str_to_print))


def check_temp_folders():
    """Creates temporaty subfolders IFF not exists  """

    os.chdir("../")
    if not os.path.exists("new_params_mversion"):
        os.makedirs("new_params_mversion")
    os.chdir("new_params_mversion")
    for vehicle in ALLVEHICLES:
        if not os.path.exists(vehicle_new_to_old_name[vehicle]):
            os.makedirs(vehicle_new_to_old_name[vehicle])

    os.chdir("../")
    if not os.path.exists("old_params_mversion"):
        os.makedirs("old_params_mversion")
    os.chdir("old_params_mversion")
    for vehicle in ALLVEHICLES:
        if not os.path.exists(vehicle_new_to_old_name[vehicle]):
            os.makedirs(vehicle_new_to_old_name[vehicle])


def replace_anchors(source_file, dest_file, version_tag):
    """
    For each parameter file generate by param_parse.py, it inserts a version tag in anchors
    to do not make confusing in sphinx toctrees.
    """
    file_in = open(source_file, "r")
    file_out = open(dest_file, "w")
    found_original_title = False
    if "latest" not in version_tag:
        file_out.write(':orphan:\n\n')

    for line in file_in:
        if (re.match("(^.. _)(.*):$", line)) and ("latest" not in version_tag):
            file_out.write(line[0:-2] + version_tag + ":\n")  # renames the anchors, but leave latest anchors "as-is" to maintaim compatibility with all links across the wiki  # noqa: E501

        elif "Complete Parameter List" in line:
            # Adjusting the page title
            out_line = "Complete Parameter List\n=======================\n\n"
            out_line += "\n.. raw:: html\n\n"
            out_line += "   <h2>Full Parameter List of " + version_tag[1:].replace("-", " ") + "</h2>\n\n"  # rename the page identifier to insert the version  # noqa: E501

            # Pigbacking and inserting the javascript selector
            out_line += "\n.. raw:: html\n   :file: ../_static/parameters_versioning_script.inc\n\n"
            file_out.write(out_line)

        elif ("=======================" in line) and (not found_original_title): # Ignores the original mark
            found_original_title = True

        else:
            file_out.write(line)

    file_in.close()
    file_out.close()


def patch_cgi_escape_for_old_versions(version, param_metadata_dir):
    """
    Live patch all Python files in param_metadata for older firmware versions that use cgi.escape()
    which was removed in Python 3.8. This affects htmlemit.py, rstemit.py, and potentially other files.
    """
    try:
        # Parse version to check if it's < 4.1.0
        version_match = re.search(r'(\d+)\.(\d+)\.(\d+)', version)
        if version_match:
            major, minor, _ = map(int, version_match.groups())
            needs_patch = (major < 4) or (major == 4 and minor < 1)
        else:
            needs_patch = True

        if not needs_patch:
            debug(f"Version {version} doesn't need cgi.escape() patching")
            return

        debug(f"Patching Python files for cgi.escape() in old version {version}")

        python_files = glob.glob(os.path.join(param_metadata_dir, "*.py"))
        files_patched = 0

        for file_path in python_files:
            filename = os.path.basename(file_path)

            try:
                with open(file_path, 'rb') as f:
                    content_bytes = f.read()
                content = content_bytes.decode('utf-8')
            except (UnicodeDecodeError, IOError):
                debug(f"Could not read {filename}, skipping")
                continue

            if 'cgi.escape' not in content:
                continue

            debug(f"Patching {filename} for cgi.escape()")

            # Replace cgi.escape with html.escape
            content = content.replace('cgi.escape', 'html.escape')

            # Add 'import html' after 'import cgi' if html not already imported
            if 'import html' not in content and 'from html import' not in content:
                # Simple approach: add after 'import cgi' line
                content = content.replace('import cgi\n', 'import cgi\nimport html\n')
                content = content.replace('import cgi\r\n', 'import cgi\r\nimport html\r\n')

            try:
                with open(file_path, 'wb') as f:
                    f.write(content.encode('utf-8'))
                files_patched += 1
                debug(f"Successfully patched {filename}")
            except IOError as e:
                error(f"Failed to write patched {filename}: {e}")
                continue

        if files_patched > 0:
            debug(f"Patched {files_patched} file(s) for cgi.escape() compatibility")

    except Exception as e:
        error(f"Failed to patch Python files for version {version}: {e}")


def process_single_parameter_file(commit_data):
    """Process a single parameter file (wrapper for serial mode)"""
    return process_single_parameter_file_with_repo(commit_data, BASEPATH)


def optimize_git_repo():
    """Optimize git repository for faster operations"""
    optimize_git_repo_for_path(BASEPATH)


def optimize_git_repo_for_path(repo_path):
    """Optimize git repository at a specific path for faster operations"""
    try:
        cleanup_git_locks(repo_path)
        # Fast git config settings - skip slow gc for temp repos
        run_git_command_with_retry("git config core.preloadindex true", cwd=repo_path, check=False)
        run_git_command_with_retry("git config gc.auto 0", cwd=repo_path, check=False)
        run_git_command_with_retry("git config advice.detachedHead false", cwd=repo_path, check=False)
    except Exception as e:
        debug(f"Git optimization failed for {repo_path}: {e}")


def setup():
    """
    Goes to the work folder, clean it and update.
    """

    # Test for a single vehicle run
    if args.single_vehicle in ALLVEHICLES:
        global VEHICLES
        VEHICLES = [args.single_vehicle]
        progress("Running only for " + str(args.single_vehicle))
    else:
        progress("Vehicle %s not recognized, running for all vehicles." % str(args.single_vehicle))

    global BASEPATH
    try:
        # Goes to ardupilot folder and clean it and update to make sure that is the most recent one.
        debug("Recovering from a previous run...")
        os.chdir(args.gitFolder)

        # Clean up any git locks first
        cleanup_git_locks(os.getcwd())

        # Use subprocess for better git operations with retry logic
        # Set BASEPATH first so optimize_git_repo can use it
        BASEPATH = os.getcwd()

        # Fast git reset to latest master
        run_git_command_with_retry("git fetch origin master")
        run_git_command_with_retry("git checkout -f master")
        run_git_command_with_retry("git reset --hard origin/master")
        run_git_command_with_retry("git submodule update --init --recursive --depth 1")
        optimize_git_repo()
        check_temp_folders()
    except Exception as e:
        error("ArduPilot Repo folder not found (cd %s failed)" % args.gitFolder)
        error(str(e))
        sys.exit(1)
    finally:
        debug("\nThe current working directory is %s" % BASEPATH)


def fetch_releases(firmware_url, vehicles):
    """
    Select folders with desired releases for the vehicles
    """
    def fetch_vehicle_subfolders(vehicle_url):
        """
        Fetch firmware.ardupilot.org/baseURL all first level folders for a given base URL.
        """
        class ParseText(HTMLParser):
            def __init__(self):
                super().__init__()
                self.links = []

            def handle_starttag(self, tag, attrs):
                if tag == 'a':
                    attr = dict(attrs)
                    if 'href' in attr:
                        self.links.append(attr['href'])

        html_parser = ParseText()
        try:
            debug("Fetching " + vehicle_url)
            content = get_cached_url(vehicle_url)
            html_parser.feed(content)
        except Exception as e:
            error(f"Folders list download error: {e}")
            sys.exit(1)

        return html_parser.links

    def get_vehicle_firmware_links(vehicle_url, version_folder):
        firmware_base_url = firmware_url[:-1]

        if "stable" in version_folder and not version_folder.endswith("stable"):
            return firmware_base_url + version_folder
        elif "latest" in version_folder:
            return firmware_base_url + version_folder
        elif "beta" in version_folder:
            return firmware_base_url + version_folder
        else:
            return None

    debug("Cleaning fetched links for the wanted folders")
    filtered_firmware_links = []

    def fetch_vehicle_links(vehicle):
        vehicle_url = firmware_url + vehicle
        page_links = fetch_vehicle_subfolders(vehicle_url)
        for folder in page_links:
            version_folder = str(folder)
            link = get_vehicle_firmware_links(vehicle_url, version_folder)
            if link is not None:
                filtered_firmware_links.append(link)

    with ThreadPoolExecutor() as executor:
        executor.map(fetch_vehicle_links, vehicles)
    return filtered_firmware_links


def get_commit_dict(releases_parsed):
    """
    For informed releases, return a dict git hashs of its build.

    """
    def get_last_board_folder(url):
        """
        For given URL returns the last folder which should be a board name.

        """
        links = []
        # Define HTML Parser

        class ParseText(HTMLParser):
            def handle_starttag(self, tag, attrs):
                if tag != 'a':
                    return
                attr = dict(attrs)
                links.append(attr)
        # Create instance of HTML parser
        lParser = ParseText()
        # Feed HTML file into parsers
        try:
            debug("Fetching " + url)
            content = get_cached_url(url)
            lParser.feed(content)
        except Exception as e:
            error(f"Folders list download error: {e}")
            return None
        finally:
            lParser.close()

        if links:  # Only proceed if we have links
            last_item = links.pop()
            last_folder = last_item['href']
            debug("Returning link of the last board folder (" + last_folder[last_folder.rindex('/')+1:] + ")")
            return last_folder[last_folder.rindex('/')+1:]  # clean the partial link
        else:
            error(f"No board folders found for {url}")
            return None
    ####################################################################################################

    def fetch_commit_hash(version_link, board, file):
        """
        For a binnary folder, gets a git hash of its build.

        """
        fetch_link = version_link + '/' + board + '/' + file

        progress("Processing link...\t" + fetch_link)

        try:
            fecth_response = get_cached_url(fetch_link)
            commit_details = fecth_response.split("\n")
            commit_hash = commit_details[0][7:]
            # version =  commit_details[6] the sizes cary
            version = commit_details.pop(-2)

            version_number = version.split(" ")[2]
            vehicle = version.split(" ")[1]

            regex = re.compile(r'[@_!#$%^&*()<>?/\|}{~:]')

            if regex.search(vehicle) is None:  # there are some non standart names
                vehicle = vehicle_old_to_new_name[vehicle.strip()]   # Names may not be standart as expected
            else:
                # tries to fix automatically
                if re.search('copter', vehicle, re.IGNORECASE):
                    vehicle = "Copter"
                    debug("Bad vehicle name auto fixed to COPTER on:\t" + fetch_link)

                elif re.search('plane', vehicle, re.IGNORECASE):
                    vehicle = "Plane"
                    debug("Bad vehicle name auto fixed to PLANE on:\t" + fetch_link)

                elif re.search('rover', vehicle, re.IGNORECASE):
                    vehicle = "Rover"
                    debug("Bad vehicle name auto fixed to ROVER on:\t" + fetch_link)

                elif re.search('sub', vehicle, re.IGNORECASE):
                    vehicle = "Sub"
                    debug("Bad vehicle name auto fixed to SUB on:\t" + fetch_link)

                elif re.search('racker', vehicle, re.IGNORECASE):
                    vehicle = "Tracker"
                    debug("Bad vehicle name auto fixed to TRACKER on:\t" + fetch_link)
                elif re.search('blimp', vehicle, re.IGNORECASE):
                    vehicle = "Blimp"
                    debug("Bad vehicle name auto fixed to BLIMP on:\t" + fetch_link)
                else:
                    error("Nomenclature exception found in a vehicle name:\t" + vehicle + "\tLink with the exception:\t" + fetch_link)  # noqa: E501

            if "beta" in fetch_link:
                version_number = "beta-" + version_number

            if "latest" in fetch_link:
                version_number = "latest-" + version_number

            return vehicle, version_number, commit_hash
        except Exception as e:
            error("An exception occurred: " + file + " DECODE ERROR. Link: " + fetch_link)
            error(e)
            # sys.exit(1) #comment to make easer debug
            return "error", "error", "error"
    ####################################################################################################

    commits_and_codes = {}
    commite_and_codes_cleanned = {}

    def fetch_data(release):
        board_folder = get_last_board_folder(release)
        return fetch_commit_hash(release, board_folder, COMMITFILE)

    with ThreadPoolExecutor() as executor:
        commits_and_codes = list(executor.map(fetch_data, releases_parsed))

    commite_and_codes_cleanned = {i: cc for i, cc in enumerate(commits_and_codes) if cc[0] != 'error'}

    if len(commite_and_codes_cleanned) == 0:
        error("Expected at least one commit")
    return commite_and_codes_cleanned


def create_vehicle_git_repos():
    """
    Create separate git repository copies for each vehicle to enable parallel processing
    """
    vehicle_repos = {}

    # Use a writable temporary directory - try multiple locations
    temp_base_options = [
        "/tmp",
        os.path.dirname(BASEPATH),
        os.path.expanduser("~/tmp"),
        "."
    ]

    temp_base = None
    for option in temp_base_options:
        if os.path.exists(option) and os.access(option, os.W_OK):
            temp_base = option
            break

    if temp_base is None:
        debug("No writable temporary directory found, falling back to serial processing")
        progress("Warning: Parallel processing disabled due to permission constraints")
        for vehicle in VEHICLES:
            vehicle_repos[vehicle] = BASEPATH
        return vehicle_repos

    debug(f"Using temporary directory base: {temp_base}")

    for vehicle in VEHICLES:
        # Create temporary directory for this vehicle in a writable location
        temp_repo_dir = os.path.join(temp_base, f"ardupilot_temp_{vehicle}")

        debug(f"Creating temporary git repo for {vehicle} at {temp_repo_dir}")

        try:
            # Remove existing temp directory if it exists
            if os.path.exists(temp_repo_dir):
                shutil.rmtree(temp_repo_dir)

            # Clone the repository using git clone --shared for efficiency
            # --shared creates a repository that shares objects with the original
            run_git_command_with_retry(f"git clone --shared {BASEPATH} {temp_repo_dir}")

            # Configure and optimize the temporary repository
            run_git_command_with_retry("git config advice.detachedHead false", cwd=temp_repo_dir, check=False)

            # Clean up any potential locks in the new repo
            cleanup_git_locks(temp_repo_dir)

            # Optimize the temporary repository for faster operations
            optimize_git_repo_for_path(temp_repo_dir)

            # Store the path for this vehicle
            vehicle_repos[vehicle] = temp_repo_dir

        except Exception as e:
            error(f"Failed to create temp repo for {vehicle}: {e}")
            # Fallback to using the main repo
            vehicle_repos[vehicle] = BASEPATH

    # Also clean up locks in the main repository
    cleanup_git_locks(BASEPATH)

    return vehicle_repos


def cleanup_vehicle_git_repos(vehicle_repos):
    """
    Clean up temporary git repositories
    """
    for vehicle, repo_path in vehicle_repos.items():
        if repo_path != BASEPATH and os.path.exists(repo_path):
            try:
                debug(f"Cleaning up temp repo for {vehicle}")
                shutil.rmtree(repo_path)
            except Exception as e:
                debug(f"Failed to cleanup temp repo for {vehicle}: {e}")


def collect_generated_files(vehicle_repos):
    """
    Collect all generated parameter files from temporary repositories to the main repository
    """
    main_param_dir = BASEPATH + "/Tools/autotest/param_metadata"

    # Ensure we're in the main param directory
    os.chdir(main_param_dir)

    # Clean any existing parameter files first
    existing_files = glob.glob("parameters-*.rst") + glob.glob("parameters-*.json")
    for old_file in existing_files:
        try:
            os.remove(old_file)
        except OSError:
            pass

    collected_count = 0
    for vehicle, repo_path in vehicle_repos.items():
        if repo_path == BASEPATH:
            continue  # Skip if using main repo

        temp_param_dir = repo_path + "/Tools/autotest/param_metadata"
        if not os.path.exists(temp_param_dir):
            continue

        # Find parameter files in the temporary repo
        temp_param_files = glob.glob(os.path.join(temp_param_dir, "parameters-*.rst"))

        for temp_file in temp_param_files:
            filename = os.path.basename(temp_file)
            dest_file = os.path.join(main_param_dir, filename)

            try:
                shutil.copy2(temp_file, dest_file)
                collected_count += 1
                debug(f"Collected {filename} from {vehicle} temp repo")
            except Exception as e:
                error(f"Failed to collect {filename} from {vehicle}: {e}")

    debug(f"Collected {collected_count} parameter files to main repository")


def process_vehicle_commits(vehicle_commits_data):
    """
    Process all commits for a specific vehicle in parallel
    """
    vehicle, commits, repo_path = vehicle_commits_data
    results = []

    debug(f"Processing {len(commits)} commits for {vehicle} in {repo_path}")

    for commit_data in commits:
        # Update the commit data to use the vehicle-specific repo
        result = process_single_parameter_file_with_repo(commit_data, repo_path)
        if result:
            results.append(result)
            debug(f"Successfully processed: {result}")

    return vehicle, results


def process_single_parameter_file_with_repo(commit_data, repo_path):
    """Process a single parameter file using a specific repository path"""
    vehicle, version, commit_id = commit_data

    # Not elegant workaround:
    # These versions present errors when parsing using param_parser.py. Needs more investigation?
    if (
        "3.2.1" in version or # last stable APM Copte
        "3.4.0" in version or # last stable APM Plane
        "3.4.6" in version or # Copter
        "2.42" in version or  # last stable APM Rover?
        "2.51" in version or  # last beta APM Rover?
        "0.7.2" in version    # Antennatracker
    ):
        debug("Ignoring APM version:\t" + vehicle + "\t" + version)
        return None

    # Checkout an Commit ID in order to get its parameters
    try:
        debug("Git checkout on " + vehicle + " version " + version + " id " + commit_id)
        run_git_command_with_retry(f"git checkout --force {commit_id}", cwd=repo_path, check=True)

    except Exception as e:
        error("GIT checkout error: " + str(e))
        return None

    debug("")

    # Run param_parse.py tool from Autotest set in the desidered commit id
    # NOTE: We use absolute paths and cwd parameter instead of os.chdir()
    # because os.chdir() affects the entire process and causes race conditions
    # in multi-threaded execution.
    try:
        param_metadata_dir = os.path.join(repo_path, "Tools", "autotest", "param_metadata")

        # Check if param_metadata directory exists (may not in very old commits)
        if not os.path.exists(param_metadata_dir):
            error(f"param_metadata directory not found for {vehicle} {version}: {param_metadata_dir}")
            return None

        # Check if param_parse.py exists
        param_parse_path = os.path.join(param_metadata_dir, "param_parse.py")
        if not os.path.exists(param_parse_path):
            error(f"param_parse.py not found for {vehicle} {version}")
            return None

        # Patch emit files for older versions that use deprecated cgi.escape()
        patch_cgi_escape_for_old_versions(version, param_metadata_dir)

        # Build the command
        if ('rover' in vehicle.lower()) and ('v3.' not in version.lower()) and ('v4.0' not in version.lower()):
            vehicle_name = 'Rover'
        else:
            vehicle_name = vehicle_new_to_old_name[vehicle]

        cmd = ["python3", "./param_parse.py", "--vehicle", vehicle_name]

        # Run with subprocess - use cwd parameter instead of os.chdir()
        result = subprocess.run(cmd, cwd=param_metadata_dir,
                                capture_output=True, text=True, timeout=300)

        if result.returncode != 0:
            error(f"param_parse.py failed for {vehicle} {version}: {result.stderr}")
            return None

        # Log param_parse.py output for debugging
        if result.stdout:
            debug(f"param_parse.py stdout for {vehicle} {version}: {result.stdout[:500]}")
        if result.stderr:
            debug(f"param_parse.py stderr for {vehicle} {version}: {result.stderr[:500]}")

        # create a filename for new parameters file
        filename = "parameters-" + vehicle
        if ("beta" in version or "rc" in version): # Plane uses BETA, Copter and Rover uses RCn
            filename += "-" + version  + ".rst"
        elif ("latest" in version):
            filename += "-" + version + ".rst"
        else:
            filename += "-stable-" + version + ".rst"

        # Use absolute paths for file operations
        parameters_rst_path = os.path.join(param_metadata_dir, "Parameters.rst")
        output_file_path = os.path.join(param_metadata_dir, filename)

        # Generate new anchors names in files to avoid toctree problems and links in sphinx.
        if os.path.exists(parameters_rst_path):
            replace_anchors(parameters_rst_path, output_file_path, filename[10:-4])
            os.remove(parameters_rst_path)
            debug("File " + filename + " generated. ")
            return filename
        else:
            error(f"Parameters.rst not found for {vehicle} {version}")
            return None

    except Exception as e:
        error(f"Error while processing {vehicle} {version} ({commit_id}): {type(e).__name__}: {e}")
        return None


def generate_rst_files(commits_to_checkout_and_parse):
    """
    For each git hash it generates its Parameters file.
    Uses parallel processing by vehicle if enabled.
    """
    if not args.parallel_vehicles or len(VEHICLES) == 1:
        # Use serial processing (original method)
        debug("Using serial processing for parameter files")
        for i in commits_to_checkout_and_parse:
            commit_data = commits_to_checkout_and_parse[i]
            result = process_single_parameter_file(commit_data)
            if result:
                debug(f"Successfully processed: {result}")
        return 0

    # Use parallel processing by vehicle
    debug(f"Using parallel processing for {len(VEHICLES)} vehicles")

    # Group commits by vehicle
    vehicle_commits = {}
    for i, commit_data in commits_to_checkout_and_parse.items():
        vehicle, version, commit_id = commit_data
        if vehicle not in vehicle_commits:
            vehicle_commits[vehicle] = []
        vehicle_commits[vehicle].append(commit_data)

    # Create separate git repositories for each vehicle
    vehicle_repos = create_vehicle_git_repos()

    # Check if we actually got separate repositories
    unique_repos = set(vehicle_repos.values())
    if len(unique_repos) == 1 and BASEPATH in unique_repos:
        debug("All vehicles fell back to main repository, using serial processing")
        for i in commits_to_checkout_and_parse:
            commit_data = commits_to_checkout_and_parse[i]
            result = process_single_parameter_file(commit_data)
            if result:
                debug(f"Successfully processed: {result}")
        return 0

    try:
        # Prepare data for parallel processing
        vehicle_data = []
        for vehicle, commits in vehicle_commits.items():
            repo_path = vehicle_repos.get(vehicle, BASEPATH)
            vehicle_data.append((vehicle, commits, repo_path))

        # Process vehicles in parallel
        all_results = []
        with ThreadPoolExecutor(max_workers=min(len(VEHICLES), 4)) as executor:
            vehicle_results = list(executor.map(process_vehicle_commits, vehicle_data))

            for vehicle, results in vehicle_results:
                all_results.extend(results)
                debug(f"Completed processing {len(results)} files for {vehicle}")

        # Collect all generated files to the main repository
        collect_generated_files(vehicle_repos)

        progress(f"Generated {len(all_results)} parameter files total")

    finally:
        # Clean up temporary repositories
        cleanup_vehicle_git_repos(vehicle_repos)

    return 0


def generate_json(vehicles):
    """
        Generates a JSON with all parameters page to be live consumed by a javascript
    """

    os.chdir(BASEPATH + "/Tools/autotest/param_metadata")

    for vehicle in vehicles:

        debug("Creating JSON files for " + vehicle)

        # Creates the JSON lines from available rst files
        parameters_files = [f for f in glob.glob("parameters-" + vehicle + "*.rst")]
        parameters_files.sort(reverse=True)

        json_lines = []
        json_lines.append("{")
        first = True

        for filename in parameters_files:
            if first:
                first = False
            else:
                json_lines.append(",")
            if ("beta" in filename or "rc" in filename): # Plane uses BETA, Copter and Rover uses RCn
                json_lines.append("\"" + vehicle + " beta " + filename[(len("parameters-" + vehicle + "-beta")+1):-4] + "\" : \"" + filename[:-3] + "html\"")  # noqa: E501
            elif ("latest" in filename):
                # json_lines.append(",\"" +  vehicle + " latest " + filename[(len("parameters-" + vehicle + "-latest")+1):-4] + "\" : \"" + filename[:-3] + "html\"")  # noqa: E501
                json_lines.append("\"" + vehicle + " latest " + filename[(len("parameters-" + vehicle + "-latest")+1):-4] + "\" : \"" + ("parameters.html\""))  # Trying to re-enable toc list on the left bar on the wiki by forcing latest file name.  # noqa: E501

            else:
                json_lines.append("\"" + vehicle + " stable " + filename[(len("parameters-" + vehicle + "-stable")+1):-4] + "\" : \"" + filename[:-3] + "html\"")  # noqa: E501

        json_lines.append("}")

        # Saves the JSON
        json_filename = "parameters-" + vehicle + ".json"
        try:
            with open(json_filename, 'w') as f:
                for lines in json_lines:
                    f.write("%s\n" % lines)
        except Exception as e:
            error("Error while creating the JSON file " + vehicle + " in folder " + str(os.getcwd()))  # noqa: E501
            error(e)
            # sys.exit(1)
        debug("")


def move_results(vehicles):
    """
        Once all parameters files are created, moves for "new_params_mversion" as the last execution result
    """

    os.chdir(BASEPATH + "/Tools/autotest/param_metadata")

    for vehicle in vehicles:
        debug("Moving created files for " + vehicle)
        try:
            folder = args.destFolder + "/" + vehicle_new_to_old_name[vehicle] + "/"
            debug("Destination folder:\t" + folder)
            debug("Current folder:\t" + str(os.getcwd()))
            debug("Destination folder absolute path:\t" + os.path.abspath(folder))

            # touch the folders
            if not os.path.exists(args.destFolder):
                os.makedirs(args.destFolder)
            if not os.path.exists(folder):
                os.makedirs(folder)

            # Cleanning last run, iff exists
            files_to_delete = [f for f in glob.glob(folder + "*")]
            for old_file in files_to_delete:
                os.remove(old_file)

            # Moving files (use shutil.move for cross-device compatibility)
            files_to_move = [f for f in glob.glob("parameters-" + vehicle + "*")]
            for file in files_to_move:
                if "latest" not in file:  # Trying to re-enable toc list on the left bar on the wiki by forcing latest file name.  # noqa: E501
                    shutil.move(file, folder + file)
                else:
                    shutil.move(file, str((folder + "parameters.rst")))

        except Exception as e:
            error("Error while moving result files of vehicle " + vehicle + " pwd: " + str(os.getcwd()))
            error(e)
            # sys.exit(1)


def print_versions(commits_to_checkout_and_parse):
    """ Partial results: present all vechicles, versions and commits selected to generate parameters """
    debug("\n\tList of parameters files to generate:\n")
    for i in commits_to_checkout_and_parse:
        debug("\n\t" + commits_to_checkout_and_parse[i][0] + ' - ' + commits_to_checkout_and_parse[i][1] + ' - ' + commits_to_checkout_and_parse[i][2])  # noqa: E501
    debug("")


# Step 1 - Select the versions for generate parameters
start_time = time.time()
progress("=== Step 1: Setup and fetch release information ===")
setup()                                                             # Reset the ArduPilot folder/repo
feteched_releases = fetch_releases(BASEURL, VEHICLES)               # All folders/releases.
commits_to_checkout_and_parse = get_commit_dict(feteched_releases)  # Parse names, and git hashes.
print_versions(commits_to_checkout_and_parse)                       # Present work dict.

# Step 2 - Generates them in ArdupilotRepoFolder/Tools/autotest/param_metadata
progress("=== Step 2: Generate parameter files ===")
progress(f"Time elapsed so far: {time.time() - start_time:.2f} seconds")
total_commits = len(commits_to_checkout_and_parse)
progress(f"Processing {total_commits} commit(s)...")
generate_rst_files(commits_to_checkout_and_parse)

# Step 3 - Generates a JSON file for each vehicle and move files to folder new_params_mversion
progress("=== Step 3: Generate JSON files and move results ===")
progress(f"Time elapsed so far: {time.time() - start_time:.2f} seconds")
generate_json(VEHICLES)
move_results(VEHICLES)

progress("=== Build completed ===")
total_time = time.time() - start_time
progress(f"Total execution time: {total_time:.2f} seconds ({total_time/60:.1f} minutes)")
progress(f"Total errors encountered: {error_count}")
sys.exit(error_count)

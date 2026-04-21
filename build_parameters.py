#!/usr/bin/env python3
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
        * It parses that to get the version and commit hash;
        * It creates a dictionary with vehicles, versions and commit hashes.

    * Second step is use the dict to navigates on a ArduPilot Repo, changing checkouts to desired hashes.
        * Relies on ArduPilotRepoFolder/Tools/autotest/param_metadata/param_parse.py to generate the parameters files;
        * It renames the anchors for all files, except for latest versions.

    * Third step: create the json files and move all generated files.

"""

import argparse
import glob
import json
import logging
import os
import re
import shutil  # noqa: F401
import subprocess
import sys
import time  # noqa: F401
from html.parser import HTMLParser

import requests

parser = argparse.ArgumentParser(description="python3 build_parameters.py [options]")
parser.add_argument("--verbose", dest='verbose', action='store_false', default=True, help="show debugging output")
parser.add_argument("--ardupilotRepoFolder", dest='gitFolder', default="../ardupilot", help="Ardupilot git folder. ")
parser.add_argument("--destination", dest='destFolder', default="../../../../new_params_mversion", help="Parameters*.rst destination folder.")  # noqa: E501
parser.add_argument('--vehicle', dest='single_vehicle', help="If you just want to copy to one vehicle, you can do this. Otherwise it will work for all vehicles (Copter, Plane, Rover, AntennaTracker, Sub, Blimp)")  # noqa: E501
args = parser.parse_args()

error_count = 0


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


handler = logging.StreamHandler(sys.stdout)
handler.setFormatter(ColoredFormatter('[build_parameters.py]: [%(levelname)s]: %(message)s'))
logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, handlers=[handler])
logger = logging.getLogger(__name__)

# Global session for HTTP requests with connection pooling
session = requests.Session()
session.headers.update({
    'User-Agent': 'Mozilla/5.0 (compatible; ArduPilotWikiBuilder/1.0)',
    'Accept': 'text/html,application/xhtml+xml,application/xml;q=0.9,*/*;q=0.8',
    'Connection': 'keep-alive'
})

# Parameters
COMMITFILE = "git-version.txt"
BASEURL = "https://firmware.ardupilot.org/"
ALLVEHICLES = ["AntennaTracker", "Copter", "Plane", "Rover", "Sub", "Blimp"]
VEHICLES = ALLVEHICLES

BASEPATH = ""


def rst_has_duplicate_labels(filepath: str) -> bool:
    """
    Check an RST file for duplicate label definitions.
    Returns True if duplicates are found, False otherwise.

    RST labels look like: .. _label_name:
    """
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
    except (UnicodeDecodeError, OSError) as e:
        error(f"Error checking RST file {filepath}: {e}")
        return False

    # Find all RST label definitions
    labels = re.findall(r'^\.\. _([^:]+):', content, re.MULTILINE)

    # Check for duplicates
    seen = set()
    duplicates = []
    for label in labels:
        label_lower = label.lower()  # RST labels are case-insensitive
        if label_lower in seen:
            duplicates.append(label)
        seen.add(label_lower)

    if duplicates:
        logger.warning(f"Found {len(duplicates)} duplicate RST labels in {filepath}: {duplicates[:5]}")
        return True
    return False


def dedupe_rngfnd_parameters_sections(filepath: str) -> None:
    """Keep only the first RNGFNDx_* Parameters section and drop subsequent duplicate sections."""
    label_re = re.compile(r'^\.\. _parameters_RNGFND([0-9A-Za-z]+)_.*:$', re.IGNORECASE)
    section_label_re = re.compile(r'^\.\. _RNGFND([0-9A-Za-z]+).*:$', re.IGNORECASE)

    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
    except (UnicodeDecodeError, OSError) as e:
        debug(f"Failed to dedupe RNGFND parameters in {filepath}: {e}")
        return

    seen_labels = set()
    saved_lines = []  # Lines to write back to the file after deduplication
    i = 0
    while i < len(lines):
        current_line = lines[i]
        current_line_stripped = current_line.strip()

        rangefinder_label = label_re.match(current_line_stripped)
        if rangefinder_label:
            rngfnd_id = rangefinder_label.group(1)
            if rngfnd_id in seen_labels:
                # Don't save the duplicated line on new file, skip to next section
                i += 1
                while i < len(lines) and not section_label_re.match(lines[i].strip()):
                    i += 1
                continue

            seen_labels.add(rngfnd_id)

        saved_lines.append(current_line)
        i += 1

    if len(saved_lines) != len(lines):
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.writelines(saved_lines)
            debug(f"Dedupe applied to RNGFND sections in {filepath}")
        except (UnicodeDecodeError, OSError) as e:
            debug(f"Failed to dedupe RNGFND parameters in {filepath}: {e}")


def patch_cgi_escape_for_old_versions(version, param_metadata_dir):
    """
    Live patch all Python files in param_metadata for older firmware versions that use cgi.escape()
    which was removed in Python 3.8. This affects htmlemit.py, rstemit.py, and potentially other files.
    """

    # Parse version to check if it's < 4.1.0
    if not version_is_below_version(version, (4, 1, 0)):
        # debug(f"Version {version} doesn't need cgi.escape() patching")
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


def parse_version(version_string: str) -> tuple[int, int, int] | None:
    """Parse the first semantic version-like string from a version token."""
    match = re.search(r"(\d+)\.(\d+)(?:\.(\d+))?", version_string)
    if not match:
        return None
    major = int(match.group(1))
    minor = int(match.group(2))
    patch = int(match.group(3) or 0)
    return major, minor, patch


def version_is_below_version(version_string: str, cutoff: tuple[int, int, int]) -> bool:
    parsed = parse_version(version_string)
    if parsed is None:
        return False
    return parsed < cutoff


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
    "AntennaTracker": "AntennaTracker",  # firmware server calls Tracker/antennatracker as AntennaTracker
    "Blimp": "Blimp",
}


def progress(msg):
    """Log info level message."""
    logger.info(msg)


def debug(msg):
    """Log debug level message."""
    logger.debug(msg)


def error(msg):
    """Log error and count errors."""
    global error_count
    error_count += 1
    logger.error(msg)


def check_temp_folders():
    """Creates temporary subfolders IFF not exists  """

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


def setup():
    """
    Goes to the work folder, clean it and update.
    """

    # Test for a single vehicle run
    if args.single_vehicle in ALLVEHICLES:
        global VEHICLES
        VEHICLES = [args.single_vehicle]
        progress(f"Running only for {args.single_vehicle}")
    else:
        progress(f"Vehicle {str(args.single_vehicle)} not recognized, running for all vehicles.")

    try:
        # Goes to ardupilot folder and clean it and update to make sure that is the most recent one.
        debug("Recovering from a previous run...")
        os.chdir(args.gitFolder)
        os.system("git reset --hard HEAD")
        os.system("git clean -f -d")
        os.system("git checkout -f master")
        os.system("git fetch origin master")
        os.system("git reset --hard origin/master")
        os.system("git pull")
        global BASEPATH
        BASEPATH = os.getcwd()
        check_temp_folders()
        os.chdir(BASEPATH) # Need to call git command correctly
    except Exception as e:
        error(f"ArduPilot Repo folder not found (cd {args.gitFolder} failed)")
        error(e)
        sys.exit(1)
    finally:
        debug(f"\nThe current working directory is {BASEPATH}")


def fetch_releases(firmware_url, vehicles):
    """
    Select folders with the desired releases for the vehicles

    """

    def fetch_vehicle_subfolders(firmware_url, vehicle):
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
                    href = attr.get('href')
                    self.links.append(href)

        html_parser = ParseText()
        try:
            debug(f"Fetching {firmware_url}{vehicle}")

            response = session.get(firmware_url + vehicle, timeout=30)
            response.raise_for_status()
            content = response.text
            html_parser.feed(content)
        except Exception as e:
            error(f"Vehicles folders list download error: {e}")
            sys.exit(1)
        return html_parser.links
    ######################################################################################

    debug("Cleaning fetched links for wanted folders")
    stableFirmwares = []
    for vehicle in vehicles:
        page_links = fetch_vehicle_subfolders(firmware_url, vehicle)

        for folder in page_links:  # Non clever way to filter the strings insert by makehtml.py, unwanted folders, and so.
            version_folder = str(folder)
            firmware_version_url = f"{firmware_url[:-1]}{version_folder}"
            if "stable" in version_folder and not version_folder.endswith("stable"): # If finish with
                stableFirmwares.append(firmware_version_url)
            elif "latest" in version_folder:
                stableFirmwares.append(firmware_version_url)
            elif "beta" in version_folder:
                stableFirmwares.append(firmware_version_url)

    return stableFirmwares # links for the firmwares folders


def get_commit_dict(releases_parsed):
    """
    For informed releases, return a dict git hashes of its build.

    """
    def get_last_board_folder(url):
        """
        For given URL returns the last folder which should be a board name.

        """

        class ParseText(HTMLParser):
            def __init__(self):
                super().__init__()
                self.links = []

            def handle_starttag(self, tag, attrs):
                if tag == 'a':
                    attr = dict(attrs)
                    href = attr.get('href')
                    self.links.append(href)

        html_parser = ParseText()
        try:
            debug(f"Fetching {url}")

            response = session.get(url, timeout=30)
            response.raise_for_status()
            content = response.text
            html_parser.feed(content)
        except Exception as e:
            error(f"Board folders list download error: {e}")
        finally:
            last_folder = html_parser.links.pop()
            board_name = os.path.basename(last_folder)
            debug(f"Returning link of the last board folder ({board_name})")
            return board_name
    ####################################################################################################

    def fetch_commit_hash(version_link, board, file):
        """
        For a binary folder, gets a git hash of its build.

        """
        fetch_link = f"{version_link}/{board}/{file}"

        progress(f"Processing link...\t{fetch_link}")

        try:
            response = session.get(fetch_link, timeout=30)
            response.raise_for_status()
            fecth_response = response.text

            commit_details = fecth_response.split("\n")
            commit_hash = commit_details[0][7:]
            # version =  commit_details[6] the sizes cary
            version = commit_details.pop(-2)

            version_number = version.split(" ")[2]
            vehicle = version.split(" ")[1]

            regex = re.compile(r'[@_!#$%^&*()<>?/\|}{~:]')

            if (regex.search(vehicle) is None):  # there are some non standard names
                vehicle = vehicle_old_to_new_name[vehicle.strip()]   # Names may not be standard as expected
            else:
                # tries to fix automatically
                if re.search('copter', vehicle, re.IGNORECASE):
                    vehicle = "Copter"
                    debug(f"Bad vehicle name auto fixed to COPTER on:\t{fetch_link}")

                elif re.search('plane', vehicle, re.IGNORECASE):
                    vehicle = "Plane"
                    debug(f"Bad vehicle name auto fixed to PLANE on:\t{fetch_link}")

                elif re.search('rover', vehicle, re.IGNORECASE):
                    vehicle = "Rover"
                    debug(f"Bad vehicle name auto fixed to ROVER on:\t{fetch_link}")

                elif re.search('sub', vehicle, re.IGNORECASE):
                    vehicle = "Sub"
                    debug(f"Bad vehicle name auto fixed to SUB on:\t{fetch_link}")

                elif re.search('racker', vehicle, re.IGNORECASE):
                    vehicle = "Tracker"
                    debug(f"Bad vehicle name auto fixed to TRACKER on:\t{fetch_link}")
                elif re.search('blimp', vehicle, re.IGNORECASE):
                    vehicle = "Blimp"
                    debug(f"Bad vehicle name auto fixed to BLIMP on:\t{fetch_link}")
                else:
                    error(f"Nomenclature exception found in a vehicle name:\t{vehicle}\tLink with the exception:\t{fetch_link}")  # noqa: E501

            if "beta" in fetch_link:
                version_number = f"beta-{version_number}"

            if "latest" in fetch_link:
                version_number = f"latest-{version_number}"

            return vehicle, version_number, commit_hash
        except Exception as e:
            error(f"An exception occurred: {file} DECODE ERROR. Link: {fetch_link}")
            error(e)
            # sys.exit(1) #comment to make easier debug
            return "error", "error", "error"
    ####################################################################################################

    commits_and_codes = {}
    commite_and_codes_cleanned = {}

    for j in range(0, len(releases_parsed)):
        board_folder = get_last_board_folder(releases_parsed[j])
        commits_and_codes[j] = fetch_commit_hash(releases_parsed[j], board_folder, COMMITFILE)

    for i in commits_and_codes:
        if commits_and_codes[i][0] != 'error':
            commite_and_codes_cleanned[i] = commits_and_codes[i]
    if len(commite_and_codes_cleanned) == 0:
        error("Expected at least one commit")
    return commite_and_codes_cleanned


def generate_rst_files(commits_to_checkout_and_parse):
    """
    For each git hash it generates its Parameters file.
    """

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
                file_out.write(f"{line[0:-2]}{version_tag}:\n")  # renames the anchors, but leave latest anchors "as-is" to maintain compatibility with all links across the wiki  # noqa: E501

            elif "Complete Parameter List" in line:
                # Adjusting the page title
                out_line = "Complete Parameter List\n=======================\n\n"
                out_line += "\n.. raw:: html\n\n"
                out_line += f"   <h2>Full Parameter List of {version_tag[1:].replace('-', ' ')}</h2>\n\n"  # rename the page identifier to insert the version  # noqa: E501

                # Pigbacking and inserting the javascript selector
                out_line += "\n.. raw:: html\n   :file: ../_static/parameters_versioning_script.inc\n\n"
                file_out.write(out_line)

            elif ("=======================" in line) and (not found_original_title): # Ignores the original mark
                found_original_title = True

            else:
                file_out.write(line)

    for i in commits_to_checkout_and_parse:
        vehicle = str(commits_to_checkout_and_parse[i][0])
        version = str(commits_to_checkout_and_parse[i][1])
        commit_id = str(commits_to_checkout_and_parse[i][2])

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
            debug(f"Ignoring APM version:\t{vehicle}\t{version}")
            continue

        # Checkout an Commit ID in order to get its parameters
        try:
            debug(f"Git checkout on {vehicle} version {version} id {commit_id}")
            os.system(f"git checkout --force {commit_id}")

        except Exception as e:
            error(f"GIT checkout error: {e}")
            sys.exit(1)
        debug("")

        # Run param_parse.py tool from Autotest set in the desired commit id
        param_metadata_dir = os.path.join(BASEPATH, "Tools", "autotest", "param_metadata")

        # Patch emit files for older versions that use deprecated cgi.escape()
        patch_cgi_escape_for_old_versions(version, param_metadata_dir)

        # Workaround the vehicle renaming (Rover, APMRover2 ArduRover...)
        if ('rover' in vehicle.lower()) and ('v3.' not in version.lower()) and ('v4.0' not in version.lower()):
            vehicle_name = 'Rover'
        else:
            vehicle_name = vehicle_new_to_old_name[vehicle]

        cmd = ["python3", "./param_parse.py", "--vehicle", vehicle_name]

        try:
            result = subprocess.run(cmd, cwd=param_metadata_dir,
                                    capture_output=True, text=True, timeout=300)
        except (subprocess.TimeoutExpired, OSError) as e:
            error(f"param_parse.py execution failed for {vehicle} {version}: {e}")
            return None

        if result.returncode != 0:
            error(f"param_parse.py failed for {vehicle} {version}: {result.stderr}")
            return None

        if result.stdout:
            debug(f"param_parse.py stdout for {vehicle} {version}: {result.stdout[:500]}")
        if result.stderr:
            debug(f"param_parse.py stderr for {vehicle} {version}: {result.stderr[:500]}")

        # create a filename for new parameters file
        filename = f"parameters-{vehicle}"
        if ("beta" in version or "rc" in version): # Plane uses BETA, Copter and Rover uses RCn
            filename += f"-{version}.rst"
        elif ("latest" in version):
            filename += f"-{version}.rst"
        else:
            filename += f"-stable-{version}.rst"

        parameters_rst_path = os.path.join(param_metadata_dir, "Parameters.rst")
        output_file_path = os.path.join(param_metadata_dir, filename)

        # Generate new anchors names in files to avoid toctree problems and links in sphinx.
        try:
            if os.path.exists(parameters_rst_path):
                replace_anchors(parameters_rst_path, output_file_path, filename[10:-4])
                os.remove(parameters_rst_path)
                debug(f"File {filename} generated.")
                # Remove duplicate RNGFNDx_ Parameters sections before checking labels.
                dedupe_rngfnd_parameters_sections(output_file_path)
                # Check for duplicate RST labels in the generated file
                if rst_has_duplicate_labels(output_file_path):
                    debug(f"RST duplicate labels detected in {output_file_path}")
            else:
                error(f"Parameters.rst not found for {vehicle} {version}")
        except (OSError, IOError) as e:
            error(f"Error while handling Parameters.rst for {vehicle} {version}: {e}")

    return 0


def generate_json(vehicles):
    """
        Generates a JSON with all parameters page to be live consumed by a javascript
    """

    os.chdir(f"{BASEPATH}/Tools/autotest/param_metadata")

    for vehicle in vehicles:

        debug(f"Creating JSON files for {vehicle}")

        # Creates the JSON lines from available rst files
        parameters_files = [f for f in glob.glob(f"parameters-{vehicle}*.rst")]
        parameters_files.sort(reverse=True)

        vehicle_json = {}

        for filename in parameters_files:
            if "beta" in filename or "rc" in filename:  # Plane uses BETA, Copter and Rover uses RCn
                key = f"{vehicle} beta {filename[len('parameters-'+vehicle+'-beta')+1:-4]}"
                target = f"{filename[:-3]}html"
            elif "latest" in filename:
                key = f"{vehicle} latest {filename[len('parameters-'+vehicle+'-latest')+1:-4]}"
                target = "parameters.html"
            else:
                key = f"{vehicle} stable {filename[len('parameters-'+vehicle+'-stable')+1:-4]}"
                target = f"{filename[:-3]}html"

            vehicle_json[key] = target

        json_filename = f"parameters-{vehicle}.json"
        try:
            with open(json_filename, "w", encoding="utf-8") as f:
                json.dump(vehicle_json, f, indent=2, ensure_ascii=False)
        except Exception as e:
            error(f"Error while creating the JSON file {vehicle} in folder {os.getcwd()}")
            error(e)
            # sys.exit(1)
        debug("")


def move_results(vehicles):
    """
        Once all parameters files are created, moves for "new_params_mversion" as the last execution result
    """

    os.chdir(f"{BASEPATH}/Tools/autotest/param_metadata")

    for vehicle in vehicles:
        debug(f"Moving created files for {vehicle}")
        try:
            folder = f"{args.destFolder}/{vehicle_new_to_old_name[vehicle]}/"

            # touch the folders
            if not os.path.exists(args.destFolder):
                os.makedirs(args.destFolder)
            if not os.path.exists(folder):
                os.makedirs(folder)

            # Cleaning last run, iff exists
            files_to_delete = [f for f in glob.glob(f"{folder}*")]
            for old_file in files_to_delete:
                os.remove(old_file)

            # Moving files (use shutil.move for cross-device compatibility)
            files_to_move = [f for f in glob.glob(f"parameters-{vehicle}*")]
            for file in files_to_move:
                if "latest" not in file:  # Trying to re-enable toc list on the left bar on the wiki by forcing latest file name.  # noqa: E501
                    shutil.move(file, f"{folder}{file}")
                else:
                    shutil.move(file, f"{folder}parameters.rst")

        except Exception as e:
            error(f"Error while moving result files of vehicle {vehicle} pwd: {os.getcwd()}")
            error(e)
            # sys.exit(1)


def print_versions(commits_to_checkout_and_parse):
    """ Partial results: present all vehicles, versions, and commits selected to generate parameters """
    debug("\n\tList of parameters files to generate:\n")
    for i in commits_to_checkout_and_parse:
        debug(f"{commits_to_checkout_and_parse[i][0]} - {commits_to_checkout_and_parse[i][1]} - {commits_to_checkout_and_parse[i][2]}")  # noqa: E501
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

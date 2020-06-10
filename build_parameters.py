#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    This script aims to provide multiple parameters source files for each vehicle based on the versions available at https://firmware.ardupilot.org

    It is intended to be run on the main wiki server. TO-DO: run locally within the project's Vagrant environment.

    Build notes:

    * Before start:
        * Folder and file management tested only in linux;
        * It is supposed to have the wiki repo in a same level of an Ardupilot repo and two other folders named new_params_mvesion/ and old_params_mversion/

    * First step is go to each vehicle on firmware.ardupilot.org and get all available versions namely as stable/beta/latest.
        * For each version it gets a board and get git_version.txt file;
        * It parsers that to get the version and commit hash;
        * It creates a dictionary with vehicles, versions and commit hashes.
    
    * Second step is use the dict to navigates on a Ardupilot Repo, changing checkouts to desidered hashes.
        * Relies on ArdupilotRepoFolder/Tools/autotest/param_metadata/param_parse.py to generate the parameters files;
        * It renames the anchors for all files, except for latest versions. 

    * Third step: create the json files and move all generated files.

"""

import os
import shutil
import sys
import time
from html.parser import HTMLParser
import urllib.request
import re
import glob
import argparse

parser = argparse.ArgumentParser(description="python3 build_parameters.py [options]")
parser.add_argument("--verbose", dest='verbose', action='store_false', default=True, help="show debugging output")
parser.add_argument("--ardupilotRepoFolder", dest='gitFolder', default="../ardupilot", help="Ardupilot git folder. ")
parser.add_argument("--destination", dest='destFolder', default="../../../../new_params_mversion", help="Parameters*.rst destination folder.")
parser.add_argument('--vehicle', dest='single_vehicle', help="If you just want to copy to one vehicle, you can do this. Otherwise it will work for all vehicles (Copter, Plane, Rover, AntennaTracker, Sub)")
args = parser.parse_args()

error_count = 0



## Parameters
COMMITFILE = "git-version.txt"
BASEURL = "https://firmware.ardupilot.org/"
ALLVEHICLES =  ["AntennaTracker", "Copter" ,  "Plane", "Rover"] 
VEHICLES = ALLVEHICLES

BASEPATH = ""

## Dicts for name replacing
vehicle_new_to_old_name = { # Used because "param_parse.py" args expect old names
    "Rover": "APMrover2",
    "Sub": "ArduSub",
    "Copter":"ArduCopter",
    "Plane":"ArduPlane",
    "AntennaTracker":"AntennaTracker",  # firmware server calls Tracker as AntennaTracker
}

vehicle_old_to_new_name = { # Used because git-version.txt use APMVersion with old names
    "APMrover2":"Rover",
    "ArduRover":"Rover",
    "ArduSub":"Sub",
    "ArduCopter":"Copter",
    "ArduPlane":"Plane",
    "AntennaTracker":"AntennaTracker",  # firmware server calls Tracker/atennatracker as AntennaTracker
}


def debug(str_to_print):
    """Debug output if verbose is set."""
    if args.verbose:
        print("[build_parameters.py] " + str(str_to_print))


def error(str_to_print):
    """Show and count the errors."""
    global error_count
    error_count += 1
    print("[build_parameters.py][error]: " + str(str_to_print))


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


def setup():
    """
    Goes to the work folder, clean it and update.
    """   

    # Test for a single vehicle run
    if args.single_vehicle in ALLVEHICLES:
        global VEHICLES
        VEHICLES = [args.single_vehicle]
        print("[build_parameters.py] Running only for " + str(args.single_vehicle))
    else:
        print("[build_parameters.py] Vehicle %s not recognized, running for all vehicles." % str(args.single_vehicle))

    try:
        ## Goes to ardupilot folder and clean it and update to make sure that is the most recent one.
        debug("Recovering from a previous run...")
        os.chdir(args.gitFolder)
        os.system("git reset --hard HEAD") 
        os.system("git clean -f -d") 
        os.system("git checkout -f master") 
        os.system("git fetch origin master") 
        os.system("git reset --hard origin/master") 
        os.system("git pull") 
        os.system("git submodule update --init --recursive")
        global BASEPATH 
        BASEPATH = os.getcwd()
        check_temp_folders()
    except Exception as e:
        error("ArduPilot Repo folder not found (cd %s failed)" % args.gitFolder)
        error(e)
        sys.exit(1)
    finally:
        debug("\nThe current working directory is %s" % BASEPATH)
    

def fetch_releases(firmware_url, vehicles):
    """
    Select folders with desidered releases for the vehicles

    """

    def fetch_vehicle_subfolders(firmware_url):
        """
        Fetch firmware.ardupilot.org/baseURL all first level folders for a given base URL.

        """
        links = []
        #Define HTML Parser
        class parseText(HTMLParser):
            def handle_starttag(self, tag, attrs):
                if tag != 'a':
                    return
                attr = dict(attrs)
                links.append(attr)
        #Create instance of HTML parser
        lParser = parseText()
        #Feed HTML file into parsers
        try:
            debug("Fetching " + firmware_url)
            lParser.feed(urllib.request.urlopen(firmware_url).read().decode('utf8'))
        except Exception as e:
            error("Folders list download error: " + e)
            sys.exit(1)    
        finally:
            lParser.links = []
            lParser.close()
            return links
    ######################################################################################    

    debug("Cleanning fetched links for wanted folders")
    stableFirmwares = []
    for f in vehicles:
        page_links = fetch_vehicle_subfolders(firmware_url + f)
 
        for l in page_links:    # Non clever way to filter the strings insert by makehtml.py, unwanted folders, and so.
            version_folder = str(l)
 
            if version_folder.find("stable")>0 and not version_folder.endswith("stable"): # If finish with
                stableFirmwares.append(firmware_url[:-1] + version_folder[10:-2])  
            elif version_folder.find("latest") > 0 :
               stableFirmwares.append(firmware_url[:-1] + version_folder[10:-2])  
            elif version_folder.find("beta") > 0:
                stableFirmwares.append(firmware_url[:-1] + version_folder[10:-2])  

    return stableFirmwares # links for the firmwares folders


def get_commit_dict(releases_parsed):
    """
    For informed releases, return a dict git hashs of its build.

    """        
    def get_last_board_folder(url):
        """
        For given URL returns the last folder which should be a board name. 

        """
        links = []
        #Define HTML Parser
        class parseText(HTMLParser):
            def handle_starttag(self, tag, attrs):
                if tag != 'a':
                    return
                attr = dict(attrs)
                links.append(attr)
        #Create instance of HTML parser
        lParser = parseText()
        #Feed HTML file into parsers
        try:
            debug("Fetching " + url)
            lParser.feed(urllib.request.urlopen(url).read().decode('utf8'))
        except Exception as e:
            error("Folders list download error:" + e)
            #sys.exit(1) #comment to make easer debug (temporary)    
        finally:
            lParser.links = []
            lParser.close()
            last_item = links.pop()
            last_folder = last_item['href']
            debug("Returning link of the last board folder (" + last_folder[last_folder.rindex('/')+1:] + ")")
            return last_folder[last_folder.rindex('/')+1:]  # clean the partial link
    ####################################################################################################
            

    def fetch_commit_hash(version_link, board, file):
        """
        For a binnary folder, gets a git hash of its build.

        """
        fetch_link = version_link + '/' + board + '/' + file

        print("Processing link...\t" + fetch_link)
        
        try:
            fecth_response = ""
            with urllib.request.urlopen(fetch_link) as response:
                fecth_response = response.read().decode("utf-8")    
            
            commit_details = fecth_response.split("\n")
            commit_hash = commit_details[0][7:]
            #version =  commit_details[6] the sizes cary
            version = commit_details.pop(-2)

            version_number =  version.split(" ")[2]
            vehicle = version.split(" ")[1]

            regex = re.compile('[@_!#$%^&*()<>?/\|}{~:]') 

            if (regex.search(vehicle) == None):  # there are some non standart names
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
                    
                else:
                    error("Nomenclature exception found in a vehicle name:\t" + vehicle + "\tLink with the exception:\t" + fetch_link)

            if "beta" in fetch_link:
                version_number = "beta-" + version_number

            if "latest" in fetch_link:
                version_number = "latest-" + version_number

            return vehicle, version_number, commit_hash        
        except Exception as e:
            error("An exception occurred: " + file + " DECODE ERROR. Link: " + fetch_link)
            error(e)
            #sys.exit(1) #comment to make easer debug     
            return "error", "error","error"
    ####################################################################################################

    commits_and_codes = {}
    commite_and_codes_cleanned = {}

    for j in range(0,len(releases_parsed)):
        commits_and_codes[j] = fetch_commit_hash(releases_parsed[j], get_last_board_folder(releases_parsed[j]), COMMITFILE)

    for i in commits_and_codes:
        if commits_and_codes[i][0] != 'error':
            commite_and_codes_cleanned[i] = commits_and_codes[i] 
    
    return commite_and_codes_cleanned


def generate_rst_files(commits_to_checkout_and_parse):
    """
    For each git hash it generates its Parameters file.
    
    """

    def replace_anchors(source_file, dest_file, version_tag):
        """
        For each parameter file generate by param_parse.py, it inserts a version tag in anchors to do not make confusing in sphinx toctrees.
        
        """
        file_in = open(source_file, "r")
        file_out = open(dest_file, "w")
        found_original_title = False
        if "latest" not in version_tag:
            file_out.write(':orphan:\n\n')

        for line in file_in:
            if (re.match("(^.. _)(.*):$", line))  and ("latest" not in version_tag):
                file_out.write(line[0:-2] + version_tag + ":\n")  # renames the anchors, but leave latest anchors  "as-is" to maintaim compatibility with all links across the wiki
           
            elif "Complete Parameter List" in line:
                # Adjusting the page title
                out_line = "Complete Parameter List\n=======================\n\n"
                out_line += "\n.. raw:: html\n\n"
                out_line += "   <h2>Full Parameter List of " + version_tag[1:].replace("-"," ") + "</h2>\n\n"  # rename the page identifier to insert the version
                
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
        if  ( 
            "3.2.1" in version or # last stable APM Copte
            "3.4.0" in version or # last stable APM Plane 
            "3.4.6" in version or # Copter
            "2.42" in version or  # last stable APM Rover?
            "2.51" in version or  # last beta APM Rover?
            "0.7.2" in version    # Antennatracker
            ):
            debug("Ignoring APM version:\t" + vehicle + "\t" + version)
            continue

        # Checkout an Commit ID in order to get its parameters
        try:
            debug("Git checkout on " + vehicle + " version " + version + " id " + commit_id )
            os.system("git checkout --force " + commit_id)
            
        except Exception as e:
            error("GIT checkout error: " + e)
            sys.exit(1)
        debug("")

        # Run param_parse.py tool from Autotest set in the desidered commit id
        try:
            os.chdir(BASEPATH + "/Tools/autotest/param_metadata")
            if ('rover' in vehicle.lower()) and ('v3.' not in version.lower()) and ('v4.0' not in version.lower()): # Workaround the vehicle renaming (Rover, APMRover2 ArduRover...)
                os.system("python3 ./param_parse.py --vehicle " + 'Rover')  
            else: # regular case
                os.system("python3 ./param_parse.py --vehicle " + vehicle_new_to_old_name[vehicle])  # option "param_parse.py --format rst" is not available in all commits where param_parse.py is found

            
            # create a filename for new parameters file
            filename = "parameters-" + vehicle 
            if ("beta" in version or "rc" in version): # Plane uses BETA, Copter and Rover uses RCn 
                filename += "-" + version  +".rst"
            elif ("latest" in version ):
                filename += "-" + version  +".rst"
            else:    
                filename += "-stable-" + version  +".rst"

            # Generate new anchors names in files to avoid toctree problems and links in sphinx.
            if os.path.exists("Parameters.rst"):
                replace_anchors("Parameters.rst", filename, filename[10:-4]) 
                os.remove("Parameters.rst")        
                debug("File " + filename + " generated. ")
            else:
                error("Parameters.rst not found to rename to  %s" % filename)  

            os.chdir(BASEPATH)
        except Exception as e:
            error("Error while parsing \"Parameters.rst\" | details:\t" +  vehicle + "\t" + version  + "\t" + commit_id)
            error(e)
            #sys.exit(1)
        debug("")

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
        json_lines.append("\"Click here to change\" : \"\"")

        for filename in parameters_files:
            if ("beta" in filename or "rc" in filename): # Plane uses BETA, Copter and Rover uses RCn 
                json_lines.append(",\"" +  vehicle + " beta " + filename[(len("parameters-" + vehicle + "-beta")+1):-4] + "\" : \"" + filename[:-3] + "html\"" )
            elif ("latest" in filename):
                #json_lines.append(",\"" +  vehicle + " latest " + filename[(len("parameters-" + vehicle + "-latest")+1):-4] + "\" : \"" + filename[:-3] + "html\"" )
                json_lines.append(",\"" +  vehicle + " latest " + filename[(len("parameters-" + vehicle + "-latest")+1):-4] + "\" : \"" + ("parameters.html\"") )   # Trying to re-enable toc list on the left bar on the wiki by forcing latest file name.

            else:    
                json_lines.append(",\"" +  vehicle + " stable " + filename[(len("parameters-" + vehicle + "-stable")+1):-4] + "\" : \"" + filename[:-3] + "html\"" )

        json_lines.append("}")

        # Saves the JSON
        json_filename = "parameters-" + vehicle + ".json"
        try:
            with open(json_filename, 'w') as f:
                for lines in json_lines:
                    f.write("%s\n" % lines)
        except Exception as e:
            error("Error while creating the JSON file " +  vehicle + " in folder " + str(os.getcwd()))
            error(e)
            #sys.exit(1)
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

            # touch the folders
            if not os.path.exists(args.destFolder):
                os.makedirs(args.destFolder)
            if not os.path.exists(folder):
                os.makedirs(folder)

            # Cleanning last run, iff exists
            files_to_delete = [f for f in glob.glob(folder + "*")]
            for old_file in files_to_delete:
                os.remove(old_file)

            # Moving files.
            files_to_move = [f for f in glob.glob("parameters-" + vehicle + "*")]
            for file in files_to_move:
                if ("latest" not in file):      # Trying to re-enable toc list on the left bar on the wiki by forcing latest file name.
                    os.rename(file, folder + file)
                else:
                    os.rename(file, str((folder + "parameters.rst"))) 

        except Exception as e:
            error("Error while moving result files of vehicle " +  vehicle + " pwd: " + str(os.getcwd()))
            error(e)
            #sys.exit(1)


def print_versions(commits_to_checkout_and_parse):
    """ Partial results: present all vechicles, versions and commits selected to generate parameters """
    debug("\n\tList of parameters files to generate:\n")
    for i in commits_to_checkout_and_parse:
        debug(commits_to_checkout_and_parse[i][0] + ' - ' + commits_to_checkout_and_parse[i][1] + ' - ' + commits_to_checkout_and_parse[i][2])
    debug("")





# Step 1 - Select the versions for generate parameters
setup()                                                             # Reset the Ardupilot folder/repo
feteched_releases = fetch_releases(BASEURL, VEHICLES)               # All folders/releases.
commits_to_checkout_and_parse = get_commit_dict(feteched_releases)  # Parse names, and git hashes.
print_versions(commits_to_checkout_and_parse)                       # Present work dict.

# Step 2 - Generates them in ArdupilotRepoFolder/Tools/autotest/param_metadata
generate_rst_files(commits_to_checkout_and_parse)

# Step 3 - Generates a JOSN file for each vehicle and mode files to folder new_params_mversion
generate_json(VEHICLES)
move_results(VEHICLES)

#sys.exit(error_count)

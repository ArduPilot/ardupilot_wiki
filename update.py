#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This script updates and rebuilds wiki sources from Github and from parameters on the test server.

It is intended to be run on the main wiki server or
locally within the project's Vagrant environment.

Build notes:

* First step is always a fetch and pull from git (master).
  * Default is just a normal fetch and pull from master
  * If the --clean option is "True" then git will reset to head

* Common topics are copied from /common/source/docs.
  * Topics are copied based on information in the copywiki shortcode. For example a topic marked as below
    would only be copied to copter and plane wikis:
    [copywiki destination="copter,plane"]
  * Topics that don't have a [copywiki] will be copied to wikis the DEFAULT_COPY_WIKIS list
  * Copied topics are stripped of the 'copywiki' shortcode in the destination.
  * Copied topics are stripped of any content not marked for the target wiki using the "site" shortcode:
    [site wiki="plane,rover"]conditional content[/site]

Parameters files are fetched from autotest using a Wget
"""
from __future__ import print_function, unicode_literals

import argparse
import errno
import re
import os
from codecs import open
import subprocess
import multiprocessing
import shutil
import glob
import filecmp
import time
import sys
import hashlib
from datetime import datetime
import distutils
from distutils import dir_util

DEFAULT_COPY_WIKIS =['copter', 'plane', 'rover']
ALL_WIKIS =['copter', 'plane', 'rover','antennatracker','dev','planner','planner2','ardupilot', 'mavproxy']
COMMON_DIR='common'

#GIT_REPO = ''

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Copy Common Files as needed, stripping out non-relevant wiki content')
parser.add_argument('--site', help="If you just want to copy to one site, you can do this. Otherwise will be copied.")
parser.add_argument('--clean', action='store_true', help="Does a very clean build - resets git to master head (and TBD cleans up any duplicates in the output).")
parser.add_argument('--cached-parameter-files', action='store_true', help="Do not re-download parameter files")
parser.add_argument('--parallel', type=int, help="limit parallel builds, -1 for unlimited", default=1)
parser.add_argument('--destdir', default="/var/sites/wiki/web", help="Destination directory for compiled docs")
parser.add_argument('--enablebackups', action='store_true', default=False, help="Enable several backups up to const N_BACKUPS_RETAIN in --backupdestdir folder")
parser.add_argument('--backupdestdir', default="/var/sites/wiki-backup/web", help="Destination directory for compiled docs")
parser.add_argument('--paramversioning', action='store_true', default=False, help="Build multiple parameters pages for each vehicle based on its firmware repo.")
parser.add_argument('--verbose', dest='verbose', action='store_false', default=True, help="show debugging output")
args = parser.parse_args()
#print(args.site)
#print(args.clean)

PARAMETER_SITE={'rover':'APMrover2', 'copter':'ArduCopter','plane':'ArduPlane','antennatracker':'AntennaTracker' }
LOGMESSAGE_SITE={'rover':'Rover', 'copter':'Copter','plane':'Plane','antennatracker':'Tracker' }
error_count = 0
N_BACKUPS_RETAIN = 10


def debug(str_to_print):
    """Debug output if verbose is set."""
    if args.verbose:
        print("[update.py] " + str_to_print)


def error(str_to_print):
    """Show and count the errors."""
    global error_count
    error_count += 1
    print("[update.py][error]: " + str(str_to_print))


def fetchparameters(site=args.site):
    """
    Fetches the parameters for all the sites from the test server and
    copies them to the correct location.

    This is always run as part of a build (i.e. no checking to see if parameters have changed.)
    """
    # remove any parameters files in root
    try:
        subprocess.check_call(["rm", 'Parameters.rst'])
    except:
        pass

    for key, value in PARAMETER_SITE.items():
        fetchurl='https://autotest.ardupilot.org/Parameters/%s/Parameters.rst' % value
        targetfile='./%s/source/docs/parameters.rst' % key
        if args.cached_parameter_files:
            if not os.path.exists(targetfile):
                raise(Exception("Asked to use cached parameter files, but (%s) does not exist" % (targetfile,)))
            continue
        if site==key or site==None:
            subprocess.check_call(["wget", fetchurl])
            try: #Remove target file if it exists
                subprocess.check_call(["rm", targetfile])
            except:
                pass
            #copy in new file
            subprocess.check_call(["mv", 'Parameters.rst', targetfile])


def fetchlogmessages(site=args.site):
    """
    Fetches the parameters for all the sites from the autotest server and
    copies them to the correct location.

    This is always run as part of a build (i.e. no checking to see if logmessages have changed.)
    """
    for key, value in LOGMESSAGE_SITE.items():
        fetchurl='https://autotest.ardupilot.org/LogMessages/%s/LogMessages.rst' % value
        targetfile='./%s/source/docs/logmessages.rst' % key
        if args.cached_parameter_files:
            if not os.path.exists(targetfile):
                raise(Exception("Asked to use cached parameter files, but (%s) does not exist" % (targetfile,)))
            continue
        if site==key or site==None:
            subprocess.check_call(["wget", fetchurl])
            try: #Remove target file if it exists
                subprocess.check_call(["rm", targetfile])
            except:
                pass
            #copy in new file
            subprocess.check_call(["mv", 'LogMessages.rst', targetfile])


def build_one(wiki):
    '''build one wiki'''
    print('make and clean: %s' % wiki)
    subprocess.check_call(["nice", "make", "clean"], cwd=wiki)
    subprocess.check_call(["nice", "make", "html"], cwd=wiki)


def sphinx_make(site):
    """
    Calls 'make html' to build each site
    """
    done = set()
    wikis = set(ALL_WIKIS[:])
    num_procs = 0
    procs = []
    global error_count

    while len(done) != len(wikis):
        wiki = list(wikis.difference(done))[0]
        done.add(wiki)
        if site=='common':
            continue
        if not site==None and not site==wiki:
            continue
        p = multiprocessing.Process(target=build_one, args=(wiki,))
        p.start()
        procs.append(p)
        while args.parallel != -1 and len(procs) >= args.parallel:
            for p in procs:
                if p.exitcode is not None:
                    p.join()
                    procs.remove(p)
                    if p.exitcode != 0:
                        error_count += 1
            time.sleep(0.1)
    while len(procs) > 0:
        for p in procs[:]:
            if p.exitcode is not None:
                p.join()
                procs.remove(p)
                if p.exitcode != 0:
                    error_count += 1
        time.sleep(0.1)


def copy_build(site):
    """
    Copies each site into the target location
    """
    global error_count
    for wiki in ALL_WIKIS:
        if site=='common':
            continue
        if not site==None and not site==wiki:
            continue
        print('copy: %s' % wiki)
        targetdir = os.path.join(args.destdir, wiki)

        print("DEBUG: Creating backup")
        olddir = os.path.join(args.destdir, 'old')
        print('DEBUG: recreating %s' % olddir )
        if os.path.exists(olddir):
            shutil.rmtree(olddir)
        os.mkdir(olddir)
        if os.path.exists(targetdir):
            print('DEBUG: moving %s into %s' % (targetdir,olddir) )
            shutil.move(targetdir, olddir)

        # copy new dir to targetdir
        #print("DEBUG: targetdir: %s" % targetdir)
        #sourcedir='./%s/build/html/*' % wiki
        sourcedir='./%s/build/html/' % wiki
        #print("DEBUG: sourcedir: %s" % sourcedir)
        #print('DEBUG: mv %s %s' % (sourcedir, args.destdir) )

        html_moved_dir = os.path.join(args.destdir, 'html')
        try:
            subprocess.check_call(['mv', sourcedir, html_moved_dir])
            #Rename move! (single move to html/* failed)
            subprocess.check_call(['mv', html_moved_dir ,targetdir])
            print("DEBUG: Moved to %s" % targetdir)
        except:
            print("DEBUG: FAIL moving output to %s" % targetdir)
            error_count += 1


        # delete the old directory
        print('DEBUG: removing %s' % olddir )
        shutil.rmtree(olddir)


def copy_and_keep_build(site):
    """
    Copies each site into the target location and keep last "n" builds as backups
    """
    global error_count
    for wiki in ALL_WIKIS:
        if site=='common':
            continue
        if site is not None and site != wiki:
            continue
        debug('coping: %s' % wiki)
        targetdir = os.path.join(args.destdir, wiki)
        distutils.dir_util.mkpath(targetdir)

        if os.path.exists(targetdir):
            olddir = os.path.join(args.backupdestdir, str(building_time + '-wiki-bkp'), str(wiki) )
            debug('Checking %s' % olddir )
            distutils.dir_util.mkpath(olddir)
            debug('Moving %s into %s' % (targetdir,olddir))
            shutil.move(targetdir, olddir)

            sourcedir = os.path.join(os.path.abspath(os.getcwd()), str(wiki), 'build', 'html')
            html_moved_dir = os.path.join(args.destdir, 'html')
            try:
                #subprocess.check_call(['mv', sourcedir, html_moved_dir])
                shutil.move(sourcedir, html_moved_dir)
                # Rename move! (single move to html/* failed)
                #subprocess.check_call(['mv', html_moved_dir ,targetdir])
                shutil.move(html_moved_dir, targetdir)
                debug("Moved to %s" % targetdir)
            except:
                error("FAIL moving output to %s" % targetdir)
            finally:
                debug("Creating a backup in %s" % olddir)
                # subprocess.check_call(['cp', '-r', targetdir ,olddir])
                distutils.dir_util.copy_tree(targetdir, olddir, preserve_symlinks=0)
        else:
            error("FAIL when looking for folder %s" % targetdir)


def delete_old_wiki_backups(folder, n_to_keep):
    try:
        debug('Checking number of number of backups in folder %s' % folder)
        backup_folders = glob.glob(folder  + "/*-wiki-bkp/")
        backup_folders.sort()
        if len(backup_folders) > n_to_keep:
            for i in range(0, len(backup_folders) - n_to_keep):
                if '-wiki-bkp' in str(backup_folders[i]):
                    debug('Deleting folder %s' % str(backup_folders[i]))
                    shutil.rmtree(str(backup_folders[i]))
                else:
                    debug('Ignoring folder %s because it does not look like a auto generated wiki backup folder' % str(backup_folders[i]))
        else:
            debug('No old backups to delete in %s' % folder)
    except Exception as e:
        error('Error on deleting some previous wiki backup folders: %s' % e)


def generate_copy_dict(start_dir=COMMON_DIR):
    """
    This creates a dict which indexes copy targets for all common docs.
    """

    #Clean existing common topics (easiest way to guarantee old ones are removed)
    #Cost is that these will have to be rebuilt even if not changed
    import glob
    for wiki in ALL_WIKIS:
        files = glob.glob('%s/source/docs/common-*.rst' % wiki)
        for f in files:
            print('remove: %s' % f)
            os.remove(f)

    #Create destination folders that might be needed (if don't exist)
    for wiki in ALL_WIKIS:
        try:
            os.mkdir(wiki)
        except:
            pass

        try:
            os.mkdir('%s/source' % wiki)
        except:
            pass

        try:
            os.mkdir('%s/source/docs' % wiki)
        except:
            pass

        try:
            os.mkdir('%s/source/_static' % wiki)
        except:
            pass


    for root, dirs, files in os.walk(start_dir):
        for file in files:
            if file.endswith(".rst"):
                print("FILE: %s" % file)
                source_file_path=os.path.join(root, file)
                source_file = open(source_file_path, 'r', 'utf-8')
                source_content=source_file.read()
                source_file.close()
                targets=get_copy_targets(source_content)
                #print(targets)
                for wiki in targets:
                    #print("CopyTarget: %s" % wiki)
                    content = strip_content(source_content, wiki)
                    targetfile='%s/source/docs/%s' % (wiki,file)
                    print(targetfile)
                    destination_file = open(targetfile, 'w', 'utf-8')
                    destination_file.write(content)
                    destination_file.close()
            elif file.endswith(".css"):
                for wiki in ALL_WIKIS:
                    shutil.copy2(os.path.join(root, file), '%s/source/_static/' % wiki)
            elif file.endswith(".js"):
                source_file_path=os.path.join(root, file)
                source_file = open(source_file_path, 'r', 'utf-8')
                source_content=source_file.read()
                source_file.close()
                targets=get_copy_targets(source_content)
                #print("JS: " + str(targets))
                for wiki in targets:
                    content = strip_content(source_content, wiki)
                    targetfile='%s/source/_static/%s' % (wiki,file)
                    print(targetfile)
                    destination_file = open(targetfile, 'w', 'utf-8')
                    destination_file.write(content)
                    destination_file.close()


def get_copy_targets(content):
    p = re.compile(r'\[copywiki.*?destination\=\"(.*?)\".*?\]',flags=re.S)
    m = p.search(content)
    targetset=set()
    if m:
        targets=m.group(1).split(',')
        for item in targets:
          targetset.add(item.strip())
    else:
        targetset=set(DEFAULT_COPY_WIKIS)
    return targetset


def strip_content(content, site):
    """
    Strips the copywiki shortcode. Removes content for other sites and the [site] shortcode itself.
    """

    def fix_copywiki_shortcode(matchobj):
        """
        Strip the copywiki shortcode if found (just return "nothing" to result of re)
        """
        #logmatch_code(matchobj, 'STRIP')
        #print("STRIPPED")
        return ''

    #Remove the copywiki from content
    newText=re.sub(r'\[copywiki.*?\]', fix_copywiki_shortcode, content, flags= re.M)


    def fix_site_shortcode(matchobj):
        #logmatch_code(matchobj, 'SITESC_')
        sitelist=matchobj.group(1)
        #print("SITES_BLOCK: %s" % sitelist)
        if site not in sitelist:
            #print("NOT")
            return ''
        else:
            #print("YES")
            return matchobj.group(2)
    #Remove the site shortcode from content
    newText=re.sub(r'\[site\s.*?wiki\=\"(.*?)\".*?\](.*?)\[\/site\]', fix_site_shortcode, newText, flags= re.S)

    return newText


def logmatch_code(matchobj, prefix):

    try:
        print("%s m0: %s" % (prefix,matchobj.group(0)) )
    except:
        print("%s: except m0" % prefix)


    try:
        print("%s m1: %s" % (prefix,matchobj.group(1)))
    except:
        print("%s: except m1" % prefix)

    try:
        print("%s m2: %s" % (prefix,matchobj.group(2)))
    except:
        print("%s: except m1" % prefix)

    try:
        print("%s m3: %s" % (prefix,matchobj.group(3)))
    except:
        print("%s: except m3" % prefix)

    try:
        print("%s m4: %s" % (prefix,matchobj.group(4)))
    except:
        print("%s: except m4" % prefix)
    try:
        print("%s m5: %s" % (prefix,matchobj.group(5)))
    except:
        print("%s: except m5" % prefix)
    try:
        print("%s m6: %s" % (prefix,matchobj.group(6)))
    except:
        print("%s: except m6" % prefix)
    try:
        print("%s m7: %s" % (prefix,matchobj.group(7)))
    except:
        print("%s: except 7" % prefix)
    try:
        print("%s m8: %s" % (prefix,matchobj.group(8)))
    except:
        print("%s: except m8" % prefix)


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

    return(digests[0] == digests[1])


def fetch_versioned_parameters(site=args.site):
    """
    It relies on "build_parameters.py" be executed before the "update.py"     
    
    Once the generated files are on ../new_params_mversion it tut all parameters and JSON files in their destinations.
    
    """

    for key, value in PARAMETER_SITE.items():
        
        if site==key or site==None:
            # Remove old param single file
            single_param_file='./%s/source/docs/parameters.rst' % key
            debug("Erasing " + single_param_file)
            try: 
                subprocess.check_call(["rm", single_param_file])
            except Exception as e:
                error(e)
                pass
             
            # Remove old versioned param files
            if 'antennatracker' in key.lower(): # To main the original script approach instead of the build_parameters.py approach.
                old_parameters_mask = os.getcwd() + '/%s/source/docs/parameters-%s-' % ("AntennaTracker","AntennaTracker")
            else:
                old_parameters_mask = os.getcwd() + '/%s/source/docs/parameters-%s-' % (key,key.title()) 
            try:  
                old_parameters_files = [f for f in glob.glob(old_parameters_mask + "*.rst")]
                for filename in old_parameters_files:
                    debug("Erasing rst " + filename)
                    os.remove(filename)    
            except Exception as e:
                error(e)
                pass

            # Remove old json file
            if 'antennatracker' in key.lower(): # To main the original script approach instead of the build_parameters.py approach.
                target_json_file='./%s/source/_static/parameters-%s.json' % ("AntennaTracker","AntennaTracker")
            else:
                target_json_file='./%s/source/_static/parameters-%s.json' % (value,key.title())
            debug("Erasing json " + target_json_file)
            try:
                subprocess.check_call(["rm", target_json_file])
            except Exception as e:
                error(e)
                pass

            # Moves the updated JSON file  
            if 'antennatracker' in key.lower():  # To main the original script approach instead of the build_parameters.py approach. 
                vehicle_json_file = os.getcwd() + '/../new_params_mversion/%s/parameters-%s.json' % ("AntennaTracker","AntennaTracker")
            else: 
                vehicle_json_file = os.getcwd() + '/../new_params_mversion/%s/parameters-%s.json' % (value,key.title())   
            new_file = key + "/source/_static/" + vehicle_json_file[str(vehicle_json_file).rfind("/")+1:] 
            try:  
                debug("Moving " + vehicle_json_file)
                #os.rename(vehicle_json_file, new_file)
                shutil.copy2(vehicle_json_file, new_file)
            except Exception as e:
                error(e)
                pass

            # Copy all parameter files to vehicle folder IFF it is new
            try:
                new_parameters_folder = os.getcwd() + '/../new_params_mversion/%s/' % value
                new_parameters_files = [f for f in glob.glob(new_parameters_folder + "*.rst")]
            except Exception as e:
                error(e)
                pass
            for filename in new_parameters_files:
                # Check possible cached version
                try:
                    new_file = key + "/source/docs/" + filename[str(filename).rfind("/")+1:]
                    if os.path.isfile(filename.replace("new_params_mversion","old_params_mversion")): # The cached file exists?
                        
                        # Temporary debug messages to help with cache tasks.
                        debug("Check cache: %s against %s" % (filename, filename.replace("new_params_mversion","old_params_mversion")))
                        debug("Check cache with filecmp.cmp: %s" % filecmp.cmp(filename, filename.replace("new_params_mversion","old_params_mversion")))
                        debug("Check cache with sha256: %s" % is_the_same_file(filename, filename.replace("new_params_mversion","old_params_mversion")))

                        if ("parameters.rst" in filename) or (not filecmp.cmp(filename, filename.replace("new_params_mversion","old_params_mversion"))):    # It is different?  OR is this one the latest. | Latest file must be built everytime in order to enable Sphinx create the correct references across the wiki.
                            debug("Overwriting %s to %s" % (filename, new_file))                               
                            shutil.copy2(filename, new_file)
                        else:
                            debug("It will reuse the last build of " + new_file)                               
                    else:   # If not cached, build it anyway.
                        debug("Creating %s to %s" % (filename, new_file))                               
                        shutil.copy2(filename, new_file)

                except Exception as e:
                    error(e)
                    pass


def create_latest_parameter_redirect(default_param_file, vehicle):
    """
    For a given vehicle create a file called parameters.rst that redirects to the latest parameters file.(Create to maintaim retro compatibility.)
    
    """
    out_line = "======================\nParameters List (Full)(\n======================\n"
    out_line += "\n.. raw:: html\n\n"
    out_line += "   <script>location.replace(\"" + default_param_file[:-3] + "html" + "\")</script>"
    out_line += "\n\n"

    filename = vehicle + "/source/docs/parameters.rst"
    with open(filename, "w") as text_file:
        text_file.write(out_line)
    
    debug("Created html automatic redirection from parameters.html to %shtml" % default_param_file[:-3])


def cache_parameters_files(site=args.site):
    """
    For each vechile: put new_params_mversion/ content in old_params_mversion/ folders and .html built files as well.
    
    """
    for key, value in PARAMETER_SITE.items():
        if site==key or site==None:
            try:
                old_parameters_folder = os.getcwd() + '/../old_params_mversion/%s/' % value
                old_parameters_files = [f for f in glob.glob(old_parameters_folder + "*.*")]
                for file in old_parameters_files:
                    debug("Removing %s" % file)
                    os.remove(file)

                new_parameters_folder = os.getcwd() + '/../new_params_mversion/%s/' % value
                new_parameters_files = [f for f in glob.glob(new_parameters_folder + "parameters-*.rst")]
                for filename in new_parameters_files:
                    debug("Copying %s to %s" % (filename, old_parameters_folder))
                    shutil.copy2(filename, old_parameters_folder)

                built_folder = os.getcwd() + "/" + key + "/build/html/docs/" 
                built_parameters_files = [f for f in glob.glob(built_folder + "parameters-*.html")]
                for built in built_parameters_files:
                    debug("Copying %s to %s" % (built, old_parameters_folder))
                    shutil.copy2(built, old_parameters_folder)

            except Exception as e:
                error(e)
                pass


def put_cached_parameters_files_in_sites(site=args.site):
    """
    For each vechile: put built .html files in site folder
    
    """
    for key, value in PARAMETER_SITE.items():
        if site==key or site==None:
            try:
                built_folder = os.getcwd() + '/../old_params_mversion/%s/' % value
                built_parameters_files = [f for f in glob.glob(built_folder + "parameters-*.html")]
                vehicle_folder = os.getcwd() + "/" + key + "/build/html/docs/" 
                debug("Site %s getting previously built files from %s" % (site,built_folder))
                for built in built_parameters_files:
                    if ("latest" not in built):  # latest parameters files must be built every time
                        debug("Reusing built %s in %s " % (built, vehicle_folder))
                        shutil.copy(built, vehicle_folder)
            except Exception as e:
                error(e)
                pass


###############################################################################################################

now = datetime.now() 
building_time = now.strftime("%Y-%m-%d-%H-%M-%S")

if args.paramversioning:                
    fetch_versioned_parameters(args.site)   # Parameters for all versions availble on firmware.ardupilot.org    
else:
    fetchparameters(args.site)              # Single parameters file. Just present the latest parameters.

fetchlogmessages(args.site)                 # Fetch most recent LogMessage metadata from autotest

generate_copy_dict()
sphinx_make(args.site)

if args.paramversioning:                
    put_cached_parameters_files_in_sites(args.site)
    cache_parameters_files(args.site)


if args.enablebackups:                
    copy_and_keep_build(args.site)
    delete_old_wiki_backups(args.backupdestdir, N_BACKUPS_RETAIN)
else:
    copy_build(args.site)




# To navigate locally and view versioning script for parameters working is necessary run Chrome as "chrome --allow-file-access-from-files". Otherwise it will appear empty locally and working once is on the server.

if error_count > 0:
    print("%u errors during Wiki build" % (error_count,))
    sys.exit(1)

sys.exit(0)

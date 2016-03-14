#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This program updates and rebuilds wiki sources from Github. 

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

"""
from __future__ import print_function, unicode_literals

import argparse
import re
import os
from codecs import open
import subprocess


DEFAULT_COPY_WIKIS =['copter', 'plane', 'rover']
ALL_WIKIS =['copter', 'plane', 'rover','antennatracker','dev','planner','planner2']
COMMON_DIR='common'
COPY_TARGET_DIR_BASE='/var/sites/new/web/wiki/'

#GIT_REPO = ''

#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Copy Common Files as needed, stripping out non-relevant wiki content')
parser.add_argument('--site', help="If you just want to copy to one site, you can do this. Otherwise will be copied.")
parser.add_argument('--clean', default='False', help="Does a very clean build - resets git to master head (and TBD cleans up any duplicates in the output).")
args = parser.parse_args()
#print(args.site)
#print(args.clean)


def refresh_from_git(clean):
    """
    Refreshes wiki sources from git
    """
    subprocess.check_call(["git", "fetch"])
    if not clean=='True':
        print("Git pull from master ...")
        subprocess.check_call(["git", "pull", "origin", "master"])
    else:
        print("Git hard reset from master ...")
        #subprocess.check_call(["git", "reset", "--hard", "HEAD")
        pass
        
def sphinx_make(site):
    """
    Calls 'make html' to build each site
    """
    
    for wiki in ALL_WIKIS:
        if site=='common':
            continue
        if not site==None and not site==wiki:
            continue
        print('make and clean: %s' % wiki)
        subprocess.check_call(["make", "-C", wiki ,"clean"])
        subprocess.check_call(["make", "-C", wiki ,"html"])
            


def copy_build(site):
    """
    Copies each site into the target location
    """
    for wiki in ALL_WIKIS:
        if site=='common':
            continue
        if not site==None and not site==wiki:
            continue
        print('copy: %s' % wiki)
        targetdir=COPY_TARGET_DIR_BASE+wiki
        # copy target directory to "old" folder
        olddir=COPY_TARGET_DIR_BASE+'old'
        try:
            subprocess.check_call(['mkdir', olddir])
        except:
            pass
        #print('DEBUG: mv %s %s' % (targetdir,olddir) )
        try:
            subprocess.check_call(['mv', targetdir, olddir])
            #print("DEBUG: Yes - moved to olddir")
        except:
            #print("DEBUG: No move to olddir")
            pass

        # copy new dir to targetdir
        #print("DEBUG: targetdir: %s" % targetdir)
        #sourcedir='./%s/build/html/*' % wiki
        sourcedir='./%s/build/html/' % wiki
        #print("DEBUG: sourcedir: %s" % sourcedir)
        #print('DEBUG: mv %s %s' % (sourcedir, COPY_TARGET_DIR_BASE) )

        html_moved_dir = COPY_TARGET_DIR_BASE+'html'
        try:
            subprocess.check_call(['mv', sourcedir, html_moved_dir])
            #Rename move! (single move to html/* failed)
            subprocess.check_call(['mv', html_moved_dir ,targetdir])
            print("Yes - moved to copied to good output location")
        except:
            print("no copy of new build")


    # delete the old directory
    print('DEBUG: rm -fi %s' % olddir )
    try:
        subprocess.check_call(["rm", "-rf", olddir])
        print("Yes - deleted olddir")
    except:
        print("no delete of olddir")
            
            
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
                
            
def get_copy_targets(content):
    p = re.compile(r'\[copywiki.*?destination\=\"(.*?)\".*?\]',flags=re.S)
    m = p.search(content)
    targetset=set()
    if m:
        #print(m.group(0))
        #print(m.group(1))
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



#refresh_from_git(clean=args.clean)
generate_copy_dict()
sphinx_make(args.site)
copy_build(args.site)


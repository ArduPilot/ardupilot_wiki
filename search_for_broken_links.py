#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    This program searches for possible 404 erros in local HTML files.
    
    It gets links from every .html files in subfolders and test its links for errors. 
  
    It still a draft... TO-DO: would be better parse only .rst files.

"""

import os
from fnmatch import fnmatch
from optparse import OptionParser
import pandas as pd
from html.parser import HTMLParser
import urllib.request
import requests
import tqdm     # fancy progress bar
import time
import re
# TO-DO from multiprocessing import Pool


parser = OptionParser("python3 search_for_broken_links.py [options]")
parser.add_option("-v", "--verbose", dest='verbose', action='store_true', default=False, help="show debugging output")
# TO-DO parser.add_option("-p", "--parallel", dest='parallel', action='store_true', default=False, help="do a parallel processing")
(opts, args) = parser.parse_args()


FILE_TYPE = "*.html" 
root = "."

error_count = 0
n_found_links = 0
n_selected_links = 0
n_checked_links = 0
n_possible_errors = 0

def debug(str_to_print):
    """Debug output if verbose is set."""
    if opts.verbose:
        print(str_to_print)

def error(str_to_print):
    """Show and count the errors."""
    global error_count
    error_count += 1
    print(str_to_print)


def create_list_of_files(initial_path, pattern):
    """
    Fetch all files based on provided path and pattern

    """    
    list_of_files = []
    for path, subdirs, files in os.walk(initial_path):
        for name in files:
            if fnmatch(name, pattern):
                list_of_files.append(os.path.join(path, name))

    if opts.verbose:
        for path in list_of_files:
            print(path)

    return list_of_files

def fetch_links(uri):
        """
        Fetch links from a file

        """
        links = []
        #Define HTML Parser
        class parseText(HTMLParser):
            def handle_starttag(self, tag, attrs):
                if tag != 'a':
                    return
                attr = dict(attrs)
                links.append(attr)
                global n_found_links 
                n_found_links += 1
        #Create instance of HTML parser
        lParser = parseText()
        #Feed HTML file into parsers
        try:
            debug("Fetching for links on " + uri)
            with open(uri, 'r', encoding="utf8") as file:
                lParser.feed(file.read())
        except:
            error("An exception occurred: search from links on the address:" + uri)
            #sys.exit(1)    
        finally:
            lParser.links = []
            lParser.close()
            return links

# def fetch_links_v2(file):
#     """
#     Fetch links from a file

#     """
#     links = []
    
#     rx = re.compile(r'^(?P<email>[^|\n]+)', re.MULTILINE)
#     with open(file, 'r', encoding="utf8") as text:
#         raw_data = text.read()
#         emails = [match.group('email') for match in rx.finditer(raw_data)]
#         print emails

#     return links
    

def filter_for_external_links(links_to_check):
    """
    Filter links for no Ardupilot Wiki internal references

    """
    external_links = []
    for links in links_to_check:
        canditate_link = links.popitem()[-1] # Looks messy: Gets the last field from the dict of each array position
        if (
            'http' in canditate_link and                                                # remove relative links
            'ardupilot.org' not in canditate_link and                                  # remove internal resources such as the firmware links and shop
            'github.com/ArduPilot/ardupilot_wiki' not in canditate_link and             # remove links to edit or open issues
            'creativecommons.org/licenses/by-sa/3.0/' not in canditate_link and         # theme
            'www.jDrones.com/' not in canditate_link                                    # theme 
        ):
            debug("Candidate link: " + canditate_link)
            external_links.append(canditate_link)
            global n_selected_links 
            n_selected_links += 1

    return external_links


def get_links(files):
    """
    Iteract over all files and links to generate a dictionay with all links

    TO-DO remove duplicated links from "common folder"

    """
    all_links = {}

    for file in tqdm.tqdm(files, desc="Progress: ", ncols=80):
        links_from_a_file = fetch_links(file)
        cleanned_links = filter_for_external_links(links_from_a_file)
        for link in cleanned_links:
            all_links.update({file:link})
    
    return all_links


def check_link_errors(pages_and_links):

    links_with_errors = {}
    for page, link in tqdm.tqdm(pages_and_links.items(),  desc="Progress: ", ncols=80):
        debug("Checking " + link)
        try:
            http_return = requests.head(link, timeout=10).status_code
        except requests.exceptions.ConnectionError:
            http_return = 404
        except requests.exceptions.Timeout:
            error("Timeout occurred fetching " + link)
        except Exception as e:
           error(e)
        finally:
            if http_return >= 400: # 3xx codes for redirection, 4xx for Client errors and 5xx for Server errors
                links_with_errors.update({page:link})
                debug("File " + page + " has a possible broken link (code " + str(http_return) + "): " + link  )

        global n_checked_links 
        n_checked_links += 1

    return links_with_errors



print("Looking for local files...")
files_to_look_for_links = create_list_of_files(root, FILE_TYPE)
print("Found " + str(len(files_to_look_for_links)) + " files to process.")
print()

print("Getting links from local files... ")
external_links_to_check = get_links(files_to_look_for_links)
print("Found a total of " + str(n_found_links) + " links.")
print("Picked " + str(n_selected_links) + " links to fetch.")
print()


print("Getting each remote link at time... (it could take a while because there are 10 seconds of timeout for link to crawl)")
print("Is necessary to check the links manually to avoid \"false positive\" alerts. (Note that errors on common files just need to be correct once)") 
print()
external_probaby_broken = check_link_errors(external_links_to_check)

for file, link in external_probaby_broken.items():
    print("File " + file + " has a possible broken link: " + link  )

print()
print("Found " + str(n_checked_links) + " links to check manually.")
print("Is necessary to check the links manually to avoid \"false positive\" alerts. (Note that errors on common files just need to be correct once)") 
print()

    

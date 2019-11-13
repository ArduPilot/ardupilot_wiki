#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
    This program searches for possible 404 erros in local HTML files.
    
    It gets links from every .html files in subfolders and test its links for errors. 
  
    It still a draft... TO-DO: improve false positivies and parallel processing.

"""

import os
from fnmatch import fnmatch
from optparse import OptionParser
import pandas as pd
from html.parser import HTMLParser
import urllib.request
import requests
import tqdm     
import time
import re

parser = OptionParser("python3 search_for_broken_links.py [options]")
parser.add_option("-v", "--verbose", dest='verbose', action='store_true', default=False, help="show debugging output")
parser.add_option("-b", "--built", dest='file_type_rst', action='store_false', default=False, help="parse an already built wiki, looking .HTML files")
parser.add_option("-s", "--source", dest='file_type_rst', action='store_true', default=True, help="parse the source of a wiki, looking .RST files")
# TO-DO parser.add_option("-p", "--parallel", dest='parallel', action='store_true', default=False, help="do a parallel url fetching")
(opts, args) = parser.parse_args()


FILE_TYPE_HTML = "*.html" 
FILE_TYPE_RST = "*.rst" 
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

def create_list_of_files(initial_path):
    """
    Fetch all files based on provided path and pattern

    """    
    if opts.file_type_rst:
        pattern = FILE_TYPE_RST
    else:
        pattern = FILE_TYPE_HTML

    list_of_files = []
    for path, subdirs, files in os.walk(initial_path):
        for name in files:
            if fnmatch(name, pattern):
                list_of_files.append(os.path.join(path, name))

    if opts.verbose:
        for path in list_of_files:
            print(path)

    return list_of_files

def fetch_links_on_html(uri):
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
    finally:
        lParser.links = []
        lParser.close()
    
    links_as_strings = []
    for item in links:
        links_as_strings.append(item.popitem()[-1])

    return links_as_strings

def fetch_links_on_rst(file_to_check):
    """
    Fetch links from a file

    """
    links = []

    regex = re.compile("\<http(.*?)\>`__")
    with open(file_to_check, 'r', encoding="utf8") as file:
        for line in file:
            result = regex.findall(line)
            if len(result)>0:
                for item in result:
                    debug("File: " + file_to_check + "  Found a link: http" + str(item))
                    links.append("http" + str(item))
                    global n_found_links 
                    n_found_links += 1
 
    return links
    
def filter_for_external_links(links_to_check):
    """
    Filter links for no Ardupilot Wiki internal references

    """
    external_links = []
    for links in links_to_check:
        canditate_link = links 
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
        if opts.file_type_rst:
            links_from_a_file = fetch_links_on_rst(file)
        else:
            links_from_a_file = fetch_links_on_html(file)
        cleanned_links = filter_for_external_links(links_from_a_file)
        for link in cleanned_links:
            all_links.update({file:link})

    return all_links

def check_link_errors(pages_and_links):

    links_with_errors = {}
    for page, link in tqdm.tqdm(pages_and_links.items(),  desc="Progress: ", ncols=80):
        debug("Checking " + link)
        try:
            headers_to_avoid_filters = {'User-Agent': 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_10_1) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/39.0.2171.95 Safari/537.36'}
            http_return = requests.get(link, timeout=10, headers=headers_to_avoid_filters).status_code
        except requests.exceptions.ConnectionError:
            http_return = 404                           # Offline as page not found
        except requests.exceptions.Timeout:
            print()
            error("Timeout occurred fetching " + link)
        except Exception as e:
           print()
           error(e)                                     # Did not happen so far
        finally:
            if http_return >= 400:                      # 3xx codes for redirection, 4xx for Client errors and 5xx for Server errors
                links_with_errors.update({page:link})
                debug("File " + page + " has a possible broken link (code " + str(http_return) + "): " + link  )

        global n_checked_links 
        n_checked_links += 1

    return links_with_errors

print("Looking for local files...")
files_to_look_for_links = create_list_of_files(root)
print("Found " + str(len(files_to_look_for_links)) + " files to proccess.")
print()

print("Getting links from local files... ")
external_links_to_check = get_links(files_to_look_for_links)
print()
print("Found a total of " + str(n_found_links) + " links.")
print("Picked " + str(n_selected_links) + " links to fetch.")
print()


print("Getting each remote link at time... (it could take a while because there are 10 seconds of timeout for each test.")
print("Is necessary to check the links manually to avoid \"false positive\" alerts. ")
print()
external_probaby_broken = check_link_errors(external_links_to_check)
print()

for source_file, link in external_probaby_broken.items():
    print("File " + source_file + " has a possible broken link: " + link  )

print()
print("Found " + str(len(external_probaby_broken)) + " links to re-check manually. ")
print("Is necessary to check the links manually to avoid \"false positive\" alerts. ") 
print()

    

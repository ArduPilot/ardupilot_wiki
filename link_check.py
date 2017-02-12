#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''
This script reads all files in the docs directory of a wiki section, parses urls
it can find and then tests those urls using requests.head(). The goal is to find
all the dead links within the wiki. An output file is created that shows all the 
searched directories and the file/link for bad links.

Known issues:

	- Some links take an extended amount of time to respond and are excepted.
		These may be good or bad, but they aren't correctly processed
	- sparkfun/virturalrobotics are returned as 404, but are actually good
	- Links split up by formatting syntax won't be picked up
'''

import os
import requests

#Get current directory
cwd = os.getcwd()

checked_directories = ["antennatracker", "common", "copter", "dev", "plane", "planner", "planner2", "rover"]
#checked_directories = ["rover"]

directories = []
for directory in checked_directories:
	directories.append(cwd + "\\" + directory + "\source\docs\\")


outputfile = open('link_check_output.txt', 'w')

#Run through all directories and check all links in the document
for src_dir in directories:
	dirlist = os.listdir(src_dir)
	
	for file in dirlist:
		with open(src_dir + file, 'r') as f:
			outputfile.write("Directory: %s\n" % (src_dir + file))
			for line in f:
				newline1 = line.split('<') # look for beginning of url formatting
				if len(newline1) > 1: # Check that the line actually contained a '<'
					pos_link = newline1[1].split('>') # Split again to totally separate the url
					check_link = pos_link[0].split('/')
					# Make sure that what is contained is actually a link
					if len(check_link) > 1:
						check_http = pos_link[0].split(':')
						if check_http[0] == "http" or check_http[0] == "https": #get rid of false positives
							
							# Test link (see source #2)
							try:
								r = requests.head(pos_link[0])
								status_code = r.status_code
								print status_code

							except:
								print "Excepted"
								status_code = 611

							if status_code == 404:
								outputfile.write("\tLink: %s\n" % (pos_link[0]))
								outputfile.write("\tLinkStatus: Bad, 404\n")
								print "Bad Link"
							elif status_code == 611:
								outputfile.write("\tLink: %s\n" % (pos_link[0]))
								outputfile.write("\tLinkStatus: Excepted\n")
								print "Unknown Issue"

							else:
								pass
						else:
							pass
					else:
						pass
				else:
					pass
		
		print "Done with file"

outputfile.close()

#Sources:
#http://www.codepool.biz/python-check-broken-links-404.html
#http://stackoverflow.com/questions/1140661/what-s-the-best-way-to-get-an-http-response-code-from-a-url#1140822
#https://mail.python.org/pipermail/tutor/2004-August/031232.html
#http://stackoverflow.com/questions/8009882/how-to-read-large-file-line-by-line-in-python
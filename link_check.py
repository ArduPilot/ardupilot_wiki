#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os

#Get current directory
cwd = os.getcwd()

checked_directories = ["antennatracker", "common", "copter", "dev", "plane", "planner", "planner2", "rover"]
#checked_directories = ["rover"]
directories = []

#Hardcoded directories for 
for directory in checked_directories:
	directories.append(cwd + "\\" + directory + "\source\docs\\")
#Run through all directories and check all links in the document
#https://mail.python.org/pipermail/tutor/2004-August/031232.html
#http://stackoverflow.com/questions/8009882/how-to-read-large-file-line-by-line-in-python

print directories

outputfile = open('link_check_output.txt', 'w')

for src_dir in directories:
	dirlist = os.listdir(src_dir)
	
	for file in dirlist:
		if file != "supported-autopilot-controller-boards.rst": # different formatting in this file
			with open(src_dir + file, 'r') as f:
				outputfile.write("Directory: %s\n" % (file))
				for line in f:
					#outputfile.write("\tLine; %s\n" % (line))
					#check for <> formatted links (most of them I think)
					newline1 = line.split('<')
					if len(newline1) > 1:
						pos_link = newline1[1].split('>')
						check_link = pos_link[0].split('/')
						if len(check_link) > 1:
							check_http = pos_link[0].split(':')
							if check_http[0] == "http" or check_http[0] == "https": #get rid of false positives
								outputfile.write("\tLink: %s\n" % (pos_link[0]))
							else:
								print "not a website"
						else:
							print "No good, moving on"
					else:
						print "No link, moving on."

					#TODO check for non <> formatted links
					#newline2 = line.split(':')
		else:
			pass

		print "Done with file"


outputfile.close()


#http://www.codepool.biz/python-check-broken-links-404.html
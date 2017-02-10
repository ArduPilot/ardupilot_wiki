#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import requests

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
								elif status_code == 601:
									outputfile.write("\tLink: %s\n" % (pos_link[0]))
									outputfile.write("\tLinkStatus: Excpeted\n")
									print "Bad Link"

								else:
									pass
									#outputfile.write("LinkStatus: Good\n")
									#outputfile.write("Not 404, status: %d\n" % (status_code))
									#print "Good Link"
							else:
								pass
								print "not a website"
						else:
							pass
							print "No good, moving on"
					else:
						pass
						#print "No link, moving on."

					#TODO check for non <> formatted links
					#newline2 = line.split(':')
		else:
			pass

		print "Done with file"


outputfile.close()


#http://www.codepool.biz/python-check-broken-links-404.html
#http://stackoverflow.com/questions/1140661/what-s-the-best-way-to-get-an-http-response-code-from-a-url#1140822
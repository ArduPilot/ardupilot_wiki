#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import requests

#Get current directory
cwd = os.getcwd()

checked_directories = ["antennatracker", "common", "copter", "dev", "plane", "planner", "planner2", "rover"]
#checked_directories = ["rover"]

directories = []
for directory in checked_directories:
	directories.append(cwd + "\\" + directory + "\source\docs\\")
#Run through all directories and check all links in the document

outputfile = open('link_check_output.txt', 'w')

for src_dir in directories:
	dirlist = os.listdir(src_dir)
	
	for file in dirlist:
		if file != "supported-autopilot-controller-boards.rst": # different formatting in this file
			with open(src_dir + file, 'r') as f:
				outputfile.write("Directory: %s\n" % (src_dir + file))
				for line in f:
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
		else:
			pass

		print "Done with file"

outputfile.close()

#Sources:
#http://www.codepool.biz/python-check-broken-links-404.html
#http://stackoverflow.com/questions/1140661/what-s-the-best-way-to-get-an-http-response-code-from-a-url#1140822
#https://mail.python.org/pipermail/tutor/2004-August/031232.html
#http://stackoverflow.com/questions/8009882/how-to-read-large-file-line-by-line-in-python
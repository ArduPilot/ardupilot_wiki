.. _google-code-in-task-list:
    
===============================================
List of Suggested Tasks for Google Code-in 2018
===============================================

This is a list of tasks suggested by ArduPilot developers for `Google Code-in 2018 <https://codein.withgoogle.com/>`__. These are currently the example tasks and they may be further refined later. If you have your own ideas then please discuss them on either the gitter channel (at https://gitter.im/ArduPilot/GSoC) or on the discuss server (see https://discuss.ardupilot.org/c/google-summer-of-code). 
We have a lot of talented developers in the ArduPilot dev team who would love to mentor good students for Code-in 2018. We're looking for enthusiastic students who can really get stuck into their project and make a substantial contributions to the ArduPilot project.

Coding Tasks
============

Create your first pull request
------------------------------
- Our project uses GitHub as its home, in order to contribute to the project you must learn to use GitHub and create pull requests. This task aims to show you how to use GitHub. Add your name to the ardupilot/Tools/GIT_Test/GIT_Success.txt file and submit a pull request. You can find information on how to that here: http://ardupilot.org/dev/docs/where-to-get-the-code.html

Fix a reported issue in Ardupilot code
--------------------------------------
- There are close to 1000 issues listed in the Ardupilot github issues list. (https://github.com/ArduPilot/ardupilot/issues) Look for issues labeled with "help wanted" or "good first issue", and try to fix it and submit your changes.

Create a driver for a new sensor
--------------------------------
- Create a driver for a sensor that is nto available in ardupilot all ready. You could write a driver for a new compass, or for a new barometer, temperature sensor, etc. Follow the wiki's recomendations for writing new drivers, and submit your code via a PR.

Modify MAVProxy to run on Mac OSX
---------------------------------
- Mavproxy is a great simple ground station that is written in python. Currently it is not fully supported under Mac OSX. Modify MavProxy so that it runs in Mac OS without issues. 

Create a script to install all dependecies needed to compile the code in Mac OSX
--------------------------------------------------------------------------------
- There exists a scipt called ardupilot/Tools/Scripts/install-prereqs-mac.sh that should install all necesary tools to compile the ardupilot code. Verify that its working and if not fix any issues you find.

Documentation Tasks
===================

Create a video tutorial of how to write a hwdef file from a schematic
---------------------------------------------------------------------

Verify the "how to build the code" instructions on the Wiki
-----------------------------------------------------------
- Go throught the wiki's instructions on how to install the development enviroment tools and how to build the code, report and fix any issues you encounter.

Make a video tutorial explaining how to debug code using GDB
------------------------------------------------------------
- Make a short video showing how to use GDB to debug code issues.

Create a tutorial showing how to connect Ardupilot Telemetry using an LTE Modem
-------------------------------------------------------------------------------
- It is possible to connect an ardupilot vehicle to a Raspbery Pi type companion computer, then have this CC connect to the internet using an LTE dongle. Create a VP network and connect to the vehicle from another internet connected computer to recieve telemetry from it. This task is to create a tutorial on the wiki showing how to do this. A video showing it working would also be great to include in this tutorial

Write a tutorial for our wiki
-----------------------------
- Choose a topic that the comunity has interest on, and create a tutorial for our wiki. Having images and a video is a must for it to be intresting. Topics for this would be things like how to set up the failsafes in plane, how to configure a plane for reverse thrust, how to setup a lidar for obstacle avoidance, how to connect a companion computer, etc.

Outreach / Research Tasks
=========================

Come up with a tagline for Ardupilot
------------------------------------
- Ardupilot needs to have a short tag line that conveys quality and the capabilities of our software. Something like Intels "Intel Inside" or "Powered by"

Make a promotional video
------------------------
- Ardupilot needs a video that shows what ardupilot is, its history and how versatile it is. It should also show the commercial platforms that currently run ardupilot and convey a sense of reliability.

Write a blog post
-----------------
- Write a blog post showcasing the capabilities of what ardupilot can do. It could be an example of an application of a vehicle running ardupilot, or a specific feature that ardupilot has. Make sure that you highlight why this makes Ardupilto different from other vehicles and what makes ardupilot the best and most versatile autopilot out there.

Create a webpage showing the usage statistics of the project
------------------------------------------------------------
- Create a web page that pulls in realtime statistics about code contributions, pull requests, downloads etc. And showcase it using realtime graphs and data in a great looking dashbard web page.

Organize a talk on your school
------------------------------
- Organiza a talk in your school that showcases the applications of drones and helps to make people more aware of the real capabilities of drones. Make sure to highlight that most stories you hear about drones in the media are highly exageretaed and show the real truth behind these stories. Try to make people aware that a cell phone camera is more intrusive than a drone camera (anyone can take a picture of you with a cell phone without you knowing, where as a drone you will most definetly hear from far away before it is actually close enough to take your picture)

Quality Assurance
=================

Review a PR and comment any potential issues you find with the code.
--------------------------------------------------------------------
- As part of the process of submitting code, PRs have to be reviewed and tested. Find a simple PR, review the submited code and comment on any potential issues you find.

Flight test new code
--------------------
- Flight test the latest beta release fo the code and report any issues you find. Also check the release notes and included changes and test any changes made to see if they are working correctly and report back

Check that the ROS documentation is correct
-------------------------------------------
- Go throught the ROS documentation on the wiki, try to reproduce the tutorials an fix any issues that you encounter. 

Update the images of pixhawk 1s to the cube on the wiki
-------------------------------------------------------
- Check the wiki for images that have an old pixhawk on them and update them to use the cube

Check the wiki for broken links
-------------------------------
- Several pages on the wiki hae broken hyepr links to old pages that no longer exist or point to products that are no longer available. Go trhough the wiki and fix any broken links you find.

Design
======

Create a new set of vehicle icons to be used in Mission planner
---------------------------------------------------------------
- The icons on the mission planner gui for the different vehicles look a bit outdated. Create a more modern set of icons for the different types of vehicles (copter, plane, trad heli, rover, antenna tracker, sub, etc...)

Propose a new UI/UX for a ground control station
------------------------------------------------


Create a simple UI/UX for APWeb
-------------------------------
- Design and code a nice simple UI/UX for APWeb

Create an ArduPilot logo in a square form factor for profile pictures
---------------------------------------------------------------------


Create a system connection diagram with updated components
----------------------------------------------------------
- Reproduce this image using new up to date components so that it reflects the current state of the art in components. https://www.dropbox.com/s/b4u3kb1rw2xbvef/Pixhawk-Inforgraphic2.jpg?dl=0"



.. _git-install:

==============
Installing Git
==============

.. inclusion-marker-do-not-remove

Git is a free and open source distributed version control system that is used to manage ArduPilot codebase.
Git is available on all major OS platforms, and a variety of tools exist to make it easier to get started.

Git command line
----------------

Ubuntu
++++++

Linux/Ubuntu users can install with apt :

 - ``sudo apt-get update``
 - ``sudo apt-get install git``
 - ``sudo apt-get install gitk git-gui``

.. youtube:: G1Kc-1aF8HI


Windows and other Systems
+++++++++++++++++++++++++

 - download and install from `git-scm.com's download page <http://git-scm.com/downloads>`__

Recommended GUI Tools
---------------------

The git command line, gitk and git-gui tools are the basics that all users should have installed and are available from a few places.

A comprehensive list of GUI tools are listed `here on git-scm.com <https://git-scm.com/downloads/guis>`__ but here are a few of the developer favourites:

- `GitHub Desktop <https://desktop.github.com/>`__ for Windows and Mac is an easy to use tool with good GitHub integration but has fewer features than other clients.
- `Sourcetree for Windows and Mac OSX <https://www.sourcetreeapp.com/>`__ is a full featured tool with a nice interface with many features including allowing individual lines from a file to be included in a new commit (simpler tools only allow the entire file to be included)
- `GitKraken <https://www.gitkraken.com/>`__ is a a full featured tool for Windows, Mac and Linux

.. Alternative for Windows user

GitHub Desktop
--------------

- Go to the `desktop.github.com <https://desktop.github.com/>`__ and push the "Download for Windows" or "Download for macOS" buttons in the middle of the page
      
      .. image:: ../images/CloningTheRepository_Windows_DownloadGithub.png
          :target: ../_images/CloningTheRepository_Windows_DownloadGithub.png
      
- Save the **GitHubSetup.exe** somewhere on your machine and then run it and follow the instructions to install GitHub client
- On GitHub client click the right arrow button to view a list of recent commits or right-mouse-button click on the ardupilot/ardupilot repository and "open in explorer".

   .. image:: ../images/CloningTheRepository_Windows_OpenGithub.png
       :target: ../_images/CloningTheRepository_Windows_OpenGithub.png

- Ensure your github settings are set to leave line endings untouched.

   -  The "Git Shell (or Bash)" terminal was also installed when you
      installed Git.  Click on your new "Git Shell (or Bash)" Icon and
      type in the following in the Git "MINGW32" Terminal window:

      ::

          git config --global core.autocrlf false

-  You can now also open the file in your favourite editor such as `NotePad++ <http://notepad-plus-plus.org/>`__, `Sublime Text <http://www.sublimetext.com/>`__ or `acme <http://acme.cat-v.org/>`__.

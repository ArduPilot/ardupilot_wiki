.. _common-wiki-editing-setup:

=========================================
Wiki Editing - Setting Up the Environment
=========================================

.. _common_wiki_editing_guide_building_docs:

Before submitting large changes to the wiki it is best to check the pages are rendered correctly including checking all images and links appear correctly.  This page explains how to "build" the wiki (i.e. convert the rst files and image files into a set of html pages) on your local machine so that you can perform these checks.

There are several methods to setup a local build environment:

#. Use the `Vagrantfile <https://github.com/ArduPilot/ardupilot_wiki/blob/master/Vagrantfile>`__ in the root of the repo to create a Linux virtual machine with all the necessary packages installed.  This is the preferred and supported method.
#. Use the `Dockerfile <https://github.com/ArduPilot/ardupilot_wiki/blob/master/Dockerfile>`__ in the root of the repo to create a Linux container with all the necessary packages installed.
#. Or simply install the required libraries on your local Linux or Windows machine, using the instructions below. 

Fork, Clone, and Update Your Copy of the Wiki
=============================================

First, the wiki should be forked and cloned much like :ref:`developers do for the flight code <dev:where-to-get-the-code>`.

#. Create a `Github account <https://github.com/join>`__ if you do not have one already
#. :ref:`Install a Git Client <dev:git-install>`
#. :ref:`Fork <dev:git-fork>` the `ardupilot_wiki <https://github.com/ArduPilot/ardupilot_wiki>`__ to create your own copy in GitHub (note linked instructions are for the ardupilot flight code so "ardupilot" should be replaced with "ardupilot_wiki")

    .. image:: ../../../dev/source/images/APM-Git-Github-Fork-300x64.jpg

#. :ref:`Clone <dev:git-clone>` your copy of ardupilot_wiki on GitHub to your local machine (note linked instructions are for the ardupilot flight code so "ardupilot" should be replaced with "ardupilot_wiki")

   .. code-block:: bash

       git clone https://github.com/YOURID/ardupilot_wiki.git

Be sure to keep your fork of the repository updated, both locally, and on GitHub both as you install and as you develop new contributions. Be sure to create a working branch locally when you start to make changes, pushing that branch up to your GitHub repo, and then making a PR (pull-request from there to the Wiki repo), in a manner similar to making :ref:`code PRs <submitting-patches-back-to-master>`.
   
.. code-block:: bash

    git remote add upstream https://github.com/ArduPilot/ardupilot_wiki.git
    git checkout master
    git fetch upstream
    git rebase upstream/master
    git push -f origin master
       
Setup the Environment
=====================

Setup in Linux (or WSL)
-----------------------

Run the following command from the ardupilot_wiki directory you cloned:

.. code-block:: bash

    ./Sphinxsetup.sh

Then jump down to "Build the wiki".

Setup in Windows
----------------

Running in WSL or Cygwin is not required. The ArduPilot wiki can be built directly on Windows.

Ensure that the latest version of `Python <https://www.python.org/downloads/>`__ is installed and the "Add Python 3.x to PATH" option is selected during installation.

Run the following command from the ardupilot_wiki directory you cloned, ensuring that it is run as administrator (right click -> Run as Administrator):

.. code-block:: bash

    ./Sphinxsetup.bat

Then jump down to "Build the wiki".


Setup with Vagrant
------------------

#. Download and install `Vagrant <https://www.vagrantup.com/downloads.html>`__

#. Download and install `Oracle VirtualBox <https://www.virtualbox.org/wiki/Downloads>`__.

#. Windows users should install an SSH client on the computer before starting vagrant. Vagrant needs  SSH client program to access the development container. We have had great success with OpenSSH packer from MLS-Software `here <http://www.mls-software.com/opensshd.html>`__

The main steps for building the docs are:

#. Open a command prompt in the root of the ardupilot_wiki repo, and start Vagrant.  The first time this is run it may take over 30 minutes to complete.

   .. code-block:: bash

       cd ardupilot_wiki
       vagrant up

#. SSH into Vagrant (Windows users may need to add SSH to your PATH)

   .. code-block:: bash

       vagrant ssh

#. Navigate in the SSH shell to the /vagrant directory and start the build.

   .. code-block:: bash

       cd /vagrant
       python3 update.py

Setup with Docker
-----------------

#. Download and install Docker according to their `official documentation <https://docs.docker.com/install/>`__.

#. Open a command prompt in the root of the ardupilot_wiki repo and build the docker container for the wiki:

   .. code-block:: bash

       cd ardupilot_wiki
       docker build . -t ardupilot_wiki

This will build a docker image with all package setup to build the wiki and name it ``ardupilot_wiki``.

#. Use the container to build the wiki with a shared volume to get the build result on your computer:

   .. code-block:: bash

       docker run --rm -it -v "${PWD}:/ardupilot_wiki" -u "$(id -u):$(id -g)" ardupilot_wiki python3 update.py

That will build the wiki with the ``update.py`` similarly as in `Build the Wiki`_. The `-v` is used to share the content of the current directory, that holds all the documentation, to the container. The `-u` is used to make docker use the same permission as your current user. With those two commands, the resulting build is accessible as in `Check the Results`_

Build the Wiki
=================

As shown in the last step of the vagrant instructions above, use update.py to build some or all of the wiki.

.. code-block:: bash

    python3 update.py (to build all wikis)
    python3 update.py --site copter  (to build just the copter wiki)
    python3 update.py --site plane   (to build just the plane wiki)
    python3 update.py --site rover   (to build just the rover wiki)
    python3 update.py --site dev     (to build just this developer wiki)

The update.py script will copy the common files into each wiki subdirectory and then build each wiki.

After the build, the wiki files will be copied to ``--destdir``. By default, in Linux this is ``/var/sites/wiki/web`` and in Windows it is ``..\wiki``.

.. note:: The script will show the build output of each of the wikis. This should be inspected for warnings and errors.

Details on the building and infrastructure of the wiki can be found :ref:`here <common-wiki-editing-infra-overview>`.

Check the Results
-----------------

With your favourite web browser, open the locally built wiki which should be near where the ardupilot_wiki repo was cloned to

- For Copter look for ``ardupilot_wiki/copter/build/html/index.html``
- For Plane look for ``ardupilot_wiki/plane/build/html/index.html``
- For Rover look for ``ardupilot_wiki/rover/build/html/index.html``
- For Developer look for ``ardupilot_wiki/dev/build/html/index.html``



Build options
-------------

.. code-block:: text

    usage: update.py [-h] [--site SITE] [--clean] [--cached-parameter-files] [--parallel PARALLEL] [--destdir DESTDIR] [--enablebackups] [--backupdestdir BACKUPDESTDIR] [--paramversioning] [--verbose] [--fast]

    Copy Common Files as needed, stripping out non-relevant wiki content

    optional arguments:
    -h, --help            show this help message and exit
    --site SITE           If you just want to copy to one site, you can do this. Otherwise will be copied.
    --clean               Does a very clean build - resets git to master head (and TBD cleans up any duplicates in the output).
    --cached-parameter-files
                            Do not re-download parameter files
    --parallel PARALLEL   limit parallel builds, -1 for unlimited
    --destdir DESTDIR     Destination directory for compiled docs
    --enablebackups       Enable several backups up to const N_BACKUPS_RETAIN in --backupdestdir folder
    --backupdestdir BACKUPDESTDIR
                            Destination directory for compiled docs
    --paramversioning     Build multiple parameters pages for each vehicle based on its firmware repo.
    --verbose             show debugging output
    --fast                Incremental build using already downloaded parameters, log messages, and video thumbnails rather than cleaning before build.

Most of the build options will only be used on the server that builds the wiki. The options that editors are most likely to find useful are `--site SITE` and `--fast`. `--site SITE` allows for building a single wiki rather than building all of the wikis. This is a huge bump in speed already, but sometimes checking changes across multiple wikis is necessary. The `--fast` option skips downloading data needed for vehicle parameters, log messages, and video thumbnails. This can be helpful when doing small formatting changes or when new parameters/log messages are not needed for the build. Newly added video thumbnails will be downloaded and cached for the next build with `--fast` enabled.

RST editing/previewing
======================

The tools described in this section can make it easier to edit RST files and reduce the time required to preview changes.

.. note:: The RST rendering tools can be useful for rapidly previewing small changes in the documentation. Rendering will not be perfect because the tools are designed for generic reStructuredText (they and are not "Sphinx-aware). We therefore recommend that you build with Sphinx to do a final review before you make a documentation pull request. 

RST rendering on Windows
------------------------

A combination of two Windows tools can help you preview your modifications:
  	
* `Notepad++ plugin for RST files <https://github.com/steenhulthin/reStructuredText_NPP>`__
* `restview (on-the-fly renderer for RST files) <https://mg.pov.lt/restview/>`__
* `PyCharm <https://www.jetbrains.com/pycharm/>`__

The Notepad++ plugin helps you with code completion and syntax highlighting during modification.
Restview renders RST files on-the-fly, i.e. each modification on the RST file can be immediately
visualized in your web browser. 

The installation of the Notepad++ plugin is clearly explained on the plugin's website (see above).

Restview can be installed with:

.. code-block:: bat
	
	python3 -m pip install restview
		
The restview executable will be installed in the **Scripts** folder of the Python main folder.
Restview will start the on-the-fly HTML rendering and open a tab page in your preferred web browser.

Example:

If you are in the root folder of your local Wiki repository:

.. code-block:: bat
	
	start \python-folder\Scripts\restview common\source\docs\common-wiki_editing_guide.rst	
	
RST rendering on Linux
----------------------

* `ReText <https://github.com/retext-project/retext>`__ is a Linux tool that provides syntax highlighting and basic on-the-fly rendering in a single application.


.. note:: Although the tool is Python based, don't try it on Windows as it is very prone to crashes (this is also stated by the website).

* `PyCharm <https://www.jetbrains.com/pycharm/>`__



[copywiki destination="copter,plane,rover,planner,planner2,antennatracker,dev,ardupilot,mavproxy"]

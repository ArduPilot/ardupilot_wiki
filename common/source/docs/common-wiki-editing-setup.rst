.. _common-wiki-editing-setup:

=========================================
Wiki Editing - Setting Up the Environment
=========================================

.. _common_wiki_editing_guide_building_docs:

Before submitting large changes to the wiki it is best to check the pages are rendered correctly including checking all images and links appear correctly.  This page explains how to "build" the wiki (i.e. convert the rst files and image files into a set of html pages) on your local machine so that you can perform these checks.

There are several methods to setup a local build environment:

#. Use the `Vagrantfile <https://github.com/ArduPilot/ardupilot_wiki/blob/master/Vagrantfile>`__ in the root of the repo to create a Linux virtual machine with all the necessary packages installed.  This is the preferred and supported method.
#. Use the `Dockerfile <https://github.com/ArduPilot/ardupilot_wiki/blob/master/Dockerfile>`__ in the root of the repo to create a Linux container with all the necessary packages installed.
#. Or simply install `Sphinx <http://www.sphinx-doc.org/en/stable/install.html>`__ on your local Linux machine. To do this, run the following command:

   .. code-block:: bash

       sudo ./Sphinxsetup.sh

Then jump down to "Build the wiki".

Fork and Clone the Wiki
-----------------------

The wiki should be forked and cloned much like :ref:`developers do for the flight code <dev:where-to-get-the-code>`.

- Create a `Github account <https://github.com/join>`__ if you do not have one already
- :ref:`Install a Git Client <dev:git-install>`
- :ref:`Fork <dev:git-fork>` the `ardupilot_wiki <https://github.com/ArduPilot/ardupilot_wiki>`__ to create your own copy in GitHub (note linked instructions are for the ardupilot flight code so "ardupilot" should be replaced with "ardupilot_wiki")

    .. image:: ../../../dev/source/images/APM-Git-Github-Fork-300x64.jpg

- :ref:`Clone <dev:git-clone>` your copy of ardupilot_wiki on GitHub to your local machine (note linked instructions are for the ardupilot flight code so "ardupilot" should be replaced with "ardupilot_wiki")

   .. code-block:: bash

       git clone https://github.com/YOURID/ardupilot_wiki.git

Setup with Vagrant
------------------

#. Download and install `Vagrant <https://www.vagrantup.com/downloads.html>`__

#. Download and install `Oracle VirtualBox <https://www.virtualbox.org/wiki/Downloads>`__.

#. Windows users should install an SSH client on the computer before starting vagrant. Vagrant needs  SSH client program to access development container. We have had great success with OpenSSH packer from MLS-Software `here <http://www.mls-software.com/opensshd.html>`__

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
       python update.py

Build the Wiki
--------------

As shown in the last step of the vagrant instructions above, use update.py to build some or all of the wiki. You may need to prefix the python commands below with "sudo", especially when rebuilding.

   .. code-block:: bash

       python update.py
       python update.py --site copter  (to build just the copter wiki)
       python update.py --site plane   (to build just the plane wiki)
       python update.py --site rover   (to build just the rover wiki)
       python update.py --site dev     (to build just this developer wiki)

The update.py script will copy the common files into each wiki subdirectory and then build each wiki.

.. note::

    The script will show the build output of each of the wikis.  This should be inspected for warnings and errors.
    The script does some file copying at the end, which will fail and can be ignored (this is used when publishing
    the docs)

Check the Results
-----------------

With your favourite web browser, open the locally built wiki which should be near where the ardupilot_wiki repo was cloned to

- For Copter look for ``ardupilot_wiki/copter/build/html/index.html``
- For Plane look for ``ardupilot_wiki/plane/build/html/index.html``
- For Rover look for ``ardupilot_wiki/rover/build/html/index.html``
- For Developer look for ``ardupilot_wiki/dev/build/html/index.html``

Setup with docker
-----------------

#. Download and install Docker according to their `official documentation <https://docs.docker.com/install/>`__ .

#. Open a command prompt in the root of the ardupilot_wiki repo and build the docker container for the wiki:

   .. code-block:: bash

       cd ardupilot_wiki
       docker build . -t ardupilot_wiki

This will build a docker image with all package setup to build the wiki and name it ``ardupilot_wiki``.

#. Use the container to build the wiki with a shared volume to get the build result on your computer:

   .. code-block:: bash

       docker run --rm -it -v "${PWD}:/ardupilot_wiki" -u "$(id -u):$(id -g)" ardupilot_wiki python update.py

That will build the wiki with the ``update.py`` similary as in `Build the Wiki`_. The `-v` is used to share the content of the current directory, that hold all the documentation, to the container. The `-u` is used to make docker use the same permission as your current user. With those two command the resulting build is accessible as in `Check the Results`_

RST editing/previewing
======================

The tools described in this section can make it easier to edit RST files and reduce the time required to preview changes.

.. note:: 
    
    The RST rendering tools can be useful for rapidly previewing small changes in the documentation. Rendering will not be perfect because the tools are designed for generic reStructuredText (they and are not "Sphinx-aware). We therefore recommend that you build with Sphinx to do a final review before you make a documentation pull request. 

RST rendering on Windows
------------------------

A combination of two Windows tools can help you previewing your modifications:
  	
* `Notepad++ plugin for RST files <https://github.com/steenhulthin/reStructuredText_NPP>`__
* `restview (on-the-fly renderer for RST files) <https://mg.pov.lt/restview/>`__
* `PyCharm <https://www.jetbrains.com/pycharm/>`__

The Notepad++ plugin helps you with code completion and syntax highlighting during modification.
Restview renders RST files on-the-fly, i.e. each modification on the RST file can be immediately
visualized in your web browser. 

The installation of the Notepad++ plugin is clearly explained on the plugin's website (see above).

Restview can be installed with:

.. code-block:: bat
	
	python -m pip install restview
		
The restview executable will be installed in the **Scripts** folder of the Python main folder.
Restview will start the on-the-fly HTML rendering and open a tab page in your preferred web browser.

Example:

If you are in the root folder of your local Wiki repository:

.. code-block:: bat
	
	start \python-folder\Scripts\restview common\source\docs\common-wiki_editing_guide.rst	
	
RST rendering on Linux
----------------------

* `ReText <https://github.com/retext-project/retext>`__ is a Linux tool that provides syntax highlighting and basic on-the-fly rendering in a single application.


.. note:: Although the tool is Python based, don't try it on Windows as it very prone to crashes (this is also stated by the website).

* `PyCharm <https://www.jetbrains.com/pycharm/>`__



[copywiki destination="copter,plane,rover,planner,planner2,antennatracker,dev"]

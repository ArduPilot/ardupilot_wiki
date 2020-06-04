.. _common-wiki-editing-infra-overview:

============================
Wiki Infrastructure Overview
============================

The wiki is built using the static site generator `Sphinx <http://www.sphinx-doc.org/en/stable/>`__ from source written in `reStructured Text markup <http://www.sphinx-doc.org/en/stable/rest.html>`__ and hosted on `Github here <https://github.com/ArduPilot/ardupilot_wiki>`__. 

The wiki has several sections:

- `Common <https://github.com/ArduPilot/ardupilot_wiki/tree/master/common/source/docs>`__ holds pages that are common to all vehicles like the :ref:`Optional Hardware <copter:common-optional-hardware>` section of the wikis.  The pages all have the **common-** prefix and are stored in the **/common/source/docs/** directory.  Common images are in the `images <https://github.com/ArduPilot/ardupilot_wiki/tree/master/images>`__ directory.  Common configuration information for the Wiki Sphinx build is stored in **/common_conf.py**.
- Each vehicle or ground station has a separate folder in the repository (e.g. '/copter', '/plane') which contains its own source and configuration files (**conf.py**).  Corresponding images are stored in the /images/ subfolder.

The **update.py** build script copies the common topics into specified (in source) target wikis directories and then build them.

The **Vagrantfile** can be used by Vagrant to set up a local build environment independent of your host system.
(`Vagrant <https://www.vagrantup.com>`__ is a software tool that allows for the quick and automated  setup of a virtual machine on your host operating system).
This allows you to edit the source in your host computer but manage the build inside Vagrant. You can also
manually set up a build environment (just inspect the Vagrantfile for dependencies).

The wikis use a `common theme <https://github.com/ArduPilot/sphinx_rtd_theme>`__ that provides the top menu bar. 

[copywiki destination="copter,plane,rover,planner,planner2,antennatracker,dev,ardupilot,mavproxy"]

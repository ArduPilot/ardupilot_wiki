.. _where-to-get-the-code:

================================
Downloading the Code / Using Git
================================

This article explains where to get the ArduPilot code and how to submit
changes to the project.

Overview
========

The ArduPilot project uses `git <http://git-scm.com/>`__ for source code
management and `GitHub <https://github.com/>`__ for source code hosting.

For developers who simply want to take a quick look at the code, the
source code for Plane, Copter, Rover, Sub and Antenna Tracker can be
viewed with any web browser by going to the `ArduPilot repo on Github <https://github.com/ArduPilot/ardupilot>`__.

For developers who wish to **download** and **build** the latest code:

- :ref:`install a git client <git-install>` on your local computer
- "clone" the project to download "master" to your local computer
- :ref:`build/compile the firmware <building-the-code>` using waf or make

Note that the latest firmware is built automatically and can be directly downloaded from `firmware.ardupilot.org <http://firmware.ardupilot.org>`__. 

For developers who wish to **download**, **compile**, **edit** and then **contribute** code
back to ArduPilot, the following steps should be followed:

- :ref:`fork <git-fork>` the project to create a personal copy on GitHub
- :ref:`install a git client <git-install>` on your local computer
- "clone" the project to download this copy to a local desktop computer
- create a new branch on their local desktop to  hold the new feature
- edit the code and "commit" it to the local repo.
- "push" the new branch up to the personal repo (i.e. their fork)
- raise a pull request to get the changes merged into the "master" project.

.. _where-to-get-the-code_learning_git:

Learning git
============

This guide covers the basic git commands/concepts needed to work with
the project: clone, branch, commit, push.

If you want to know more about git there are many great resources
online. Here are just a few you may find useful:

-  `Try Git <http://try.github.io/levels/1/challenges/1>`__:
   browser-based interactive tutorial for learning git
-  `Git Ready <http://gitready.com/>`__: tutorials of varying difficulty
   levels
-  `Git SCM Book <http://git-scm.com/book/en/Getting-Started>`__:
   introduction and full documentation
-  `Git Flight Rules <https://github.com/k88hudson/git-flight-rules/blob/master/README.md>`__:
   a guide for what to do when things go wrong

Next Steps
==========

Follow the links below to learn more about how we use Git with ArduPilot

.. toctree::
    :maxdepth: 1

    Installing Git <git-install>
    Fork the repository <git-fork>
    Clone the repository <git-clone>
    Branching and Committing <git-branch>
    Rebasing: keeping your code up to date <git-rebase>
    Interactive Rebase: cleaning up commits <git-interactive-rebase>
    Git Submodules <git-submodules>

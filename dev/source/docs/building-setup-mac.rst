.. _building-setup-mac:

=========================================
Setting up the Build Environment (MacOSX)
=========================================

This article shows how to set up the build environment on macOS.

..  youtube:: wLK2wLwEXm4
    :width: 100%


The recommended way to install the prerequisites is the environment install
script. It is kept up to date with the current toolchain and Python
requirements, so it is less likely to drift than a manual list of steps. From
your ``ardupilot`` directory, run:

::

    ./Tools/environment_install/install-prereqs-mac.sh -y

The script installs the Xcode command line tools, Homebrew, the ARM toolchain,
and the Python packages ArduPilot needs. If you would rather set things up by
hand, read the script and run the steps it performs.

.. note::

    Do not run the script with ``sudo``, and avoid ``sudo`` for Python or pip
    packages in general. The script installs its Python packages into whatever
    ``python3`` is first on your ``PATH``; answering yes to its pyenv prompt
    keeps them out of the system Python. Homebrew will prompt for your
    password itself when it needs it.

Once the prerequisites are installed you should be able to build with waf as
described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

Cleaning
--------

If there have been updates to some git submodules you may need to do a full clean build. To do that use:

::

    ./waf distclean

Commands ``clean`` and ``distclean`` can be used to clean the objects produced by the build.
``clean`` keeps the configure information, cleaning only the objects for the current board. ``distclean`` cleans everything for every board, including the saved configure information.

Follow the instructions for `build <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__ .

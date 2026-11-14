.. _building-setup-mac:

=========================================
Setting up the Build Environment (MacOSX)
=========================================

This article shows how to set up a build environment on macOS. It is written for Apple Silicon Macs; Intel Macs can only use it if Homebrew is already installed (see below).

The setup is done by the `install-prereqs-mac.sh <https://github.com/ArduPilot/ardupilot/blob/master/Tools/environment_install/install-prereqs-mac.sh>`__ script in the ArduPilot repository. If you prefer to set up the environment by hand, read the script and follow the steps it takes.

Install Xcode Command Line Tools
--------------------------------

These provide git and the compilers. Open a terminal and run:

::

    xcode-select --install

Clone ArduPilot repository
--------------------------

.. include:: git-clone.rst
    :start-after: inclusion-marker-do-not-remove
    :end-before: Cloning with the GitHub GUI (Windows or MAC)

Run the install script
----------------------

On Apple Silicon Macs:

- install `Homebrew <https://brew.sh>`__ first and run the ``eval "$(/opt/homebrew/bin/brew shellenv)"`` step it prints at the end, otherwise the script stops with ``brew: command not found``
- if you will build firmware for autopilots, install Rosetta 2, which the ARM toolchain needs: ``softwareupdate --install-rosetta``

On Intel Macs, Homebrew's installer no longer supports Intel processors and stops with "Homebrew on macOS is only supported on Apple Silicon processors!". The script therefore only works on an Intel Mac where Homebrew is already installed.

Do not run the script with ``sudo``. From your ardupilot directory, run:

::

    Tools/environment_install/install-prereqs-mac.sh -y

The script:

- installs `Homebrew <https://brew.sh>`__ if it is not already installed (Apple Silicon only), then uses it to install gawk, coreutils, wget and ccache
- optionally installs Python 3.10 using `pyenv <https://github.com/pyenv/pyenv>`__. This is recommended if your system Python is causing build errors
- optionally installs the ARM toolchain, which is needed to build firmware for autopilots but not for SITL
- installs the Python packages needed to build ArduPilot and run MAVProxy and SITL
- adds the toolchain, the ArduPilot tools and ccache to your PATH in ``~/.zshrc`` (or ``~/.bash_profile`` if you use bash)
- updates the git submodules

``-y`` answers yes to every question. Without it the script asks before installing pyenv, before installing the ARM toolchain, and before each change to your PATH. It will ask for your password when installing the ARM toolchain.

When it finishes, close and reopen the terminal (or run ``source ~/.zshrc``, or ``source ~/.bash_profile`` if you use bash) so the PATH changes take effect.

Now you should be able to build with waf as described in `BUILD.md <https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md>`__.

Follow the :ref:`MAVProxy documentation <mavproxy:mavproxy-downloadinstallmac>` if you plan to use the simulator.

Cleaning
--------

If there have been updates to some git submodules you may need to do a full clean build. To do that use:

::

    ./waf distclean

Commands `clean` and `distclean` can be used to clean the objects produced by the build.
`clean` keeps the configure information, cleaning only the objects for the current board. `distclean` cleans everything for every board, including the saved configure information.

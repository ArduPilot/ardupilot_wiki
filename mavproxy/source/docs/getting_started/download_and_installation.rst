.. _mavproxy-downloadinstall:

=========================
Download and Installation
=========================

The following instructions are for user that just want to run MAVProxy. Developers should look at the :ref:`Developer's Guide <mavproxy-development>` for setting up the development environment.

MAVProxy is distributed as a Python package (see `PyPI <https://pypi.org/project/MAVProxy/>`__).
A number of pre-requisite packages are required to run MAVProxy, as detailed in the individual sections below.

.. _mavproxy-downloadinstallwindows:

Windows
=======

A complete windows installer (package) for MAVProxy is available at
http://firmware.ardupilot.org/Tools/MAVProxy/. The latest version is: ``MAVProxySetup-latest.exe``. It is compatible
with Windows 7 or later.

A link to MAVProxy will be available on the start menu. MAVProxy can
also be called from the commandline via mavproxy.exe

.. note::
    Any references to "mavproxy.py" in this documentation should be
    replaced by "mavproxy.exe" on the Windows commandline.

Alternatively, Windows 10 users can use the Windows Subsystem for Linux (WSL) to run MAVProxy from a Linux environment
within Windows. It generally provides better stability than the native Windows installer above.


.. _mavproxy-downloadinstalllinux:

Linux
=====

For Debian based systems (including Ubuntu, WSL, Raspian):

.. code:: bash

    sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
    pip3 install PyYAML mavproxy --user
    echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc


For Fedora based systems:

.. code:: bash

    sudo dnf install python3-devel python3-opencv python3-wxpython4 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml redhat-rpm-config
    pip3 install PyYAML mavproxy --user
    echo 'export PATH="$PATH:$HOME/.local/bin"' >> ~/.bashrc

.. note::
    If MAVProxy is intended to run "headless" (non-GUI), the ``python3-wxgtk4.0`` or ``python3-wxpython4`` packages can be skipped.

If you get a "permission denied" error message when connecting to serial devices,
the user permissions may need to be changed:

.. code:: bash

    sudo usermod -a -G dialout <username>

The system will need to be restarted to apply the above change.

Updating
--------

To update an existing installation with the current release:

.. code:: bash

    pip3 install mavproxy pymavlink --user --upgrade

To update an existing installation with the current development version (ie, from its master branch):

.. code:: bash

    pip3 install mavproxy --user git+https://github.com/ArduPilot/mavproxy.git@master

.. _mavproxy-downloadinstallmac:

Mac
===

Install Python via

.. code:: bash

    brew install python --framework

Uninstall python-dateutil (OSX and Windows come bundled with a version that is not supported for some dependencies):

.. code:: bash

    sudo pip3 uninstall python-dateutil

Install MAVProxy and its remaining dependencies from the public PyPi repository:

.. code:: bash

    sudo pip3 install wxPython
    sudo pip3 install gnureadline
    sudo pip3 install billiard
    sudo pip3 install numpy pyparsing
    sudo pip3 install MAVProxy

.. _mavproxy-downloadinstall:

=========================
Download and Installation
=========================

The following instructions are for user that just want to run MAVProxy. Developers should look at the :ref:`Developer's Guide <mavproxy-development>` for setting up the development environment.

.. _mavproxy-downloadinstallwindows:

Windows
=======

A complete windows installer for MAVProxy is available at
http://firmware.ardupilot.org/Tools/MAVProxy/.

.. note::

    Please note that the "missionedit" module does not currently run on the
    Windows package of MAVProxy. This is being investigated and will
    hopefully be fixed soon. See https://github.com/ArduPilot/MAVProxy/issues/129 for more details.

Alternatively, Windows Subsystem for Linux (WSL) can be used to run MAVProxy from a Linux environment
within Windows. It generally provides better stability than the native Windows installer above.


.. _mavproxy-downloadinstalllinux:

Linux
=====

MAVProxy can be installed via package managers. Select the appropriate set of commands for your Linux variant below.

For Debian based systems (including WSL):

.. code:: bash

    sudo apt-get install python3-dev python3-opencv python3-wxgtk3.0 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml
    pip3 install mavproxy --user
    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
    
For Fedora based systems:

.. code:: bash

    sudo dnf install python3-devel python3-opencv wxPython python3-pip python3-matplotlib pygame python3-lxml python3-yaml redhat-rpm-config
    pip3 install mavproxy --user
    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc

For Raspian / Raspberry Pi OS:

.. code:: bash

    sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml libxml2-dev
    pip3 install mavproxy --user
    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc


The user permissions may also need to be changed to allow access to serial devices:
   
.. code:: bash

    sudo adduser <username> dialout    

The system will need to be logged out and logged back in again to apply the above change.

.. _mavproxy-downloadinstallmac:

Mac
===

Install Python via

.. code:: bash

    brew install python --framework

Uninstall python-dateutil (OSX and Windows come bundled with a version that is not supported for some dependencies):

.. code:: bash

    sudo pip uninstall python-dateutil

Install MAVProxy and its remaining dependencies from the public PyPi repository:

.. code:: bash

    sudo pip install wxPython
    sudo pip install gnureadline
    sudo pip install billiard
    sudo pip install numpy pyparsing
    sudo pip install MAVProxy



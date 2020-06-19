.. _mavproxy-downloadinstall:

=========================
Download and Installation
=========================

The following instructions are for user that just want to run MAVProxy. Developers should look at the :ref:`Developer's Guide <mavproxy-development>` for setting up the development environment.

.. _mavproxy-downloadinstallwindows:

Windows
=======

A complete windows installer (package) for MAVProxy is available at
http://firmware.ardupilot.org/Tools/MAVProxy/. It is compatible
with Windows 7 or later.

A link to MAVProxy will be available on the start menu. MAVProxy can
also be called from the commandline via mavproxy.exe

.. note::
    Any references to "mavproxy.py" in this documentation should be
    replaced by "mavproxy.exe" on the Windows commandline.

.. note::

    Please note that the "missionedit" module does not currently run on the
    Windows package of MAVProxy. This is being investigated and will
    hopefully be fixed soon. See https://github.com/ArduPilot/MAVProxy/issues/129 for more details.

Alternatively, Windows 10 users can use the Windows Subsystem for Linux (WSL) to run MAVProxy from a Linux environment
within Windows. It generally provides better stability than the native Windows installer above.


.. _mavproxy-downloadinstalllinux:

Linux
=====

MAVProxy can be installed via package managers. Select the appropriate set of commands for your Linux variant below.

For Debian based systems (including WSL):

.. code:: bash

    sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml
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


.. note::

    On older Linux installations, the ``python3-wxgtk3.0`` package will need to be installed if the 
    ``python3-wxgtk4.0`` package does not exist.

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



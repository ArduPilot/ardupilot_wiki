.. _mavproxy-downloadinstall:

=========================
Download and Installation
=========================

The following instructions are for user that just want to run MAVProxy. Developers should look at the :ref:`Developer's Guide <mavproxy-development>` for setting up the development environment.

MAVProxy is distributed as a Python package (see `PyPI <https://pypi.org/project/MAVProxy/>`__) and can run under Python 2 or 3.
A number of pre-requisite packages are required to run MAVProxy, as detailed in the individiual sections below.

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

MAVProxy runs within Python 2 or 3. If your operating system version is less than 2 years old, it
is recommended to use the Python 3 environment. Otherwise use Python 2.

For Python 3 on Debian based systems (including Ubuntu, WSL, Raspian):

.. code:: bash

    sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
    pip3 install PyYAML mavproxy --user
    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc

For Python 2 on Debian based systems (including Ubuntu, WSL, Raspian):

.. code:: bash

    sudo apt-get install python-dev python-opencv python-wxgtk4.0 python-pip python-matplotlib python-lxml python-pygame
    pip install PyYAML mavproxy --user
    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
    
For Python 3 on Fedora based systems:

.. code:: bash

    sudo dnf install python3-devel python3-opencv python3-wxpython4 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml redhat-rpm-config
    pip3 install PyYAML mavproxy --user
    echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc

If you get a "permission denied" error message when connecting to serial devices,
the user permissions may need to be changed:

.. code:: bash

    sudo usermod -a -G dialout <username>

The system will need to be restarted to apply the above change.

Updating
--------

To update an existing installation with the current release on Python 3 based systems:

.. code:: bash

    pip3 install mavproxy --user --upgrade

or for Python 2 systems:

.. code:: bash

    pip install mavproxy --user --upgrade

To update an existing installation with the current development version (ie, from its master branch) for Python 3 systems:

.. code:: bash

    pip3 install mavproxy --user git+https://github.com/ArduPilot/mavproxy.git@master

or for Python 2 systems:

.. code:: bash

    pip install mavproxy --user git+https://github.com/ArduPilot/mavproxy.git@master

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



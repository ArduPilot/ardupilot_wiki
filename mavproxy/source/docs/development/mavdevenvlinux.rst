=============================
Linux Development Environment
=============================

Follow the user installation instructions as per the
:ref:`here <mavproxy-downloadinstall>` for the prerequisite libraries. Additionally, install:

.. code:: bash

    sudo apt-get install git

The pip-installed MAVProxy will need to uninstalled (if already installed) to prevent system conflicts:

.. code:: bash

    pip uninstall MAVProxy

Use git to download the MAVProxy source:

.. code:: bash

    git clone https://github.com/ArduPilot/MAVProxy.git

After making the desired changes, MAVProxy is required to be installed
(any change to the modules won't work otherwise). This needs to happen after any
changes to the source code. This can be done by:

.. code:: bash

    python setup.py build install --user

MAVProxy can then be run as per normal.

=============================
Linux Development Environment
=============================

Follow the user installation instructions as per the
:ref:`here <mavproxy-downloadinstall>` for the prerequisite libraries. Additionally, install:

.. code:: bash

    sudo apt-get install git

The pip-installed MAVProxy will need to uninstalled (if already installed) to prevent system conflicts:

.. code:: bash

    python3 -m pip uninstall MAVProxy

Use git to download the MAVProxy source:

.. code:: bash

    git clone https://github.com/ArduPilot/MAVProxy.git



.. tabs::

   .. group-tab:: User Python Environment

    After making the desired changes, MAVProxy is required to be installed
    (any change to the modules won't work otherwise). This needs to happen after any
    changes to the source code. This can be done by:

      .. code-block:: bash

        python -m pip install .


   .. group-tab:: Virtual Python Environment

    Using a virtual environment allows quicker editing.
    If you make code changes, just re-run MAVProxy.
    We give access to system site packages because
    compiling wx in a virtual environment is extremely difficult.
    Instead, rely on the host OS to supply that.

      .. code-block:: bash

        python3 -m venv .venv --system-site-packages
        source .venv/bin/activate
        python -m pip install --editable .

MAVProxy can then be run as per normal.

.. code:: bash

    which mavproxy.py
    mavproxy.py --help

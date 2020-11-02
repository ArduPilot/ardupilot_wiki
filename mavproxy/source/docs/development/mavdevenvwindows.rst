===============================
Windows Development Environment
===============================

You will need to have `Python <http://www.python.org>`_ 2 or 3. Python 3 is preferred.

It is assumed that Python and pip are on the system path. If not, see `here <https://datatofish.com/add-python-to-windows-path/>`_ for instructions.

To install the required libraries:

.. code:: bash

    pip install pywin32 lxml pymavlink numpy matplotlib==3.2.2 pyserial opencv-python PyYAML Pygame Pillow wxpython prompt-toolkit
    pip install pyinstaller setuptools packaging --no-use-pep517
    
Download the MAVProxy `source <https://github.com/ArduPilot/MAVProxy>`_.

After making the desired changes, MAVProxy is required to be compiled 
into the Python directory (the modules won't work otherwise).
This needs to happen after any changes to the source code. This can be
done by running the :file:`./MAVProxy/MAVProxyWinUSB.bat` or 
:file:`./MAVProxy/MAVProxyWinLAN.bat` file. This will
perform the necessary build actions and then run MAVProxy. Some of the 
details in the batch files (port numbers, etc) may need to be altered to 
match the user's system configuration.

To create a one-click windows installer for MAVProxy, run ``MAVProxyWinBuild.bat```, 
which is in the ``./windows`` directory. The installer will be created in the 
``./windows/output`` directory. The `Inno Setup <http://www.jrsoftware.org/isdl.php#stable>`_ 
program will be required for this process and is assumed to be installed in the 
``C:\Program Files (x86)\Inno Setup 6\`` folder


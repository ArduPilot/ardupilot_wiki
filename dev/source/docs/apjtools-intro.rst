.. _apjtools-intro:

=======================================
APJ Tools for Custom Parameter Defaults
=======================================

`APJ Tools <https://github.com/ArduPilot/ardupilot/blob/master/Tools/scripts/apj_tool.py>`__ is a command-line `Python <https://www.python.org/downloads/>`__ program which can be used to add custom default parameter values to a pre-built ArduPilot firmware.  It can also mark these parameter as read-only to avoid users accidentally overwriting them.  This can help companies reduce the time required to set-up many identical vehicles.

How to Install APJ Tools
------------------------

- `Download and Install Python <https://www.python.org/downloads/>`__ if not already installed on your machine
- Use a web browser to open `APJ Tools <https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/scripts/apj_tool.py>`__ in the ArduPilot GitHub repository and then right-mouse-button click and save to a file on your PC

How to update the Defaults
--------------------------

- Find and download the ArduPilot firmware you wish to customise from `firmware.ardupilot.org <https://firmware.ardupilot.org/>`__.  For example the stable version of ArduPilot Copter for the Hex Cube Black is the "arducopter.apj" file found at `firmware.ardupilot.org/Copter/stable/CubeBlack/ <https://firmware.ardupilot.org/Copter/stable/CubeBlack>`__.
- Place the above .apj file in the same directory as the apj_tool.py file
- Use a text editor to create a parameters file (maybe called "param-defaults.parm") of this format

::

    # Default parameter file for vehicleX
    MOT_PWM_MIN 1000
    ATC_RAT_RLL_P 0.100 @READONLY   # users will not be able to modify this parameter

- Embed the parameter defaults in the .apj firmware with this command

::

    python apj_tool.py --set-file param-defaults.parm arducopter.apj

- Check the defaults have been applied correctly with this command

::

    python apj_tool.py --show arducopter.apj

- Load the modified .apj file to your vehicle and check the defaults have worked correctly

Additional Info
---------------

- APJ Tool's help can be displayed using this command

::

    python apj_tool.py --help

- The parameters will be the new default values only if the user has not modified them
- If all parameters are reset to their defaults, they will return to the values specified in the file
- The @READONLY modifier will stop users from modifying the parameter as long as they use this particular firmware.  The users can get around this though if they go to the extra effort of loading the standard ArduPilot firmware to the autopilot
- The new parameter default file can be a maximum of 8k including comments

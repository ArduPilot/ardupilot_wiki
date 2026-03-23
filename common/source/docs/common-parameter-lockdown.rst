.. _common-parameter-lockdown:

==================
Parameter Lockdown
==================

To avoid accidental changes to critical parameters, ArduPilot provides a parameter lockdown feature using the `param_set.lua <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/param-set.lua>`__ Lua applet.

As with other Lua scripts this requires an autopilot with a fast CPU and sufficient memory (e.g. STM32H7).

Please note that this is not an iron clad security feature because some ground stations (including Mission Planner) allow users to stop and start scripts meaning the user could stop the script and then change parameter values.

Setup
-----

Details on the setup can be found in the `param-set.md file <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/param-set.md>`__ but in short this involves:

- Follow the :ref:`Lua Script setup instructions <common-lua-scripts>` to set up Lua scripting on the autopilot including setting :ref:`SCR_ENABLE <SCR_ENABLE>` = 1
- Download `param-set.lua <https://raw.githubusercontent.com/ArduPilot/ardupilot/refs/heads/master/libraries/AP_Scripting/applets/param-set.lua>`__ from the ArduPilot repository
- Copy the script to the autopilot's SD card's /APM/scripts/ directory and restart the autopilot
- Confirm the script has loaded by looking for "param-set script loaded" in the GCS's messages tab
- Confirm parameters cannot be set by using the GCS's full parameter list to change a single parameter value. If using Mission Planner it may be necessary to reload parameters and then check if the parameter value has actually changed or not

Customising the Script
----------------------

The list of parameters that may be changed is called the "white list" and can be found `here in the param-set script <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/param-set.lua#L120>`__

.. code-block:: lua

    -- handle PARAM_SET message
    local parameters_which_can_be_set = {}
    parameters_which_can_be_set["MAV_OPTIONS"] = true
    parameters_which_can_be_set["PARAM_SET_ENABLE"] = true
    parameters_which_can_be_set["BATT_ARM_MAH"] = true

This list can be customised by adding or removing lines from the list and then re-uploading the script to the autopilot

[copywiki destination="copter,plane,rover,sub,dev"]

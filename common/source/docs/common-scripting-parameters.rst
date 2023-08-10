.. _common-scripting-parameters:

=======================================
Accessing/Adding Parameters via Scripts
=======================================

LUA scripts have the ability to access and change ArduPilot parameters setup by the firmware.

for example: ::

    local parameter = parm:get('SCR_ENABLE')

will retrieve the SCR_ENABLE parameter and assign it to the variable "parameter"

However, looking up parameters with a name search, as the above code does, is slow.  You can create a local variable using the Parameter user data method which avoids a name search every use and is much faster: ::

   local SCR_ENABLE = Parameter()      --creates a parameter object
   SCR_ENABLE:init('SCR_ENABLE')       --get the physical location in parameters memory area so no future name search is done
   local parameter = SCR_ENABLE:get()  --retrieve that parameters value and assign to "parameter"

the current parameter access/modify methods, using the name searching param: methods:

- get(name)                  -retrieves the value
- set(name, value)           -sets the value but does not store in non-volatile
- set_and_save(name, value)  -sets value and saves in non-volatile
- set_default(name, value)   -changes the default value

For Parameter objects the above access methods apply also, but without the name arguments since the Parameter object should already be initialized and no slow name search is required, and the following methods are applicable:

- init(name)                          -initializes a parameter object
- configured()                        -check to see if a Parameter object has been initialized

Adding Parameters
=================

In addition to accessing/modifying existing parameters created by the firmware, LUA scripts can add new parameters for use by any LUA script. These parameters behave the same as those created and used by the firmware in that they can be read/written and saved into non-volatile memory that persists across reboots and power cycles.

However, unlike firmware parameters they behave slightly differently. Firmware parameters are reset to their default values upon a parameter reset or when changing firmware to a different vehicle type. Scripting created parameters do not. In fact, scripting created parameters are invisible unless a script actually "re-creates" them within a script.

This is explained below in the section :ref:`How Parameters Work in ArduPilot <how-params-work>`. Suffice it to say that there must a script running that "creates" the new parameters as discussed below on any given bootup, in order to use them and for ground control stations like Mission Planer to list them.

Creating a Parameter Group
--------------------------

Groups of up to 63 parameters can created within a script using the "add_table" command. :: 

   local PARAM_TABLE_KEY = 72
   assert(param:add_table(PARAM_TABLE_KEY, "AERO_", 30), 'could not add param table')

This defines a unique table key (0 - 200) to identify the parameter storage area in memory, so that different scripts can have different parameters, if desired. The next command creates a 30 slot table for parameters with names beginning "AERO\_" and uses the LUA assert error command to halt the script if it cannot be created for some reason (like lack of memory space or conflicts with another script trying to add a table with the same key)

Next, individual parameters can be added. ::

   assert(param:add_param(PARAM_TABLE_KEY, 1,  'TRICK_ID', 0), 'could not add param1')
   assert(param:add_param(PARAM_TABLE_KEY, 2,  'RPT_COUNT', 0), 'could not add param2')

This adds two parameters, AERO_TRICK_ID and AERO_RPT_COUNT, with indexes 1 and 2, and initial values of 0 to that table. Other concurrently running scripts could also add other parameters as long as they do not overlap index numbers. Those scripts would use the "param:add_table" method with the same prefix ("AERO" in the above example) and key, and then add parameters with "param"add_param" methods as above. Calling the "param:add_table" with the same key and prefix in two concurrently running scripts does not create an error.

.. note:: if there are other con-currently running scripts that will try to access these parameters, be sure to delay their first updates until the script adding the parameters has time to complete the setup process since each script is run independently in its own thread

These may now be accessed just like any other parameter. ::

    local TRICK_ID = Parameter()
    TRICK_ID:init('AERO_TRICK_ID')
    local trick_id = TRICK_ID:get()

.. note:: all scripting defined parameters are of type FLOAT, ie floating point numbers

.. _how-params-work:

How Parameters Work in ArduPilot
================================

Parameters are defined in the firmware. Each parameter has a name, a default value, and value type(boolean,float, integer 8,etc).
Upon boot up, the firmware creates RAM variables for each parameter and initializes them with a value. That value is the default if the parameter has never been changed by a user, or a value that has been stored in EEPROM of the autopilot, if it has been changed from default. If the parameters have been "reset" by the user (see :ref:`common-parameter-reset`) then the EEPROM storage for changes is blank and all default values are used.

The string names of the parameters are only used by the Ground Control Stations and LUA scripts. Access from within the C++ code is directly to addresses of the initialized parameter variables in RAM while executing, not by name strings.

LUA created parameters exist in the same manner as firmware created parameters. User/Script changes are stored in non-volatile memory. But their names only exist for a lookup if a "param:add_table" and "param:add_param" has been executed after bootup by a script. Then they will appear in the GCS parameter list and can be accessed by name from LUA scripts. Without this, the data would still exist in non-volatile memory, but is invisible and non-existent for all intents and purposes.

This also means that parameters created by one script, which are modified by a user or script, are still in storage. If that script is removed, another script with different parameters is run, then removed, and the original script re-run, those changed parameters will re-appear in the GCS with their last values and can be accessed again by the script. Resetting all the parameters to defaults will erase those changes, just like the firmware created parameters.

From LUA scripts you can access any parameter (created by firmware or by LUA script) by name. However, that is slow since the LUA interpreter must do a long string name search each time. Instead, using a Parameter() object and initializing it, gives the address pointers to that named parameter's location for direct accessing it quickly.

[copywiki destination="copter,plane,rover,dev"]

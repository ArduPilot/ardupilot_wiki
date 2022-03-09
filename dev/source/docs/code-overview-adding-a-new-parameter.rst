.. _code-overview-adding-a-new-parameter:

================================
Adding a New Parameter to Copter
================================

Parameters can either be part of the main code or part of a library.

Adding a parameter to the main code
===================================

**Step #1:** Find a spare slot in the Parameters class's enum in
`Parameters.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Parameters.h>`__
and add your new parameter as shown below in red.  Some things to be
careful of:

-  Try to add the parameter close to parameters that share a similar
   function or worst case add to the end of the "Misc" section.
-  Ensure that the section you are adding it to is not full.  You can
   check if the section is full by counting the number of parameters in
   the section and ensuring that the last element is less than the next
   sections first element.  I.e. the Misc section's first parameter is
   #20.  my_new_parameter is #36.  If the next section began at #36 we
   would not be able to add the new parameter here.
-  Do not add a parameter in the middle of a group thus causing the
   slots for existing parameters to change
-  Do not use slots with "deprecated" or "remove" comments beside them
   because some users may still have the defaults for these old
   parameters in their eeprom so your new parameter's default value
   could be set strangely.

::

        enum {
            // Misc
            //
            k_param_log_bitmask = 20,
            k_param_log_last_filenumber,            // *** Deprecated - remove
                                                    // with next eeprom number
                                                    // change
            k_param_toy_yaw_rate,                           // THOR The memory
                                                            // location for the
                                                            // Yaw Rate 1 = fast,
                                                            // 2 = med, 3 = slow

            k_param_crosstrack_min_distance,    // deprecated - remove with next eeprom number change
            k_param_rssi_pin,
            k_param_throttle_accel_enabled,     // deprecated - remove
            k_param_wp_yaw_behavior,
            k_param_acro_trainer,
            k_param_pilot_velocity_z_max,
            k_param_circle_rate,
            k_param_sonar_gain,
            k_param_ch8_option,
            k_param_arming_check_enabled,
            k_param_sprayer,
            k_param_angle_max,
            k_param_gps_hdop_good,          // 35
            k_param_my_new_parameter,       // 36

**Step #2:** Declare the variable within the Parameters class somewhere
below the enum.  Possible types include AP_Int8, AP_Int16, AP_Float,
AP_Int32 and AP_Vector3 (Note: unsigned integers are not currently
supported).  The name of the variable should be the same as the new enum
but with the "k_param\_" prefix removed.

::

            // 254,255: reserved
        };

        AP_Int16        format_version;
        AP_Int8         software_type;

        // Telemetry control
        //
        AP_Int16        sysid_this_mav;
        AP_Int16        sysid_my_gcs;
        AP_Int8         serial3_baud;
        AP_Int8         telem_delay;

        AP_Int16        rtl_altitude;
        AP_Int8         sonar_enabled;
        AP_Int8         sonar_type;       // 0 = XL, 1 = LV,
                                          // 2 = XLL (XL with 10m range)
                                          // 3 = HRLV
        AP_Float        sonar_gain;
        AP_Int8         battery_monitoring;         // 0=disabled, 3=voltage only,
                                                    // 4=voltage and current
        AP_Float        volt_div_ratio;
        AP_Float        curr_amp_per_volt;
        AP_Int16        pack_capacity;              // Battery pack capacity less reserve
        AP_Int8         failsafe_battery_enabled;   // battery failsafe enabled
        AP_Int8         failsafe_gps_enabled;       // gps failsafe enabled
        AP_Int8         failsafe_gcs;               // ground station failsafe behavior
        AP_Int16        gps_hdop_good;              // GPS Hdop value below which represent a good position
        AP_Int16        my_new_parameter;                  // my new parameter's description goes here

**Step #3:** Add the variable declaration to the var_info table in
`Parameters.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Parameters.cpp>`__

::

        // @Param: MY_NEW_PARAMETER
        // @DisplayName: My New Parameter
        // @Description: A description of my new parameter goes here
        // @Range: -32768 32767
        // @User: Advanced
        GSCALAR(my_new_parameter, "MY_NEW_PARAMETER", MY_NEW_PARAMETER_DEFAULT),

The @Param ~ @User comments are used by the ground station (i.e. Mission
Planner) to display advice to the user and limit the values that the
user may set the parameter to.

The parameter name (i.e. "MY_NEW_PARAMETER") is limited to 16
characters.

**Step #4:** Add the parameters default to
`config.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/config.h>`__

::

    #ifndef MY_NEW_PARAMETER_DEFAULT
     # define MY_NEW_PARAMETER_DEFAULT      100     // default value for my new parameter
    #endif

You're done!  The new parameter can be access from the main code (not
the libraries) as g.my_new_parameter.

.. note:: ArduPilot vehicle's global parameter classes are crowded. To avoid exceeding the maximum number of parameters in a vehicle's Parameter class, another class for global parameters (named 'ParametersG2') has been introduced. Adding parameters to this class is similar to adding parameters in libraries. The parameters added to this class can similarly be accessed from within the vehicle's main code.

Adding a parameter to a library
===============================

Parameters can also be added to libraries by following these steps which
use the `AP_Compass <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Compass>`__
library as an example.

**Step #1:** Add the new class variable to the top level .h file (i.e.
`Compass.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/AP_Compass.h>`__).
Possible types include AP_Int8, AP_Int16, AP_Float, AP_Int32 and
AP_Vector3f.  Also add the default value you'd like for the parameter
(we will use this in step #2)

::

    #define MY_NEW_PARAM_DEFAULT         100

    class Compass
    {
    public:
        int16_t product_id;                         /// product id
        int16_t mag_x;                      ///< magnetic field strength along the X axis
        int16_t mag_y;                      ///< magnetic field strength along the Y axis
        int16_t mag_z;                      ///< magnetic field strength along the Z axis
        uint32_t last_update;               ///< micros() time of last update
        bool healthy;                               ///< true if last read OK

        /// Constructor
        ///
        Compass();

    protected:
        AP_Int8 _orientation;
        AP_Vector3f _offset;
        AP_Float _declination;
        AP_Int8 _use_for_yaw;                       ///<enable use for yaw calculation
        AP_Int8 _auto_declination;                  ///<enable automatic declination code
        AP_Int16 _my_new_lib_parameter;              /// description of my new parameter
    };

**Step #2:**\ Add the variable to the var_info table in the .cpp file
(i.e. `Compass.cpp <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Compass/AP_Compass.cpp>`__)
including @Param ~ @Increment comments to allow the GCS to display the
description to the user and to control the min and max values set from
the ground station.  When adding the new parameter be careful that:

    -  The slot (i.e. "9" below) is one higher than the previous
       variable.
    -  the parameter's name (i.e. MY_NEW_P) length is less than 16
       including the object's prefix that will be added.  I.e. the
       compass object's prefix is "COMPASS\_".

::

    const AP_Param::GroupInfo Compass::var_info[] = {
        // index 0 was used for the old orientation matrix

        // @Param: OFS_X
        // @DisplayName: Compass offsets on the X axis
        // @Description: Offset to be added to the compass x-axis values to compensate for metal in the frame
        // @Range: -400 400
        // @Increment: 1

    <snip>

        // @Param: ORIENT
        // @DisplayName: Compass orientation
        // @Description: The orientation of the compass relative to the autopilot board.
        // @Values: 0:None,1:Yaw45,2:Yaw90,3:Yaw135,4:Yaw180,5:Yaw225,6:Yaw270,7:Yaw315,8:Roll180
        AP_GROUPINFO("ORIENT", 8, Compass, _orientation, ROTATION_NONE),

        // @Param: MY_NEW_P
        // @DisplayName: My New Library Parameter
        // @Description: The new library parameter description goes here
        // @Range: -32768 32767
        // @User: Advanced
        AP_GROUPINFO("MY_NEW_P", 9, Compass, _my_new_lib_parameter, MY_NEW_PARAM_DEFAULT),

        AP_GROUPEND
    };

The parameter can be accessed from within the library as
\_my_new_lib_parameter.  Note that we made the parameter "protected"
so it cannot be access from outside the class.  If we'd made it public
then it would have been accessible to the main code as well as
"compass._my_new_lib_parameter".

**Step #3:** Add a declaration for var_info to the public section of the .h file of new library class in addition to the definition in the .cpp file:

::

    static const struct AP_Param::GroupInfo var_info[];

**Step #4:** If the class is a completely new addition to the code (as
opposed to an existing class like AP_Compass), it should be added to
the main vehicle's var_info table in the
`Parameters.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Parameters.cpp>`__
file (its order in the var_info table is not important).  Below in red
where the Compass class appears.

::

    const AP_Param::Info var_info[] = {
        // @Param: FORMAT_VERSION
        // @DisplayName: Eeprom format version number
        // @Description: This value is incremented when changes are made to the eeprom format
        // @User: Advanced
        GSCALAR(format_version, "FORMAT_VERSION",   0),
    <snip>

        // @Group: COMPASS_
        // @Path: ../libraries/AP_Compass/Compass.cpp
        GOBJECT(compass,        "COMPASS_", Compass),

    <snip>
        // @Group: INS_
        // @Path: ../libraries/AP_InertialSensor/AP_InertialSensor.cpp
        GOBJECT(ins,            "INS_", AP_InertialSensor),

        AP_VAREND
    };

**Step #5:**
If the class is a completely new addition to the code, also add k_param_my_new_lib to the enum in `Parameters.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Parameters.h>`__, where my_new_lib is the first argument to the GOBJECT declaration in `Parameters.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Parameters.cpp>`__. Read the comments above the enum to understand where to place the new value, as order is important here.


Changing the type of a parameter
================================

From time to time parameters need to be altered or renamed. ArduPilot has capabilities that allow this to be managed from release to release so that upgrades always preserve user configured parameters. Parameters are keyed off slots in the eeprom, so if the occupied slot does not change then the parameter does not change - regardless of its name. If however the type needs to change then the parameter needs to be moved to a new slot and the existing slot be reserved to prevent unexpected configuration. In order for the configuration to be preserved it needs to be copied and converted from the old slot to the new slot.

**Step #1:** Change the index of the existing parameter to an unused index and add a comment to the effect that the old slot is reserved.

**Step #2:** In order to figure out the keys for conversion:

    * Set the AP_PARAM_KEY_DUMP definition to "1" `here in AP_Param.h <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Param/AP_Param.h#L36>`__

    * Start the old code in SITL and all the parameter names and their magic numbers will be displayed

    * Copy-paste from Copter's Parameters.cpp file's `existing parameter conversion tables like the ones done for the attitude controller gains <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Parameters.cpp#L1247>`__.

Parameter Conversion Expiration
===============================

Parameter conversions are necessary to ensure continuity for users as they upgrade to newer versions of ArduPilot.  However, the conversion process does utilise some resources on boot and does add to the flash cost of the code. 
Therefore we don't keep the conversion code forever more. To make it easier to find and remove the conversions, please add the following as a comment above any parameter conversion that you add:

::

    // PARAMETER_CONVERSION - Added: Aug-2021

Please use the date format shown above to avoid ambiguity.  Please also keep the date on the same line as the key word "PARAMETER_CONVERSION" so that the date can be seen in the return, when grep-ing/searching the code.  When adding to an already existing AP_Param::ConversionInfo table, please add the conversions as a new block with the conversion comment above the new block.  For example:

.. code-block:: cpp

    const AP_Param::ConversionInfo q_conversion_table[] = {
        // PARAMETER_CONVERSION - Added: Jan-2018
        { Parameters::k_param_quadplane, 4044, AP_PARAM_FLOAT, "Q_P_POSZ_P" },     //  Q_PZ_P
        { Parameters::k_param_quadplane, 4045, AP_PARAM_FLOAT, "Q_P_POSXY_P"},     //  Q_PXY_P
        { Parameters::k_param_quadplane, 4046, AP_PARAM_FLOAT, "Q_P_VELXY_P"},     //  Q_VXY_P
        { Parameters::k_param_quadplane, 78,   AP_PARAM_FLOAT, "Q_P_VELXY_I"},     //  Q_VXY_I
        { Parameters::k_param_quadplane, 142,  AP_PARAM_FLOAT, "Q_P_VELXY_IMAX"},  //  Q_VXY_IMAX

        // PARAMETER_CONVERSION - Added: Aug-2021
        { Parameters::k_param_quadplane, 206,  AP_PARAM_FLOAT, "Q_P_VELXY_FLTE"},  //  Q_VXY_FILT_HZ

        // PARAMETER_CONVERSION - Added: Jan-2018
        { Parameters::k_param_quadplane, 4047, AP_PARAM_FLOAT, "Q_P_VELZ_P"},      //  Q_VZ_P
        { Parameters::k_param_quadplane, 4048, AP_PARAM_FLOAT, "Q_P_ACCZ_P"},      //  Q_AZ_P
        { Parameters::k_param_quadplane, 80,   AP_PARAM_FLOAT, "Q_P_ACCZ_I"},      //  Q_AZ_I
        { Parameters::k_param_quadplane, 144,  AP_PARAM_FLOAT, "Q_P_ACCZ_D"},      //  Q_AZ_D
        { Parameters::k_param_quadplane, 336,  AP_PARAM_FLOAT, "Q_P_ACCZ_IMAX"},   //  Q_AZ_IMAX

        // PARAMETER_CONVERSION - Added: Jun-2019
        { Parameters::k_param_quadplane, 400,  AP_PARAM_FLOAT, "Q_P_ACCZ_FLTD"},   //  Q_AZ_FILT


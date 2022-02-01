.. _common-lua-API:

[copywiki destination="plane,copter,rover,dev"]
=====================================
API Documentation
=====================================

The API documentation described here is not a complete list, but rather some examples.  For a **full list of the methods** currently available, the LUA auto-generated document file is a complete list, It can be found `here <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/docs/docs.lua>`__ and lists all the available bindings and their parameters.

See :ref:`API Docs Syntax<common-lua-API>`.

.. note:: If you use VScode for your editor, by installing `this <https://marketplace.visualstudio.com/items?itemName=sumneko.lua>`__ lua extension (version 2.4.1 only as of now), you can get this information integrated as suggestions and autocomplete in the editor. 

For complete information on how a binding works the source can be found in the `binding generator descriptions <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/generator/description/bindings.desc>`_ .

See :ref:`Binding Description Method Syntax<common-lua-binding-syntax>` for more information on how to decode the methods shown there. ArduPilot uses Lua version 5.3.5.

Location
~~~~~~~~

Location is a userdata object that holds locations expressed as latitude, longitude, altitude. The altitude can be in several different frames, relative to home, absolute altitude above mean sea level, or relative to terrain. To create a new Location userdata you can call Location() to allocate an empty location object, or call a method that returns one to you.

A Location userdata object supports the following calls:

- :code:`lat( [new_lat] )` - If called with no arguments, returns the current latitude in degrees * 1e7 as an integer. If called with one argument, it will assign the value to the latitude.

- :code:`lng( [new_lng] )` - If called with no arguments, returns the current longitude in degrees * 1e7 as an integer. If called with one argument, it will assign the value to the longitude.

- :code:`alt( [new_alt] )` - If called with no arguments, returns the current altitude in cm as an integer. If called with one argument, it will assign the value to the altitude.

- :code:`relative_alt( [is_relative] )` - If called with no arguments returns true if the location is planned as relative to home. If called with a boolean value this will set the relative altitude.

- :code:`terrain_alt( [is_terrain] )` - If called with no arguments returns true if the location is planned as relative to terrain. If called with a boolean value this will flag if the altitude should be relative to terrain.

- :code:`origin_alt( [is_origin] )` - If called with no arguments returns true if the location is planned in absolute altitude relative to mean sea level. If called with a boolean value this will set the altitude to be relative to mean sea level.

- :code:`loiter_xtrack( [should_xtrack] )` - If the location is used for a loiter location this flags if the aircraft should track from the center point, or from the exit location of the loiter.

- :code:`get_distance( other_Location )` - Given a Location this calculates the horizontal distance between the two locations in meters.

- :code:`offset( offset_north ,  offset_east )` - Translates this Location by the specified number of meters.

- :code:`get_vector_from_origin_NEU()` - Returns nil or Vector3f that contains the offset from the EKF origin to this location. This will return nil if the EKF origin wasn’t available at the time this was called.

- :code:`get_bearing( other_Location )` - Given a Location this calculates the relative bearing to the location in radians

- :code:`get_distance_NED( other_Location )` - Returns nil or Vector3f that contains the 3D vector in meters to the other Location.

- :code:`get_distance_NE( other_Location )` - Returns nil or Vector2f that contains the 2D vector in meters to the other Location.

Vector2f
~~~~~~~~

Vector2f is a userdata object that holds a 2D vector. The components are stored as floating point numbers. To create a new Vector2f you can call Vector2f() to allocate a new one, or call a method that returns one to you.

A Vector2f object supports the following calls:

- :code:`x( [new_x] )` - If called with no arguments, return the currently set X component as a floating point number. If called with one argument it will assign the value to the X component.

- :code:`y( [new_y] )` - If called with no arguments, return the currently set Y component as a floating point number. If called with one argument it will assign the value to the Y component.

- :code:`length()` - Returns the length of the vector as a floating point number.

- :code:`normalize()` - Normalizes the vector to be a unit vector.

- :code:`is_nan()` - Returns true if the vector contains any NaN members.

- :code:`is_inf()` - Returns true if the vector contains any infinity members.

- :code:`is_zero()` - Returns true if all the vector fields are 0.

The following operators are supported on a Vector2f:

- :code:`+` - Adds the two vectors by components.

- :code:`-` - Subtracts the two vectors by components.

Vector3f
~~~~~~~~

Vector3f is a userdata object that holds a 3D vector. The components are stored as floating point numbers. To create a new Vector3f you can call Vector3f() to allocate a new one, or call a method that returns one to you.

A Vector3f object supports the following calls:

- :code:`x( [new_x] )` - If called with no arguments, return the currently set X component as a floating point number. If called with one argument it will assign the value to the X component.

- :code:`y( [new_y] )` - If called with no arguments, return the currently set Y component as a floating point number. If called with one argument it will assign the value to the Y component.

- :code:`z( [new_z] )` - If called with no arguments, return the currently set Z component as a floating point number. If called with one argument it will assign the value to the Z component.

- :code:`length()` - Returns the length of the vector as a floating point number.

- :code:`normalize()` - Normalizes the vector to be a unit vector.

- :code:`is_nan()` - Returns true if the vector contains any NaN members.

- :code:`is_inf()` - Returns true if the vector contains any infinity members.

- :code:`is_zero()` - Returns true if all the vector fields are 0.

The following operators are supported on a Vector3f:

- :code:`+` - Adds the two vectors by components.

- :code:`-` - Subtracts the two vectors by components.

AHRS (ahrs:)
~~~~~~~~~~~~

The ahrs library represents the Attitude Heading Reference System computed by the autopilot. It provides estimates for the vehicles attitude, and position.

- :code:`get_roll()` - Returns the current vehicle roll angle in radians.

- :code:`get_pitch()` - Returns the current vehicle pitch angle in radians.

- :code:`get_yaw()` - Returns the current vehicle yaw angle in radians.

- :code:`get_position()` - Returns nil or Location userdata that contains the vehicles current position. Note: This will only return a Location if the system considers the current estimate to be reasonable.

- :code:`get_home()` - Returns a Location that contains the vehicles current home waypoint.

- :code:`get_gyro()` - Returns a Vector3f containing the current smoothed and filtered gyro rates (in radians/second)

- :code:`get_hagl()` - Returns nil, or the latest altitude estimate above ground level in meters

- :code:`wind_estimate()` - Returns a Vector3f containing the current wind estimate for the vehicle.

- :code:`groundspeed_vector()` - Returns a Vector2f containing the vehicles velocity in meters/second in north and east components.

- :code:`get_velocity_NED()` - Returns nil, or a Vector3f containing the current NED vehicle velocity in meters/second in north, east, and down components.

- :code:`get_velocity_NE()` - Returns nil, or a Vector2f containing the current NE vehicle velocity in meters/second in north and east components.

- :code:`home_is_set()` - Returns a true if home position has been set.

- :code:`prearm_healthy()` - Returns a true if current pre-arm checks are passing.

- :code:`airspeed_estimate()` - Returns current airspeed estimate in meters/second or nil.


Arming (arming:)
~~~~~~~~~~~~~~~~

The Arming library provides access to arming status and commands.

- :code:`disarm()` - Disarms the vehicle in all cases. Returns false only if already disarmed.

- :code:`is_armed()` -Returns a true if vehicle is currently armed.

- :code:`arm()` - Attempts to arm the vehicle. Returns true if successful.


Battery (battery:)
~~~~~~~~~~~~~~~~~~

The battery library provides access to information about the currently connected batteries on the vehicle.

- :code:`num_instances()` - Returns the number of battery instances currently available.

- :code:`healthy( instance )` - Returns true if the requested battery instance is healthy. Healthy is considered to be ArduPilot is currently able to monitor the battery.

- :code:`voltage( instance )` - Returns the voltage of the selected battery instance.

- :code:`voltage_resting( instance )` - Returns the estimated battery voltage if it was not under load.

- :code:`current_amps( instance )` - Returns the current (in Amps) that is currently being consumed by the battery, or nil if current monitoring is not available.

- :code:`consumed_mah( instance )` - Returns the capacity (in milliamp hours) used from the battery, or nil if current monitoring is not available.

- :code:`consumed_wh( instance )` - Returns the used watt hours from the battery, or nil if energy monitoring is not available.

- :code:`capacity_remaining_pct( instance )` - Returns the remaining percentage of battery (from 0 to 100)

- :code:`pack_capacity_mah( instance )` - Returns the full pack capacity (in milliamp hours) from the battery.

- :code:`has_failsafed()` - Returns true if any of the batteries being monitored have triggered a failsafe.

- :code:`overpower_detected( instance )` - Returns true if too much power is being drawn from the battery being monitored.

- :code:`get_temperature( instance )` - Returns the temperature of the battery in degrees Celsius if the battery supports temperature monitoring.

- :code:`get_cycle_count( instance )` - Returns cycle count of the battery or nil if not available.


GPS (gps:)
~~~~~~~~~~

The GPS library provides access to information about the GPS's on the vehicle.

- :code:`num_sensors()` - Returns the number of connected GPS devices. If GPS blending is turned on that will show up as the third sensor, and be reported here.

- :code:`primary_sensor()` - Returns which GPS is currently being used as the primary GPS device.

- :code:`status(instance)` - Returns the GPS fix status. Compare this to one of the GPS fix types (GPS.NO_GPS, GPS.GPS_OK_FIX_2D, GPS.GPS_OK_FIX_3D GPS.GPS_OK_FIX_3D_DGPS GPS.GPS_OK_FIX_3D_RTK_FLOAT GPS.GPS_OK_FIX_3D_RTK_FIXED

- :code:`location( instance )` - Returns a Location userdata for the last GPS position. You must check the status to know if the location is still current, if it is NO_GPS, or NO_FIX then it will be returning old data.

- :code:`speed_accuracy( instance )` - Returns nil, or the speed accuracy of the GPS in meters per second, if the information is available for the GPS instance.

- :code:`horizontal_accuracy( instance )` - Returns nil, or the horizontal accuracy of the GPS in meters, if the information is available for the GPS instance.

- :code:`vertical_accuracy( instance )` - Returns nil, or the vertical accuracy of the GPS in meters, if the information is available for the GPS instance.

- :code:`velocity( instance )` - Returns a Vector3f that contains the velocity as observed by the GPS. You must check the status to know if the velocity is still current.

- :code:`ground_speed( instance )` - Returns the ground speed of the vehicle in meters per second. You must check the status to know if the ground speed is still current.

- :code:`ground_course( instance )` - Returns the ground course of the vehicle in degrees. You must check the status to know if the ground course is still current.

- :code:`num_sats( instance )` - Returns the number of satellites that the GPS is currently tracking.

- :code:`time_week( instance )` - Returns the GPS week number.

- :code:`time_week_ms( instance )` - Returns the number of milliseconds into the current week.

- :code:`get_hdop( instance )` - Returns the horizontal dilution of precision of the GPS instance.

- :code:`get_vdop( instance )` - Returns the vertical dilution of precision of the GPS instance.

- :code:`last_fix_time_ms( instance )` - Returns the time of the last fix in system milliseconds.

- :code:`have_vertical_velocity( instance )` - Returns true if the GPS instance can report the vertical velocity.

- :code:`get_antenna_offset( instance )` - Returns a Vector3f that contains the offsets of the GPS in meters in the body frame.

- :code:`first_unconfigured_gps()` - Returns nil or the instance number of the first GPS that has not been fully configured. If all GPS's have been configured this returns 255 if all the GPS's have been configured.


GCS (gcs:)
~~~~~~~~~~

- :code:`send_text( severity ,  text )` - Will send the text  string with message severity level . Severity level is :

+---------------+-----------+
|Severity Level | Type      |
+---------------+-----------+
|0              | Emergency |
+---------------+-----------+
|1              | Alert     |
+---------------+-----------+
|2              | Critical  |
+---------------+-----------+
|3              | Error     |
+---------------+-----------+
|4              | Warning   |
+---------------+-----------+
|5              | Notice    |
+---------------+-----------+
|6              | Info      |
+---------------+-----------+


- :code:`set_message_interval( serial_channel ,  message_type ,  rate )` - Sets the message_type's update rate on SERIAL(serial_channel). For example, gcs:set_message_interval(0, 30, 500000) sets SERIAL0 rate for ATTITUDE message stream (30) to 2.0 Hz (500,000 microseconds)

Serial LED (serialLED:)
~~~~~~~~~~~~~~~~~~~~~~~

This library allows the control of WS8212B RGB LED strings via an output reserved for scripting and  selected by SERVOx_FUNCTION = 94 thru 109 (Script Out 1 thru 16)

- :code:`set_num_LEDs( output_number ,  number_of_LEDs )` - Sets the number_of_LEDs in the string on a servo output. output_number is servo output number 1-16 that the string is attached to with a string having <number_of_LEDs>.

- :code:`set_RGB( output_number ,  LED_number ,  r , g , b )` - Set the data for LED_number (1-32) on the string attached servo output_number (1-16) output to the r,g,b values (0-255)

- :code:`send()` - Sends the data to the LED strings


Notify (notify:)
~~~~~~~~~~~~~~~~

- :code:`play_tune( tune )` - Plays a MML tune through the buzzer on the vehicle. The tune is provided as a string.

An online `tune tester can be found here <https://firmware.ardupilot.org/Tools/ToneTester/>`__


Vehicle (vehicle:)
~~~~~~~~~~~~~~~~~~

- :code:`set_mode(mode_number)` - Attempts to change vehicle (in this example Plane) mode to mode_number. Returns true if successful, false if mode change is not successful.

- :code:`get_mode()` - Returns current vehicle mode by mode_number.

Mode numbers for each vehicle type can be `found here <https://mavlink.io/en/messages/ardupilotmega.html#PLANE_MODE>`__

- :code:`get_likely_flying()` - Returns true if the autopilot thinks it is flying. Not guaranteed to be accurate.

- :code:`get_flying_time_ms()` - Returns time in milliseconds since the autopilot thinks it started flying, or zero if not currently flying.

- :code:`start_takeoff(altitude)` - Begins auto takeoff to given altitude above home in meters. Returns false if not available.

- :code:`set_target_location(Location)` - Sets target location using a Location object. Returns false if not successful.

- :code:`get_target_location()` - Returns Location object of the current target location. Returns false if not available.

- :code:`set_target_velocity_NED()` - Sets the target velocity using a Vector3f object. Returns false if not available.


Terrain (terrain:)
~~~~~~~~~~~~~~~~~~


The terrain library provides access to checking heights against a terrain database.

- :code:`enabled()` - Returns true if terrain is enabled.

- :code:`status()` - Returns the current status of the rangefinder. Compare this to one of the terrain statuses (terrain.TerrainStatusDisabled, terrain.TerrainStatusUnhealthy, terrain.TerrainStatusOK).

- :code:`height_amsl( Location )` - Returns the height (in meters) above mean sea level at the provided Location userdata, or returns nil if that is not available.

- :code:`height_terrain_difference_home( Location )` - Returns the difference in height (in meters) between the provided location and home, or returns nil if that is not available.

- :code:`height_above_terrain()` - Returns the height (in meters) that the vehicle is currently above the terrain, or returns nil if that is not available.


Relay (relay:)
~~~~~~~~~~~~~~

The relay library proivdes access to controlling relay outputs.

- :code:`on(relay_num)` - Turns the requested relay on.

- :code:`off(relay_num)` - Turns the requested relay off.

- :code:`enabled(relay_num)` - Returns true if the requested relay is currently turned on.

- :code:`toggle(relay_num)` - Toggles the requested relay on or off.


Servo Channels (SRV_Channels:)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- :code:`find_channel(output_function)` - Returns first servo output number -1 of an output assigned output_function (See ``SERVOx_FUNCTION`` parameters ). False if none is assigned.

- :code:`get_output_pwm(output_function)` - Returns first servo output PWM value an output assigned output_function (See ``SERVOx_FUNCTION`` parameters ). False if none is assigned.

- :code:`set_output_pwm_chan_timeout(channel, pwm, timeout)` - Sets servo channel to specified PWM for a time in ms. This overrides any commands from the autopilot until the timeout expires.

RC Channels (rc:)
~~~~~~~~~~~~~~~~~

- :code:`get_pwm()` - Returns the RC input PWM value given a channel number. Note that channel here is indexed from 1. Returns false if channel is not available.


Serial/UART (serial:)
~~~~~~~~~~~~~~~~~~~~~

- :code:`find_serial(protocol)` - Returns the first UART instance that allows the given protocol, or nil if not found.

	- :code:`UART:begin(baud)` - Start serial connection at given baud rate.
	- :code:`UART:read()` - Returns a sequence of bytes from UART instance.
	- :code:`UART:write(number)` - Writes a sequence of bytes to UART instance.
	- :code:`UART:available()` -  Returns integer of currently available bytes.
	- :code:`UART:set_flow_control(flow_control)` - Sets flow control for UART instance.


Barometer (baro:)
~~~~~~~~~~~~~~~~~

- :code:`get_pressure()` - Returns pressure in Pascal.

- :code:`get_temperature()` - Returns temperature in degrees C.

- :code:`get_external_temperature()` - Returns external temperature in degrees C.


ESC Telemetry (esc_telem:)
~~~~~~~~~~~~~~~~~~~~~~~~~~

- :code:`get_usage_seconds(channel)` - Returns an individual ESC's usage time in seconds, or false if not available.


Parameters (param:)
~~~~~~~~~~~~~~~~~~~

- :code:`get(parameter_name)` - Returns parameter value if available, or nil if not found.

- :code:`set(parameter_name)` - Sets a parameter by name if available. Returns true if the parameter is found.

- :code:`set_and_save(parameter_name)` Sets and saves a parameter by name if available. Returns true if the parameter is found.


RPM (RPM:)
~~~~~~~~~~

- :code:`get_rpm(instance)` - Returns RPM of given instance, or nil if not available.


Button (button:)
~~~~~~~~~~~~~~~~

- :code:`get_button_state(button_number)` - Returns button state if available. Buttons are 1 indexed.

Servo Output
~~~~~~~~~~~~

This method stands alone and is called directly as shown below.

- :code:`servo.set_output (function_number, PWM)` -Sets servo outputs of type function_number to a PWM value (typically between 1000 and 2000)


.. lua:autoall::

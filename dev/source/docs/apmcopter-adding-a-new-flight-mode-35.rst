.. _apmcopter-adding-a-new-flight-mode-35:

====================================================
Adding a New Flight Mode to Copter 3.5 (and earlier)
====================================================

This section covers the basics of how to create a new high level flight
mode (i.e. equivalent of Stabilize, Loiter, etc) for Copter version 3.5 (or earlier).
For newer versions see the main :ref:`Adding a New Flight Mode <apmcopter-adding-a-new-flight-mode>` page. 

As a reference the diagram below provides a high level view of Copter's architecture for ver 3.5 (and earlier)

.. image:: ../images/copter-code-overview-architecture2.png
    :target: ../_images/copter-code-overview-architecture2.png
    :width: 450px

#. Create the #define for the new flight mode in
   `defines.h <https://github.com/ArduPilot/ardupilot/blob/Copter-3.5/ArduCopter/defines.h#L88>`__.
   and increase the NUM_MODES by 1. Or add your mode to the enum control_mode_t (depend on you copter version)

   ::

       // Auto Pilot modes
       // ----------------
       #define STABILIZE 0 // hold level position
       #define ACRO 1 // rate control
       #define ALT_HOLD 2 // AUTO control
       #define AUTO 3 // AUTO control
       #define GUIDED 4 // AUTO control
       #define LOITER 5 // Hold a single location
       #define RTL 6 // AUTO control
       #define CIRCLE 7 // AUTO control
       #define LAND 9 // AUTO control
       #define OF_LOITER 10 // Hold a single location using optical flow sensor
       #define DRIFT 11 // DRIFT mode (Note: 12 is no longer used)
       #define SPORT 13 // earth frame rate control
       #define FLIP 14 // flip the vehicle on the roll axis
       #define AUTOTUNE 15 // autotune the vehicle's roll and pitch gains
       #define POSHOLD 16 // position hold with manual override
       #define NEWFLIGHTMODE 17                // new flight mode description
       #define NUM_MODES 18

#. Create a new control_<new flight mode> sketch based on a similar
   flight mode such as
   `control_stabilize.cpp <https://github.com/ArduPilot/ardupilot/blob/Copter-3.5/ArduCopter/control_stabilize.cpp>`__
   or
   `control_loiter.cpp <https://github.com/ArduPilot/ardupilot/blob/Copter-3.5/ArduCopter/control_loiter.cpp>`__.
   This new file should have an ``_init()`` function and ``_run()``
   function.

   ::

       /// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

       /*
        * control_newflightmode.cpp - init and run calls for new flight mode
        */

       // newflightmode_init - initialise flight mode
       static bool newflightmode_init(bool ignore_checks)
       {
           // do any required initialisation of the flight mode here
           // this code will be called whenever the operator switches into this mode

           // return true initialisation is successful, false if it fails
           // if false is returned here the vehicle will remain in the previous flight mode
           return true;
       }

       // newflightmode_run - runs the main controller
       // will be called at 100hz or more
       static void newflightmode_run()
       {
           // if not armed or throttle at zero, set throttle to zero and exit immediately
           if(!motors.armed() || g.rc_3.control_in <= 0) {
               attitude_control.relax_bf_rate_controller();
               attitude_control.set_yaw_target_to_current_heading();
               attitude_control.set_throttle_out(0, false);
               return;
           }

           // convert pilot input into desired vehicle angles or rotation rates
           //   g.rc_1.control_in : pilots roll input in the range -4500 ~ 4500
           //   g.rc_2.control_in : pilot pitch input in the range -4500 ~ 4500
           //   g.rc_3.control_in : pilot's throttle input in the range 0 ~ 1000
           //   g.rc_4.control_in : pilot's yaw input in the range -4500 ~ 4500

           // call one of attitude controller's attitude control functions like
           //   attitude_control.angle_ef_roll_pitch_rate_yaw(roll angle, pitch angle, yaw rate);

           // call position controller's z-axis controller or simply pass through throttle
           //   attitude_control.set_throttle_out(desired throttle, true);
       }

#. Add declarations in
   `Copter.h <https://github.com/ArduPilot/ardupilot/blob/Copter-3.5/ArduCopter/Copter.h>`__
   for the new ``_init()`` function and ``_run()`` functions:

   ::

       bool newflightmode_init(bool ignore_checks);
       void newflightmode_run();

#. Add a case for the new mode to the ``set_mode()`` function in `flight_mode.cpp <https://github.com/ArduPilot/ardupilot/blob/Copter-3.5/ArduCopter/flight_mode.cpp#L12>`__
   to call the ``above _init()`` function.

   ::

       // set_mode - change flight mode and perform any necessary initialisation
       static bool set_mode(uint8_t mode)
       {
           // boolean to record if flight mode could be set
           bool success = false;
           bool ignore_checks = !motors.armed();   // allow switching to any mode if disarmed.  We rely on the arming check to perform

           // return immediately if we are already in the desired mode
           if (mode == control_mode) {
               return true;
           }

           switch(mode) {
               case ACRO:
                   #if FRAME_CONFIG == HELI_FRAME
                       success = heli_acro_init(ignore_checks);
                   #else
                       success = acro_init(ignore_checks);
                   #endif
                   break;

               case NEWFLIGHTMODE:
                   success = newflightmode_init(ignore_checks);
                   break;
           }
       }

#. Add a case for the new mode to the ``update_flight_mode()`` function in `flight_mode.cpp <https://github.com/ArduPilot/ardupilot/blob/Copter-3.5/ArduCopter/flight_mode.cpp#L172>`__
   to call the above ``_run()`` function.

   ::

       // update_flight_mode - calls the appropriate attitude controllers based on flight mode
       // called at 100hz or more
       static void update_flight_mode()
       {
           switch (control_mode) {
               case ACRO:
                   #if FRAME_CONFIG == HELI_FRAME
                       heli_acro_run();
                   #else
                       acro_run();
                   #endif
                   break;
               case NEWFLIGHTMODE:
                   success = newflightmode_run();
                   break;
           }
       }

#. Add the string to print out the flight mode to the
   ``print_flight_mode()`` function in `flight_mode.cpp <https://github.com/ArduPilot/ardupilot/blob/Copter-3.5/ArduCopter/flight_mode.cpp#L444>`__.

   ::

       static void
       print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
       {
           switch (mode) {
           case STABILIZE:
               port->print_P(PSTR("STABILIZE"));
               break;
           case NEWFLIGHTMODE:
               port->print_P(PSTR("NEWFLIGHTMODE"));
               break;

#. Add the new flight mode to the list of valid ``@Values`` for the
   ``FLTMODE1 ~ FLTMODE6`` parameters in `Parameters.cpp <https://github.com/ArduPilot/ardupilot/blob/Copter-3.5/ArduCopter/Parameters.cpp#L306>`__.

   ::

           // @Param: FLTMODE1
           // @DisplayName: Flight Mode 1
           // @Description: Flight mode when Channel 5 pwm is 1230, <= 1360
           // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,8:Position,9:Land,10:OF_Loiter,11:ToyA,12:ToyM,13:Sport,17:NewFlightMode
           // @User: Standard
           GSCALAR(flight_mode1, "FLTMODE1",               FLIGHT_MODE_1),

           // @Param: FLTMODE2
           // @DisplayName: Flight Mode 2
           // @Description: Flight mode when Channel 5 pwm is >1230, <= 1360
           // @Values: 0:Stabilize,1:Acro,2:AltHold,3:Auto,4:Guided,5:Loiter,6:RTL,7:Circle,8:Position,9:Land,10:OF_Loiter,11:ToyA,12:ToyM,13:Sport,17:NewFlightMode
           // @User: Standard
           GSCALAR(flight_mode2, "FLTMODE2",               FLIGHT_MODE_2),

#. Raise a request in the `Mission Planner's Issue List <https://github.com/ArduPilot/MissionPlanner/issues>`__ if you
   wish the new flight mode to appear in the Mission Planner's HUD and
   Flight Mode set-up.

   .. image:: ../images/FlightMode.jpg
       :target: ../_images/FlightMode.jpg
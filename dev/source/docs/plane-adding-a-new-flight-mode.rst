.. _plane-adding-a-new-flight-mode:

===============================
Plane: Adding a New Flight Mode
===============================

This page covers the basics of how to create a new fixed wing flight mode for Plane (i.e. the equivalent of FBWA, Cruise, etc). Like :ref:`Copter <apmcopter-adding-a-new-flight-mode>` and :ref:`Rover <rover-adding-a-new-drive-mode>`, each Plane flight mode is a class derived from ``Mode`` in `mode.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode.h>`__, and user selectable modes generally have their own ``mode_<name>.cpp`` file. A real example of adding a new flight mode can be found in `this commit <https://github.com/ArduPilot/ardupilot/commit/32f5afb22a9fc20bd3c2cc98fd62e3427b5f8724>`__ that first added the AUTOLAND mode.

Before starting, read the :ref:`Plane Architecture Overview <plane-architecture>` and :ref:`Plane Navigation and Altitude Control <plane-navigation-overview>` pages to understand how the navigation (L1/TECS) and attitude controllers fit together.

How a mode is run
=================

The currently active mode is held in ``plane.control_mode``. The vehicle code calls the following methods on it:

- ``update()`` is called every main loop from ``Plane::update_control_mode()``. It converts the pilot's input and/or the navigation controllers' output into roll and pitch targets (``plane.nav_roll_cd`` and ``plane.nav_pitch_cd``) and sets up throttle handling.
- ``run()`` is called every main loop from ``Plane::stabilize()``, except while a Lua script is controlling the vehicle through ``nav_scripting`` (i.e. scripted aerobatics), when the script's rate and throttle targets are used instead. The default implementation runs the roll, pitch and yaw attitude controllers to achieve ``nav_roll_cd`` and ``nav_pitch_cd``. Modes may override ``run()``; for example FBWA, where the pilot controls throttle directly, calls ``Mode::run()`` and then ``output_pilot_throttle()``.
- ``navigate()`` is called from the 10Hz ``Plane::navigate()`` task, provided the vehicle has a position estimate and a valid next waypoint, and should be overridden by modes that navigate towards a location (i.e. update the L1 controller).
- ``update_target_altitude()`` is called at 10Hz to update the altitude target for modes that use TECS altitude control.

Steps
=====

#. Pick a name for the new mode (i.e. "NEW_MODE") and add it to the ``Mode::Number`` enum in `mode.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode.h>`__ using an unused number. Mode number 30 is reserved for external/Lua control.

   ::

        enum Number : uint8_t {
            MANUAL        = 0,
            CIRCLE        = 1,
            STABILIZE     = 2,
            ...
        #if MODE_AUTOLAND_ENABLED
            AUTOLAND      = 26,
        #endif
            NEW_MODE      = 27,

        // Mode number 30 reserved for "offboard" for external/lua control.
        };

#. Define a new class for the mode in `mode.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode.h>`__. It is easiest to copy a similar existing mode's class definition and change the class name (i.e. copy ``class ModeFBWA`` and rename it ``class ModeNewMode``). The new class must implement the ``mode_number()``, ``name()``, ``name4()`` and ``update()`` methods. ``name4()`` must return exactly 4 characters; it is the short mode name used for notifications and displays such as the OSD (the logs record the mode number).

   ::

        class ModeNewMode : public Mode
        {
        public:

            Number mode_number() const override { return Number::NEW_MODE; }
            const char *name() const override { return "NEW_MODE"; }
            const char *name4() const override { return "NEWM"; }

            // methods that affect movement of the vehicle in this mode
            void update() override;
        };

   Optionally, override ``run()`` (see above) and the protected ``_enter()`` and ``_exit()`` methods. ``_enter()`` performs any initialisation required as the vehicle enters the mode (returning false will prevent the mode change) and ``_exit()`` performs any cleanup as the vehicle leaves the mode. Only declare the methods you need, as each one declared in the class must also be defined in the mode's ``.cpp`` file or the build will fail to link.

   ::

            void run() override;

        protected:

            bool _enter() override;
            void _exit() override;

   There are also many simple methods returning true/false in the ``Mode`` base class that you may want to override to control how the rest of the vehicle code treats the mode. Some of the most commonly used are:

   ::

        // true if the mode sets the vehicle destination, which controls
        // whether control input is ignored with STICK_MIXING=0
        virtual bool does_auto_navigation() const { return false; }

        // true if the mode controls throttle automatically (via TECS)
        virtual bool does_auto_throttle() const { return false; }

        // true if the mode supports autotuning via the AUTOTUNE RC switch
        virtual bool mode_allows_autotuning() const { return false; }

        // true for all VTOL (Q) modes
        virtual bool is_vtol_mode() const { return false; }

        // mode specific pre-arm checks
        virtual bool _pre_arm_checks(size_t buflen, char *buffer) const;

#. Create a new ``mode_newmode.cpp`` file based on a similar mode such as `mode_fbwa.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_fbwa.cpp>`__ (pilot controlled throttle) or `mode_cruise.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_cruise.cpp>`__ (navigation and TECS controlled throttle). The file should include ``mode.h`` and ``Plane.h`` and implement the ``update()`` method. The mode accesses the Plane object's variables and controllers through the ``plane`` reference.

   Below is an excerpt from ``ModeFBWA::update()`` that demonstrates how the pilot's input is converted into roll and pitch targets (in centi-degrees):

   ::

        void ModeFBWA::update()
        {
            // set nav_roll and nav_pitch using sticks
            plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
            plane.update_load_factor();
            float pitch_input = plane.channel_pitch->norm_input();
            if (pitch_input > 0) {
                plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
            } else {
                plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
            }
            ...
        }

        void ModeFBWA::run()
        {
            // Run base class function and then output throttle
            Mode::run();

            output_pilot_throttle();
        }

   Modes that navigate typically use ``plane.calc_nav_roll()``, ``plane.calc_nav_pitch()`` and ``plane.calc_throttle()`` in ``update()`` so that the L1 and TECS controllers set the roll, pitch and throttle targets, and override ``navigate()`` and ``does_auto_throttle()``. See `mode_cruise.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_cruise.cpp>`__ or `mode_loiter.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/mode_loiter.cpp>`__ for examples.

#. Instantiate the new mode class in `Plane.h <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/Plane.h>`__ by searching for "ModeFBWA mode_fbwa" and adding the new mode below it. Also add the new class to the list of "friend" classes near the top of ``Plane.h``, which allows the mode to access the Plane class's internal variables and functions.

   ::

        class Plane : public AP_Vehicle {
        public:
            ...
            friend class Mode;
            friend class ModeCircle;
            ...
            friend class ModeNewMode;

   ::

        ModeFBWA mode_fbwa;
        ModeFBWB mode_fbwb;
        ...
        ModeNewMode mode_newmode;

#. In `control_modes.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/control_modes.cpp>`__ add the new mode to the ``mode_from_mode_num()`` function to create the mapping between the mode's number and the instance of the class.

   ::

        Mode *Plane::mode_from_mode_num(const enum Mode::Number num)
        {
            Mode *ret = nullptr;
            switch (num) {
            ...
            case Mode::Number::NEW_MODE:
                ret = &mode_newmode;
                break;

#. Add the new mode to the other mode lists so that ground stations and the rest of the code handle it correctly:

   - the ``fw_modes`` (or ``q_modes`` for a VTOL mode) list in ``GCS_MAVLINK_Plane::send_available_mode()`` in `GCS_MAVLink_Plane.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/GCS_MAVLink_Plane.cpp>`__ so that ground stations which use the ``AVAILABLE_MODES`` message list it
   - the ``base_mode()`` switch in `GCS_MAVLink_Plane.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/GCS_MAVLink_Plane.cpp>`__ and the switch in `GCS_Plane.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/GCS_Plane.cpp>`__ that sets the rate/attitude/position control flags reported in ``SYS_STATUS``
   - optionally, the ``mode_list`` in ``Plane::gcs_mode_enabled()`` in `system.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/system.cpp>`__ so that the mode can be blocked from GCS selection with :ref:`FLTMODE_GCSBLOCK<plane:FLTMODE_GCSBLOCK>`. New modes must only be added to the end of this list (and the parameter's ``@Bitmask`` in ``AP_Vehicle.cpp`` updated) so that existing bits do not change. Also increase the size in the ``static_assert(ARRAY_SIZE(mode_list) == ...)`` check below the list and, if the mode can be compiled out, add a ``0xFF`` placeholder in its ``#else`` branch (as QAUTOTUNE and AUTOLAND have) so that later bits do not shift

   Also search the ``ArduPlane`` directory for an existing mode with similar behaviour (i.e. ``mode_fbwa`` or ``Mode::Number::FLY_BY_WIRE_A``) to find other places, such as failsafe handling in ``events.cpp``, where the new mode may need special treatment.

#. Add the new flight mode to the list of valid ``@Values`` for the ``FLTMODE1 ~ FLTMODE6`` parameters (and ``INITIAL_MODE``) in `Parameters.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduPlane/Parameters.cpp>`__ (search for "FLTMODE1"). Once committed to master, this will cause the new mode to appear in the list of valid values for these parameters in ground stations that use the parameter metadata. Note that even before being committed to master, a user can set up the new flight mode to be activated from the transmitter's flight mode switch by directly setting the FLTMODE1 (or FLTMODE2, etc) parameters to the number of the new mode.

   ::

        // @Param: FLTMODE1
        // @DisplayName: FlightMode1
        // @Description: Flight mode for switch position 1 (910 to 1230 and above 2049)
        // @Values: 0:Manual,1:CIRCLE,2:STABILIZE,...,26:AUTOLAND,27:NEW_MODE
        // @User: Standard

#. Add the new mode to the ``PLANE_MODE`` enum in `mavlink/ardupilotmega.xml <https://github.com/ArduPilot/mavlink/blob/master/message_definitions/v1.0/ardupilotmega.xml>`__ and submit a PR to `pymavlink <https://github.com/ArduPilot/pymavlink>`__ adding it to the ``mode_mapping_apm`` table in `mavutil.py <https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py>`__. MAVProxy does not use the ``AVAILABLE_MODES`` message and takes its mode names from this table, so the new mode will appear as unknown in MAVProxy until it is updated.

   `QGroundControl <https://github.com/mavlink/qgroundcontrol>`__ requests the list of modes from the vehicle with ``AVAILABLE_MODES``, so it shows the new mode by name without changes. Its hard-coded list of Plane modes in ``ArduPlaneFirmwarePlugin`` is only a fallback for older firmware, so adding the new mode there is optional. Mission Planner takes its list of modes from the ``FLTMODE1`` parameter metadata, so it needs no changes beyond the ``@Values`` update above.

#. Optionally, add an RC auxiliary switch option to enter the new mode (see ``RC_Channel_Plane.cpp``) and an autotest in ``Tools/autotest/arduplane.py`` that exercises the new mode in :ref:`SITL <sitl-simulator-software-in-the-loop>`.

.. note:: Many simple custom behaviours can be implemented without modifying the firmware by using a :ref:`Lua script <common-lua-scripts>`, which can take control of the vehicle in Guided mode or via the ``nav_scripting`` interface used for aerobatics.

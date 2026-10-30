.. _plane-adding-custom-controller:

==================================
Adding Custom Controllers to Plane
==================================

The ``AP_CustomControl`` library lets you implement and run your own controller inside Plane, in the same way that :ref:`AC_CustomControl does for Copter <copter-adding-custom-controller>`. "Main" refers to the standard ArduPilot controller and "custom" refers to the new controller.

This custom framework is best suited to support attitude and angular rate custom controllers (e.g. model-based control). But unlike the equivalent Copter library, which only replaces the roll, pitch and yaw mixer inputs, the Plane custom controller can drive any servo output, by output function or by output number, including outputs that are not otherwise configured. This makes it suitable for experimenting with the full range of control, from low-level control allocation and custom mixers up to navigation algorithms.

Features
========

- In-flight switching between main and custom controller with an RC switch, auxiliary function 109
- Custom controller can drive any servo output by function or by output number
- Bitmask to choose which custom controller outputs to enable
- Integrator reset when switching between controllers
- Ground and in-flight state separation to avoid build-up during arming and takeoff with the custom controller
- Frontend-backend separation that allows adding a new controller with very little overhead
- Single parameter to switch between different custom controllers, reboot required
- Checks to avoid accidentally running a misconfigured or unconfigured custom controller with the RC switch
- Custom controller parameters start with ``CP_`` in the GCS
- Build option to include the custom controller on hardware, ``--enable-PLANE_CUSTOM_CONTROL``

Parameters
==========

The frontend library has the following parameters:

- :ref:`CP_TYPE<plane:CP_TYPE>`: choose which custom controller backend to use, reboot required

  - Setting it to 0 turns this feature off, and the GCS will not display parameters related to the custom controller

- :ref:`CP_MASK<plane:CP_MASK>`: choose which features of the custom controller should run

  - This is a bitmask. Its meaning depends on each custom controller implementation. It is typically used to gate the custom controller outputs on each axis (roll, pitch, yaw), but also any other control output the controller provides

Backend parameters appear under ``CPn_``, where n is the :ref:`CP_TYPE<plane:CP_TYPE>` value, for example ``CP2_`` for the PID backend.

.. warning:: The custom controller requires the user to set up the build environment (:ref:`building-the-code`) and clone the ArduPilot GitHub repo locally (:ref:`where-to-get-the-code`). The example controllers are for experimental purposes only. Use caution when testing on a real vehicle.

Interaction With Main Controller
================================

The custom controller update is called after the stabilization task has run the main controller and before the servo output task. This placement allows overriding the inputs to the mixers, for example the values written to the aileron, elevator and rudder output functions, without any functional change inside the servo output code.

After running the custom controller, the updated outputs are read by the servo output task and sent to the servos and motors. By default, the safety checks and mixing in the servo output code still apply and may overwrite what the custom controller wrote. ``set_output_pwm_chan_override()`` (see `Utility API`_) can be used to stop the servo output code modifying a channel for that loop, which is useful for custom mixers.

The custom controller uses the same attitude targets as the main controller, which are passed to it each loop.

Bumpless Transfer
-----------------

When switching from the custom controller back to the main controller, the main controller integrators are reset, since they will have wound up while their output was being overridden. Only the axes the custom controller was actually overriding are reset, so the main controllers on other axes keep their state.

The custom controller is not only engaged and disengaged by the RC switch. It is also suspended and resumed without a switch event whenever its run conditions change, for example on an RC failsafe, a mode change, or a :ref:`CP_TYPE<plane:CP_TYPE>` change in flight. These are treated the same as a switch event: the backend ``reset()`` is called on every engage, so it never resumes with integrators and filters frozen at their pre-suspend values, and the main controllers are reset on every disengage. A suspension is reported to the GCS.

The main controller targets are not reset to the current states, so despite the integrator reset, some step change in the actuators should be expected when switching.

Backend Type
============

There are currently two custom controller backends:

- Empty backend, :ref:`CP_TYPE<plane:CP_TYPE>` = 1
- PID backend, :ref:`CP_TYPE<plane:CP_TYPE>` = 2

Empty Controller - CP_TYPE = 1
------------------------------

This is a template controller. It is not functional and cannot be engaged. It does no calculations and only forwards the aileron, elevator, throttle and rudder commands calculated by the main controller. It is intended as a starting point to copy for a new controller.

PID Controller - CP_TYPE = 2
----------------------------

The PID backend has roughly the same architecture as the main controller, and its default gains are the same as the main controller's, so you can change them and observe the difference.

It also demonstrates the library's capabilities by reading various RC input channels and writing to various servo outputs. Read ``AP_CustomControl_PID.cpp`` for its full behavior.

.. warning:: Make sure the autopilot is configured with the input channels the PID backend uses, and that the additional servo outputs it drives will not put the flight in danger. If necessary, remove those outputs from ``AP_CustomControl_PID.cpp::update()`` and recompile.

How To Use It
=============

The custom controller is enabled by default in SITL. You can test it using the PID backend.

**Step #1:** Compile and run the default SITL model. In the GCS, choose the custom controller type and set which RC switch activates the custom controller, then reboot. For example in MAVProxy, using RC channel 6 as the switch:

::

    param set CP_TYPE 2
    param set RC6_OPTION 109
    reboot

**Step #2:** Display the backend parameters, which are under ``CP2_`` for the PID backend:

::

    param show CP2*

**Step #3:** Arm and take off. While in flight, in any stabilized mode, switch RC6 high. In MAVProxy:

::

    rc 6 2000

Do not switch to the custom controller in MANUAL, STABILIZE or ACRO modes. The custom controller tracks the attitude targets, which are not generated in these modes, and the PID backend will refuse to engage in them.

**Step #4:** The GCS should show ``Custom controller is ON``.

**Step #5:** Set RC6 low to switch back to the main controller. The GCS should show ``Custom controller is OFF``.

Real Flight Testing
-------------------

It is recommended to always arm, take off, land and disarm with the main controller running, and to switch to the custom controller in level flight. Only arm and take off with the custom controller if proper ground handling has been implemented in it.

To test on hardware, build with the ``--enable-PLANE_CUSTOM_CONTROL`` option (scripting must also be enabled, as it is on most boards):

::

    ./waf configure --board CubeOrange --enable-PLANE_CUSTOM_CONTROL
    ./waf plane

It can also be enabled in a custom hwdef by adding:

::

    define AP_PLANE_CUSTOMCONTROL_ENABLED 1

Post Flight Logs
----------------

Switching in and out of the custom controller is logged in the ``CP`` log message. ``CP.Act`` is what the pilot requested with the RC switch, and ``CP.Run`` is whether the controller was actually running. These differ whenever the controller is suspended without a switch event, so use ``CP.Run`` for when the custom controller was really in control.

How To Add a New Custom Controller
==================================

#. In ``libraries/AP_CustomControl``, create new ``AP_CustomControl_*.h`` and ``AP_CustomControl_*.cpp`` files for your controller. The existing controllers can be copied as a starting point.
#. In the ``.cpp`` file, declare your parameters in ``var_info[]``, your control logic in ``update()`` and your reset logic in ``reset()``. You are also encouraged to add engage conditions to ``can_run()``, so that the controller cannot be activated under the wrong conditions.
#. Register the controller with the library:

   - Add a new entry to ``enum class CustomControlType`` in ``AP_CustomControl.h`` and increment ``AP_PLANE_CUSTOMCONTROL_MAX_TYPES``
   - In ``AP_CustomControl.cpp``, include your new header, register your controller's parameters in ``var_info[]`` and create the controller in ``init()``

#. Add your new defines to ``AP_CustomControl_config.h``.

If your controller is provided externally or is auto-generated (for example from Simulink, see :ref:`copter-adding-custom-controller`), it is a good idea to place it as-is in the ``AP_CustomControl`` folder and include it from your new custom controller files.

Utility API
-----------

Although code in ``AP_CustomControl`` can potentially control any part of Plane, the following API is provided for safer interaction with the rest of the system.

Attitude targets:

.. code-block:: c++

    float get_roll_target_deg();
    float get_nav_pitch_target_deg();
    float get_pitch_target_deg();   // includes pitch trim

Servo outputs, through the ``_frontend`` object:

.. code-block:: c++

    // Write a scaled value to all channels with a function.
    void set_output_scaled(SRV_Channel::Function function, float value);
    // Write a pwm value to all channels with a function. Not min/max constrained. servos.cpp may overwrite it.
    void set_output_pwm(SRV_Channel::Function function, uint16_t value);
    // Write pwm values on a channel. Not min/max constrained. servos.cpp may overwrite it.
    void set_output_pwm_chan(uint8_t chan, uint16_t value);
    // Override pwm values on a channel for one loop. servos.cpp will not overwrite it.
    void set_output_pwm_chan_override(uint8_t chan, uint16_t value);

RC inputs are read using the ``rc()`` singleton. Check the returned channel against ``nullptr`` before using it.

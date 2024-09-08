.. _connect-escs-and-motors:

=======================
Connect ESCs and Motors
=======================

This article explains how to connect the ESCs, motors and propellers to an autopilot.  The Pixhawk is used as an example but other autopilots are connected in a similar way.

Connect the power (+), ground (-), and signal (s) wires for each ESC to
the autopilot's main output pins by motor number. Find your frame type
below to determine the assigned order of the motors.

.. figure:: ../images/Pixhwak_outputs.jpg
   :target: ../_images/Pixhwak_outputs.jpg

   Pixhawk output pins (numbered). First 4 pins are colour-coded for connecting a Quadframe

Motor order diagrams
====================

The diagrams below show motor order for each frame type.
Propeller direction is shown in green (clockwise, CW) or blue (counter-clockwise, CCW).
The letters shown in red indicate which motor should turn when using Mission Planner's **Motor Test** feature found under its SETUP->Optional Hardware tab.

.. figure:: ../images/MOTORS_CW_CCWLegend.jpg
   :target: ../_images/MOTORS_CW_CCWLegend.jpg

   Legend for motor-order diagrams

QUAD FRAMES
-----------

.. image:: ../images/m_01_00_quad_plus.svg
    :target: ../_images/m_01_00_quad_plus.svg
    :scale: 39%
    :alt: QUAD PLUS

.. image:: ../images/m_01_01_quad_x.svg
    :target: ../_images/m_01_01_quad_x.svg
    :scale: 40%
    :alt: QUAD X

.. image:: ../images/m_01_02_quad_v.svg
    :target: ../_images/m_01_02_quad_v.svg
    :scale: 40%
    :alt: QUAD V

.. image:: ../images/m_01_03_quad_h.svg
    :target: ../_images/m_01_03_quad_h.svg
    :scale: 40%
    :alt: QUAD H

.. image:: ../images/m_01_04_quad_v_tail.svg
    :target: ../_images/m_01_04_quad_v_tail.svg
    :scale: 40%
    :alt: QUAD V TAIL

.. image:: ../images/m_01_05_quad_a_tail.svg
    :target: ../_images/m_01_05_quad_a_tail.svg
    :scale: 40%
    :alt: QUAD A TAIL

.. note::
    Quad A Tail and V Tail frames do not use the front motors for yaw control (NYT). The direction of motor rotation is not critical for basic operation, but unless the front motors spin in opposite directions, roll and pitch inputs will result in adverse yaw, resulting in less dynamic range of the yaw control.

.. image:: ../images/m_01_06_quad_plus_rev.svg
    :target: ../_images/m_01_06_quad_plus_rev.svg
    :scale: 39%
    :alt: QUAD PLUS (REVERSED)

.. image:: ../images/m_01_12_quad_x_bf.svg
    :target: ../_images/m_01_12_quad_x_bf.svg
    :scale: 40%
    :alt: QUAD X (BETAFLIGHT)

.. image:: ../images/m_01_13_quad_x_dji.svg
    :target: ../_images/m_01_13_quad_x_dji.svg
    :scale: 40%
    :alt: QUAD X (DJI)

.. image:: ../images/m_01_14_quad_x_cw.svg
    :target: ../_images/m_01_14_quad_x_cw.svg
    :scale: 40%
    :alt: QUAD X (CLOCKWISE)

.. image:: ../images/m_01_16_quad_plus_nyt.svg
    :target: ../_images/m_01_16_quad_plus_nyt.svg
    :scale: 39%
    :alt: QUAD PLUS (NO YAW TORQUE)

.. image:: ../images/m_01_17_quad_x_nyt.svg
    :target: ../_images/m_01_17_quad_x_nyt.svg
    :scale: 40%
    :alt: QUAD X (NO YAW TORQUE)

.. note::
    Quad 'No Yaw Torque' (NYT) frames are primarily intended for :ref:`VTOL Tailsitter<guide-tailsitter>` configurations with large control surfaces. Motor rotation direction does not matter for these motors, but unless the motor's spin directions are setup as any kind of normal QUAD frame, roll and pitch inputs will result in adverse yaw, resulting in less dynamic range of the yaw control of the fixed wing control surfaces.

.. image:: ../images/m_01_18_quad_x_bf_rev.svg
    :target: ../_images/m_01_18_quad_x_bf_rev.svg
    :scale: 40%
    :alt: QUAD X (BF REVERSED)

.. image:: ../images/m_01_19_quad_y4a.svg
    :target: ../_images/m_01_19_quad_y4a.svg
    :scale: 45%
    :alt: QUAD Y4A

HEXA FRAMES
-----------

.. image:: ../images/m_02_00_hexa_plus.svg
    :target: ../_images/m_02_00_hexa_plus.svg
    :scale: 36%
    :alt: HEXA PLUS

.. image:: ../images/m_02_01_hexa_x.svg
    :target: ../_images/m_02_01_hexa_x.svg
    :scale: 35%
    :alt: HEXA X

.. image:: ../images/m_02_03_hexa_h.svg
    :target: ../_images/m_02_03_hexa_h.svg
    :scale: 35%
    :alt: HEXA H

.. image:: ../images/m_02_13_hexa_x_dji.svg
    :target: ../_images/m_02_13_hexa_x_dji.svg
    :scale: 35%
    :alt: HEXA X (DJI)

.. image:: ../images/m_02_14_hexa_x_cw.svg
    :target: ../_images/m_02_14_hexa_x_cw.svg
    :scale: 35%
    :alt: HEXA X (CLOCKWISE)

OCTO FRAMES
-----------

.. image:: ../images/m_03_00_octo_plus.svg
    :target: ../_images/m_03_00_octo_plus.svg
    :scale: 30%
    :alt: OCTO PLUS

.. image:: ../images/m_03_01_octo_x.svg
    :target: ../_images/m_03_01_octo_x.svg
    :scale: 31%
    :alt: OCTO X

.. image:: ../images/m_03_02_octo_v.svg
    :target: ../_images/m_03_02_octo_v.svg
    :scale: 33%
    :alt: OCTO V

.. image:: ../images/m_03_03_octo_h.svg
    :target: ../_images/m_03_03_octo_h.svg
    :scale: 30%
    :alt: OCTO H

.. image:: ../images/m_03_13_octo_x_dji.svg
    :target: ../_images/m_03_13_octo_x_dji.svg
    :scale: 31%
    :alt: OCTO X (DJI)

.. image:: ../images/m_03_14_octo_x_cw.svg
    :target: ../_images/m_03_14_octo_x_cw.svg
    :scale: 31%
    :alt: OCTO X (CLOCKWISE)

.. image:: ../images/m_03_15_octo_i.svg
    :target: ../_images/m_03_15_octo_i.svg
    :scale: 33%
    :alt: OCTO I

OCTO QUAD FRAMES
----------------

.. image:: ../images/m_04_00_octo_quad_plus.svg
    :target: ../_images/m_04_00_octo_quad_plus.svg
    :scale: 38%
    :alt: OCTO QUAD PLUS

.. image:: ../images/m_04_01_octo_quad_x.svg
    :target: ../_images/m_04_01_octo_quad_x.svg
    :scale: 40%
    :alt: OCTO QUAD X

.. image:: ../images/m_04_02_octo_quad_v.svg
    :target: ../_images/m_04_02_octo_quad_v.svg
    :scale: 40%
    :alt: OCTO QUAD V

.. image:: ../images/m_04_03_octo_quad_h.svg
    :target: ../_images/m_04_03_octo_quad_h.svg
    :scale: 40%
    :alt: OCTO QUAD H

.. image:: ../images/m_04_12_octo_quad_x_bf.svg
    :target: ../_images/m_04_12_octo_quad_x_bf.svg
    :scale: 40%
    :alt: OCTO QUAD X (BETAFLIGHT)

.. image:: ../images/m_04_14_octo_quad_x_cw.svg
    :target: ../_images/m_04_14_octo_quad_x_cw.svg
    :scale: 40%
    :alt: OCTO QUAD X (CLOCKWISE)

.. image:: ../images/m_04_18_octo_quad_x_bf_rev.svg
    :target: ../_images/m_04_18_octo_quad_x_bf_rev.svg
    :scale: 40%
    :alt: OCTO QUAD X (BF REVERSED)

Y6 FRAMES
---------

.. image:: ../images/m_05_00_y6_a.svg
    :target: ../_images/m_05_00_y6_a.svg
    :scale: 36%
    :alt: Y6 A

.. image:: ../images/m_05_10_y6_b.svg
    :target: ../_images/m_05_10_y6_b.svg
    :scale: 36%
    :alt: Y6 B

.. image:: ../images/m_05_11_y6_f.svg
    :target: ../_images/m_05_11_y6_f.svg
    :scale: 36%
    :alt: Y6 F

TRICOPTER FRAMES
----------------

.. image:: ../images/m_07_00_tricopter.svg
    :target: ../_images/m_07_00_tricopter.svg
    :scale: 36%
    :alt: TRICOPTER 

.. image:: ../images/m_07_06_tricopter_pitch_rev.svg
    :target: ../_images/m_07_06_tricopter_pitch_rev.svg
    :scale: 36%
    :alt: TRICOPTER PITCH REVERSED

.. note::
    Since the tail (or nose) servo is used for yaw control, Tricopter motor rotation direction is not critical for basic operation, but unless the front motors spin in opposite directions, roll and pitch inputs will result in adverse yaw, resulting in less dynamic range of the yaw control. If the direction of your tail (or nose) servo is going the wrong way in response to yaw then either the ``RCn_REVERSE`` RC input direction or the tilt servo's ``SERVOn_REVERSE`` parameter should be set to 1 (from 0), See :ref:`TriCopter setup page <tricopter>` for details.

BICOPTER FRAMES
---------------

.. image:: ../images/m_10_00_bicopter.svg
    :target: ../_images/m_10_00_bicopter.svg
    :scale: 40%
    :alt: BICOPTER 

.. note::
    If desired, Bicopter motors may rotate opposite the directions shown (e.g., CW left, CCW right).

DODECAHEXA FRAMES
-----------------

.. image:: ../images/m_12_00_dodecahexa_plus.svg
    :target: ../_images/m_12_00_dodecahexa_plus.svg
    :scale: 32%
    :alt: DODECAHEXA PLUS

.. image:: ../images/m_12_01_dodecahexa_x.svg
    :target: ../_images/m_12_01_dodecahexa_x.svg
    :scale: 32%
    :alt: DODECAHEXA X

DECA FRAMES
-----------

.. image:: ../images/m_14_00_deca_plus.svg
    :target: ../_images/m_14_00_deca_plus.svg
    :scale: 28%
    :alt: DECA PLUS

.. image:: ../images/m_14_01_deca_x_and_cw_x.svg
    :target: ../_images/m_14_01_deca_x_and_cw_x.svg
    :scale: 28%
    :alt: DECA X (and CW X)

.. _connect-escs-and-motors_attach_propellers:

Custom Frames
-------------

It is possible to configure custom frame types using up-to 12 motors using :ref:`lua scripting<common-lua-scripts>`. The roll, pitch and yaw factors for each motor must be calculated and loaded from a script.
This is enabled by setting :ref:`FRAME_CLASS <FRAME_CLASS>` to 15 - Scripting Matrix. See: `plus quad example <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/MotorMatrix_setup.lua>`__ 
and `fault tolerant hex example <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/examples/MotorMatrix_fault_tolerant_hex.lua>`__.

.. note::
    Not all autopilots support scripting see: :ref:`firmware limitations <common-limited_firmware>`.

Recognizing clockwise and counterclockwise propellers
-----------------------------------------------------

The diagrams above show two types of propellers: clockwise (called pushers) and counterclockwise (called pullers). The most reliable to recognize the correct propeller type by its shape as shown below. The thicker edge is the leading edge which moves in the direction of rotation. The trailing edge is more radical scalloped and usually thinner. |prop_direction|

.. _connect-escs-and-motors_testing_motor_spin_directions:

Testing motor spin directions
=============================

If you have completed the :ref:`Radio <common-radio-control-calibration>`
and :ref:`ESC calibration <esc-calibration>`, you can check that your
motors are spinning in the correction direction:

#. Make sure there are no propellers on your copter!
#. Turn transmitter on and ensure the flight mode switch is set to
   Stabilize.
#. Connect battery.
#. Arm copter by holding the throttle down and rudder right for five
   seconds.
#. If it fails to Arm with the throttle down and to the right and the
   motors will not spin, it has probably failed the  Pre-Arm Safety
   Check.

   -  Pre-Arm safety check failure is also indicated by the red arming
      light double flashing and then repeating.
   -  If the Pre-Arm check fails go to the :ref:`Prearm Safety Check Page <common-prearm-safety-checks>` and correct the problem or disable
      the check before continuing.

#. When you can Arm successfully, apply a small amount of throttle, and
   observe and note spin direction of each motor. They should match
   directions shown in the images above for the frame you've chosen.
#. Reverse any motor spinning in the wrong direction.

   .. tip::

      Motor Direction is reversed simply by interchanging two of the
         three ESC to motor power leads.

Checking the motor numbering with the Mission Planner Motor test
================================================================

An alternative way to check that the motors have been hooked up
correctly is to use the "Motors" test in the Mission Planner Initial
Setup menu.

.. figure:: ../images/MissionPlanner_MotorTest.png
   :target: ../_images/MissionPlanner_MotorTest.png

   Mission Planner: Motor Test

When connected to the vehicle via MAVLink, you can click on the green
buttons shown above and the corresponding motor should spin for five
seconds. Letters correspond to motor numbers as shown in the example
below.

-  Take off your props first!
-  If no motors turn, raise the "Throttle %" to 10% and try again. If
   that doesn't work, try 15%

The first motor to spin will be the one located directly forward in the
case of + configuration, or the first motor to the right of straight
forward in the case of X configuration. The motor test will then proceed
in a clockwise rotation.

.. image:: ../images/APM_2_5_MOTORS_QUAD_enc.jpg
    :target: ../_images/APM_2_5_MOTORS_QUAD_enc.jpg

In the case of X8, it will spin the top front-right motor first, then
the bottom front-right, and proceed around with the same pattern.

OctoV will spin the front-right motor first, and then again, proceed
clock-wise until reaching the front left motor.

Using a Power Distribution Board
================================

.. image:: ../images/3dr_power_distribution_board.jpg
    :target: ../_images/3dr_power_distribution_board.jpg

There are two methods of connecting the motor outputs.  Either connect the
electronic speed controllers (ESCs) directly to the autopilot OR use a power distribution board (PDB).

When using a PDB, connect the power (+), ground (-), and signal (s)
wires for each ESC to the PDB according to motor number. Find your frame
type below to determine the assigned order of the motors. Then connect
the signal wires from the PDB to the main output signal pins on the
autopilot  (ensuring that the motor order numbers match the
main output pin numbers on the controller). If you are using a power
module, it is optional to connect the power and ground wires from the
PDB to the autopilot board. If you would like to use these
cables in addition to or instead of the power module or as a common
point for low current servos, connect the ground (-) wire to a main
output ground (-) pin and the power (+) wire to a main output power (+) pin.

KDE (and other) Opto Isolated ESCs
==================================

The KDEXF-UAS and KDEF-UASHV Series are opto-isolated and do not provide
BEC power output for the peripheral equipment. They require +5V to power
the opto-isolator and while the Pixhawk can be powered from the servo
rail, it does not provide +5V to the servo rail. The ESCs must be
powered by a BEC or with a jumper from an unused connector on the board.
It is strongly recommended that you use a BEC to power the rail rather
than a jumper.

.. image:: ../images/Pixhawk-Correction-to-KDE-ESC2.png
    :target: ../_images/Pixhawk-Correction-to-KDE-ESC2.png

The KDE ESCs have fixed PWM ranges so you must manually set the output
range of each PWM signal so that RCx_MIN is 1100 and RCx_MAX is 1900us
using the Advanced Parameter or Full Parameter Settings Page in the
planner.

Pixhawk ESC issues
==================

Some ESCs have been reported as not working with Pixhawk.

The Pixhawk should work with every ESC that works with a normal RC
receiver (because it sends the same type of signal) but there is `one known exception, the EMAX ESC <https://github.com/ArduPilot/ardupilot/issues/2094>`__.

In most cases problems are due to incorrect wiring. Always connect signal and ground. 
Check your ESC type to decide how to connect the +5V line. 
For Pixhawk you must connect both the signal and the signal ground in order to make the ESC work.

For more information see `this video <https://youtu.be/6C1YG1e2aTo>`__.

.. |prop_direction| image:: ../images/prop-direction.png
    :target: ../_images/prop-direction.png
    :width: 450px

.. _servo-functions:

========================
Choosing Servo Functions
========================

.. note:: this page only applies for plane firmware version 3.8.0 and later. 

The most fundamental setup for any plane is the servo output
functions. Flight boards vary in the number of outputs they support,
some with as few as 6 outputs and others with up to 16. Each of
these outputs can be configured as needed for your airframe.

The default outputs are very simple:

- servo output 1 is aileron
- servo output 2 is elevator
- servo output 3 is throttle
- servo output 4 is rudder

This is fine for many simple aircraft, but will need to be changed if
you have elevons, a vtail, flaps or any other more complex setup.

Overview
========

Each of the servo output functions is settable with the
corresponding SERVOn_FUNCTION parameter. The defaults are:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th><th>Meaning</th></tr>
   <tr><td>SERVO1_FUNCTION</td><td>4</td><td>aileron</td></tr>
   <tr><td>SERVO2_FUNCTION</td><td>19</td><td>elevator</td></tr>
   <tr><td>SERVO3_FUNCTION</td><td>70</td><td>throttle</td></tr>
   <tr><td>SERVO4_FUNCTION</td><td>21</td><td>rudder</td></tr>
   </table>
   
Note that this is very different from firmware versions prior to 3.8,
where the function of the first 4 outputs was fixed, and outputs above
4 could be set using the RCn_FUNCTION parameters.

Available Output Functions
==========================

Each of your servo outputs can be configured to a wide range of
possible functions. The key options for traditional fixed wing
aircraft are:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Function</th><th>Value</th></tr>
   <tr><td>FLAP</td><td>2</td</tr>
   <tr><td>AILERON</td><td>4</td></tr>
   <tr><td>ELEVATOR</td><td>19</td></tr>
   <tr><td>RUDDER</td><td>21</td></tr>
   <tr><td>STEERING</td><td>26</td></tr>
   <tr><td>THROTTLE</td><td>70</td></tr>
   <tr><td>THROTTLE_LEFT</td><td>73</td></tr>
   <tr><td>THROTTLE_RIGHT</td><td>74</td></tr>
   <tr><td>ELEVON_LEFT</td><td>77</td></tr>
   <tr><td>ELEVON_RIGHT</td><td>78</td></tr>
   <tr><td>VTAIL_LEFT</td><td>79</td></tr>
   <tr><td>VTAIL_RIGHT</td><td>80</td></tr>
   </table>

in addition, there are servo functions for vtol lifting motors on
quadplanes, control of internal combustion engines, pass-thru of
controls from RC inputs, camera controls, control of motor tilt
functions for tilt-rotors and many others. Those functions are
discussed in other parts of the documentation.

The guide below will concentrate in the primary flight control
surfaces of fixed wing aircraft.

Controlling Servo Throw, Trim and Reversal
==========================================

In addition to the function, each servo output also has parameters to
control the servo throw, the trim and servo reversal.

The parameters are:

- SERVOn_MIN is the minimum PWM output value
- SERVOn_MAX is the maximum PWM output value
- SERVOn_TRIM is the trim value for output types that have trim
- SERVOn_REVERSED controls servo reversal

For example, if you have:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th></tr>
   <tr><td>SERVO5_FUNCTION</td><td>77</td></tr>
   <tr><td>SERVO5_MIN</td><td>1000</td></tr>
   <tr><td>SERVO5_MAX</td><td>2000</td></tr>
   <tr><td>SERVO5_TRIM</td><td>1504</td></tr>
   <tr><td>SERVO5_REVERSED</td><td>1</td></tr>
   </table>

then that would setup servo output 5 as a reversed left elevon, with a
throw from 1000 to 2000, and a trim of 1504.

.. note:: The reversed parameter is different from the REV parameters
          in firmware versions 3.7 and earlier. In 3.7 the RC2_REV
          value was 1 for not reversed and -1 for reversed, which
          means it was a multipler. In 3.8 the REVERSED parameter is a
          boolean, where 1 means reversed, and 0 means not reversed.

Multiple Outputs for One Type
=============================

The SERVOn_FUNCTION values can be assigned to any output, and you can
have multiple output channels with the same output function. For
example, you can have multiple output channels marked as aileron
(function 4) and set the throws and trim on each aileron
separately. This allows you to trim your left and right ailerons
differently to account for differences in servo trim.
          
Setup of Specific Aircraft Types
================================

Now that you understand the basics of the servo output functions you
should read the guide for your particular type of aircraft.

Before you do that however, you should make sure you have correctly
setup your :ref:`RC inputs <rc-throw-trim>`.

- :ref:`4 Channel Plane Setup Guide <guide-four-channel-plane>`
- :ref:`Elevon Plane Setup Guide <guide-elevon-plane>`
- :ref:`VTail Plane Setup Guide <guide-vtail-plane>`
- :ref:`QuadPlane Setup Guide <quadplane-support>`
- :ref:`TiltRotor Setup Guide <guide-tilt-rotor>`
- :ref:`Tailsitter Setup Guide <guide-tailsitter>`

Note that you can combine setups for more complex aircraft. So for
example, you could setup a vtail tiltrotor quadplane by combining
functions from the guides above.

Final Setup
===========

After completing the above you should move onto the final setup of
your aircraft.

- :ref:`ESC Calibration <guide-esc-calibration>`
- :ref:`Center of Gravity <guide-center-of-gravity>`

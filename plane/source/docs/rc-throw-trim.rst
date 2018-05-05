.. _rc-throw-trim:

Controlling Input Throw, Trim and Reversal
==========================================

Where the parameters starting with \SERVOn_ control the servo outputs,
the parameters starting with \RCn_ control the input side, related to
pilot stick movement.

For the 3.8 firmware and above the input side and the output side are
completely separated, allowing you to have completely different PWM
ranges, reversal and trim on your transmitter input as compared to the
servo output.

In addition to the \RCn_ parameters for RC input range, trim and
reversal, you also can choose the function of the first 4 input
channels using the \RCMAP_ parameters. The defaults are:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th></tr>
   <tr><td>RCMAP_ROLL</td><td>1</td></tr>
   <tr><td>RCMAP_PITCH</td><td>2</td></tr>
   <tr><td>RCMAP_THROTTLE</td><td>3</td></tr>
   <tr><td>RCMAP_YAW</td><td>4</td></tr>
   </table>
   
that means your first input channel will be roll input, the 2nd input
channel will be pitch input, the 3rd input channel will be throttle
input and the 4th input channel will be yaw input (which maps to
rudder control in most flight modes).

The input range, trim and reversal are controlled by the RCn_MIN,
RCn_MAX, RCn_TRIM and RCn_REVERSED parameters.

For example, if you have:

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th></tr>
   <tr><td>RCMAP_ROLL</td><td>1</td></tr>
   <tr><td>RC1_MIN</td><td>1100</td></tr>
   <tr><td>RC1_MAX</td><td>1900</td></tr>
   <tr><td>RC1_TRIM</td><td>1500</td></tr>
   <tr><td>RC1_REVERSED</td><td>0</td></tr>
   </table>
   
then channel 1 input will be roll (mapping to aileron), with a minimum
PWM of 1100, a maximum of 1900 and a trim of 1500. The RC1_REVERSED
value of zero means it is not reversed.

Meaning of Reversed
===================

For RC inputs, the following table shows the standard meanings of the
REVERSED parameter for each input type.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Input</th><th>REVERSED=0</th><th>REVERSED=1</th></tr>
   <tr>
     <td>Roll</td>
     <td>Larger PWM values mean roll to the right</td>
     <td>Larger PWM values mean roll to the left</td>
   </tr>
   <tr>
     <td>Pitch</td>
     <td>Larger PWM values mean pitch up</td>
     <td>Larger PWM values mean pitch down</td>
   </tr>
   <tr>
     <td>Throttle</td>
     <td>Larger PWM values mean more throttle</td>
     <td>Larger PWM values mean less throttle</td>
   </tr>
   <tr>
     <td>Yaw</td>
     <td>Larger PWM values mean yaw right</td>
     <td>Larger PWM values mean yaw left</td>
   </tr>
   </table>

Throttle Reversal
=================

It is very unusual to have a reversed throttle input. In most cases it
means your transmitter is setup incorrectly. Having a reversed
throttle output is much more common for internal combustion motors. In
firmware version 3.8 and later you do not need to reverse throttle
input just because your output is reversed (ie. you can set
RC3_REVERSED separately from SERVO3_REVERSED).

If you do need to use a reversed throttle input then you need to be be
very careful with your throttle failsafe settings. The plane parameter
THR_FS_VALUE is normally the throttle value below which the autopilot
will consider your transmitter to be in RC failsafe. If you have a
reversed throttle input then that value will need to be a high value,
and will be the value above which you are in RC failsafe.

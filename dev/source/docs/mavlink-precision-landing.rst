.. _mavlink-precision-landing:

=================
Precision Landing
=================

Copter, QuadPlane and Rover support precision landing using the `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`__ mavlink message

References:

- :ref:`Copter's user wiki for precision landing <copter:precision-landing-with-irlock>`
- `QuadPlane discussion of precision landing <https://discuss.ardupilot.org/t/quadplane-precision-landing-support-testers-needed/114072>`__
- `MAVLink's Landing Target documentation <https://mavlink.io/en/services/landing_target.html>`__
- :ref:`Testing Precision Landing in SITL <testing_precision_landing>`

Users should follow the :ref:`user precision landing wiki page <copter:precision-landing-with-irlock>` including setting :ref:`PLND_TYPE <PLND_TYPE>` = 1 (MAVLink)

The external camera system should send the `LANDING_TARGET <https://mavlink.io/en/messages/common.html#LANDING_TARGET>`__ message to the autopilot at no less than 1hz (a higher rate is better).

If the x (e.g. forward), y (e.g. right) and z (e.g. down) distance to the target is known (in body frame) then "x", "y" and "z" fields should be populated, and "position_valid" should be set to "1".

If only the body-frame angle to the target is known then "angle_x" and "angle_y" fields should be populated and "position_valid" should be set to "0".

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Field Name</th>
   <th>Type</th>
   <th>Description</th>
   </tr>
   <tr>
   <td><strong>time_usec</strong></td>
   <td>uint64_t</td>
   <td>Timestamp since system boot.  This does not need to be syncronised with the autopilot's time</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>target_num</td>
   <td>uint8_t</td>
   <td>not used</td>
   </tr>
   <tr>
   <td><strong>frame</strong></td>
   <td>uint8_t</td>
   <td>MAV_FRAME_BODY_FRD (12)</td>
   </tr>
   <tr>
   <td><strong>angle_x</strong></td>
   <td>float</td>
   <td>X-axis angular offset (in radians) of the target from the center of the image</td>
   </tr>
   <tr>
   <td><strong>angle_y</strong></td>
   <td>float</td>
   <td>Y-axis angular offset (in radians) of the target from the center of the image</td>
   </tr>
   <tr>
   <td><strong>distance</strong></td>
   <td>float</td>
   <td>Distance (in m) to the target from the vehicle or 0 if unknown</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>size_x</td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>size_y</td>
   <td>float</td>
   <td>not used</td>
   </tr>
   <tr>
   <td><strong>x</strong></td>
   <td>float</td>
   <td>x position of the landing target in meters (e.g. forward in vehicle body-frame)</td>
   </tr>
   <tr>
   <td><strong>y</strong></td>
   <td>float</td>
   <td>y position of the landing target in meters (e.g. right in vehicle body-frame)</td>
   </tr>
   <tr>
   <td><strong>z</strong></td>
   <td>float</td>
   <td>z position of the landing target in meters (e.g down in vehicle body-frame)</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>q</td>
   <td>float[4]</td>
   <td>not used</td>
   </tr>
   <tr style="color: #c0c0c0">
   <td>type</td>
   <td>uint8_t</td>
   <td>not used</td>
   </tr>
   <td><strong>position_valid</strong></td>
   <td>uint8_t</td>
   <td>0 if angle_x, angle_y should be used.  1 if x, y, z fields contain position information</td>
   </tr>
   </tbody>
   </table>

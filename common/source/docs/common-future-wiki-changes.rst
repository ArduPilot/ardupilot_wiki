.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================

COMMON WIKI PAGES
=================

RCx_OPTIONs on Auxillary Functions Page:
----------------------------------------


+----------------------+----------------------------+----------+---------+---------+
| **RCx_OPTION value** | **Feature Description**    |**Copter**|**Plane**|**Rover**|
+----------------------+----------------------------+----------+---------+---------+
|        78            | RunCam Control             |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        79            | RunCam OSD Control         |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        80            | Viso Align                 |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        81            | Disarm                     |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        82            | Q_Assist 3Pos Sw           |          |    X    |         |
+----------------------+----------------------------+----------+---------+---------+
|        83            | ZIGZAG Auto                |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        84            | AIRMODE(not a flight mode) |    X     |         |         |
+----------------------+----------------------------+----------+---------+---------+
|        100           | Kill IMU1 (testing only!)  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        101           | Kill IMU2 (testing only!)  |    X     |    X    |    X    |
+----------------------+----------------------------+----------+---------+---------+
|        105           | GPS Disable Yaw            |    X     |    X    |    X    |
|                      | (testing only!)            |          |         |         |
+----------------------+----------------------------+----------+---------+---------+

.. raw:: html

   <table border="1" class="docutils">
   <tbody>
   <tr>
   <th>Option</th>
   <th>Description</th>
   </tr>

.. raw:: html

   </td>
   <tr>
   <td><strong>ZigZag Mode Learn Waypoints</strong></td>
   <td>

Sets zigzag point A and B. See :ref:`zigzag-mode` .

.. raw:: html

   </td>
   </tr>
   <tr>

   <td><strong>RunCam Control</strong></td>
   <td>

Allows starting and stopping video recording of compatible RunCam cameras. See :ref:`common-camera-runcam`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>RunCam OSD Control</strong></td>
   <td>

Enables control of RunCam cameras OSDs. See :ref:`common-camera-runcam`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Viso Align</strong></td>
   <td>

Align T265 Visual Odometry camera attitude to vehicle's.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Disarm</strong></td>
   <td>

Disarm vehicle unconditionally and immediately. Unlike Emergency Stop Motors, which waits for :ref:`DISARM_DELAY<DISARM_DELAY>` in Copter.

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Q_Assist 3Pos SW</strong></td>
   <td>

Low: disable Q_Assist entirely, Middle: Normal Q_Assist operation, High: Q_Assist active at all times. See Assisted Fixed Wing Flight section of :ref:`quadplane-flying`



.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>ZigZag Mode Auto Enable</strong></td>
   <td>

Enable automatic zigzag and sprayer in ZIGZAG mode. See :ref:`zigzag-mode`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>AIRMODE (not a regular flight mode)</strong></td>
   <td>

Enables and disables AIRMODE feature. See :ref:`airmode`

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>GPS Disable Yaw</strong></td>
   <td>

Disables yaw for testing (advanced users only!)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Kill IMU1 </strong></td>
   <td>

Disables IMU1 for testing (advanced users only!)

.. raw:: html

   </td>
   </tr>
   <tr>
   <td><strong>Kill IMU2 </strong></td>
   <td>

Disables IMU2 for testing (advanced users only!)

.. raw:: html

   </td>
   </tr>
   </tbody>
   </table>

------------------------------------------------------

PLANE
=====

On Quadplane Flying Page:
-------------------------

Change to WARNING box:

This can be managed somewhat with manual throttle control when manually transitioning, but in AUTO mode, a VTOL to fixed wing transition is currently done with :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` p on the forward motor until transition is complete, so very high currents can be experienced..

Changes to Assisted Fixed-Wing Flight:

- change From: "To enable quad assistance you should set Q_ASSIST_SPEED parameter to the airspeed below which you want assistance." To : Quad motor assistance is enabled if :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>`, :ref:`Q_ASSIST_ALT<Q_ASSIST_ALT>` , or :ref:`Q_ASSIST_ANGLE<Q_ASSIST_ANGLE>` are non-zero.
- remove: "The attitude assistance will only be used if Q_ASSIST_SPEED greater than zero."
- add after Q_ASSIST_ALT paragraph: Assistance can also be enabled, disabled, or forced by setting an RC switch to ``RCx_OPTION`` = 82. If that channel is below  1200us (LOW), then assistane is unconditionally disabled, if above 1800us, (HIGH) then assistance is always enabled. Fot other RC values, assistance will be enabled as explained above.

-----------------------------------------------------

Copter
======

On BendyRuler page:
-------------------

- replace OA_LOOKAHEAD label with :ref:`OA_BR_LOOKAHEAD<OA_BR_LOOKAHEAD>`
- add these params under configuration:
    - :ref:`OA_BR_CONT_RATIO<OA_BR_CONT_RATIO>` : BendyRuler will avoid changing bearing unless ratio of previous margin from obstacle (or fence) to present calculated margin is atleast this much.
    - :ref:`OA_BR_CONT_ANGLE<OA_BR_CONT_ANGLE>` : BendyRuler will resist changing current bearing if the change in bearing is over this angle

[copywiki destination="plane,copter,rover,dev"]
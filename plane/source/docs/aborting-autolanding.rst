.. _aborting-autolanding:

===========================
How to Abort an Autolanding
===========================
A landing-abort mechanism is provided to allow you to abort a autolanding sequence in a safe, controlled, and expected way. Custom abort behaviour can be pre-programmed as part of the mission or you can use the default abort mechanism. To enable autolanding abort feature, set param :ref:`LAND_ABORT_THR<LAND_ABORT_THR>` = 1.
 
There are three aspects of this feature:

1. :ref:`Triggering an abort <trigger_an_abort>`
#. :ref:`The behavior during the abort <behavior_during_the_abort>`
#. :ref:`The mission state after the abort completes <mission_state_after_an_aborted_landing_completes>`


.. _trigger_an_abort:

Step 1) Abort land triggers
---------------------------
The are three ways to trigger an auto-landing abort. All of them will only work while in AUTO mode and currently executing a ``NAV_LAND`` waypoint mission item:

-  **Send the ``MAV_CMD_DO_GO_AROUND`` command using a GCS.** Mission Planner has a button labeled "Abort Landing" on the FlightData Actions tab.
-  **RC input Throttle > 90%**. This will trigger an abort while staying in AUTO mode. The throttle only needs to be high briefly to trigger it. Don't forget to lower it!
-  **Mode change**. For human piloted landing abort you can switch out of AUTO mode into, for example MANUAL/STABILIZE/FBWA, and navigate the aircraft safely however you'd like. Using this method will skip abort behavior step 2 because it is being done manually. When switching back to AUTO the mission will resume as described in step 3 below.

.. _behavior_during_the_abort:

Step 2) Abort land flight behavior
----------------------------------
The abort behaviour has a default configuration and does not require a pre-planned mission. The default abort behavior is to simulate an auto-takeoff: pitch up at least 10 degrees and set throttle to :ref:`TKOFF_THR_MAX<TKOFF_THR_MAX>` and hold the heading until it reaches a target altitude of 30m. It is possible to override the pitch and altitude to allow for a customized behavior.

- Pitch minimum. If there was a NAV_TAKEOFF ever executed on this mission then the same minimum pitch (param1) will be re-used here.
- Target altitude. If NAV_LAND param1 is >0 then it is used as a target altitude in meters. Else If a NAV_TAKEOFF was ever executed on this mission then the same altitude (param7) will be re-used here.
  
This step is skipped if the abort trigger is via mode change because it is assumed the pilot manually took over and flew the aircraft to a safe altitude at the pitch and throttle of their choosing.

.. _mission_state_after_an_aborted_landing_completes:

Step 3) Mission state after an aborted landing completes
--------------------------------------------------------
Once an abort land has completed, by either reaching the target altitude or switching back to AUTO, the mission index will have changed and you will no longer be executing a NAV_LAND command. The mission index will change to be one of these three options and checked for in this order:

- If the NAV_LAND mission item is followed by mission item :ref:`CONTINUE_AND_CHANGE_ALT <mav_cmd_nav_continue_and_change_alt>` with param1 = 0 or 1 then the mission index will increment once to that command and execute it like normal. This can be followed by further post-abort mission planning for any custom planned mission behavior.
- Else If there is a :ref:`DO_LAND_START <mav_cmd_do_land_start>` in the mission then it jumps to that index.
- Else the mission index decrements once to be the index before the NAV_LAND. This will ensure the same landing approach is repeated.


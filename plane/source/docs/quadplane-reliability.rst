.. _quadplane-reliability:

===========================================
Tips for Improving QuadPlane Safe Operation
===========================================

Large commercial QuadPlanes not only represent significant investments, but also can have safety concerns due to their size and speed. Often these vehicles are operated near their operational limits to obtain maximum duration and payload delivery. Here are some tips to prevent accidents, based on reported incidents and subsequent analysis by ArduPilot support engineers.

- Utilize an autopilot with redundant IMUs and sensors
- Utilize a quality airspeed sensor, compass, and GPS. Redundant sensors are suggested, if possible.
- Read and follow the Wiki's :ref:`plane-configuration-landing-page` and :ref:`quadplane-setup` instructions. Be sure to setup the :ref:`Q_M_BAT_VOLT_MAX<Q_M_BAT_VOLT_MAX>` and :ref:`Q_M_BAT_VOLT_MIN<Q_M_BAT_VOLT_MIN>` parameters.
- Follow the Wiki's :ref:`quadplane-vtol-tuning-process` instructions before the first hovers. In step 10, using the QuickTune LUA script is preferable to manual TX tuning, or using the AUTOTUNE mode. Be sure to :ref:`autocal the airspeed sensor<calibrating-an-airspeed-sensor>` in the first Fixed Wing flight and follow the :ref:`Fixed Wing Tuning<tuning-quickstart>` and subsequent tuning pages' instructions.
- After the first VTOL/FW/VTOL flights, use MAVExplorer's MAGFIT utility to refine the :ref:`compass calibration <common-magfit>`.

Suggested Parameters (differing from defaults)
==============================================

- Although it requires setup per flying location, the use of :ref:`DO_LAND_START<do_land_start>` missions and :ref:`RTL_AUTOLAND<RTL_AUTOLAND>` is advised, to control approach direction.
- Be sure to configure the landing sequence with long enough base and final legs so as to assure the turn offshoots are corrected before the final transition to VTOL on landing. A vehicle moving at 30m/s with 30deg maximum bank angle will only turn at 9 deg/sec resulting a >200meter turn radius. Bear in mind that winds can widen the turn radius significantly.
- Be sure to set the :ref:`TECS_LAND_ARSPD<TECS_LAND_ARSPD>` parameter, it should be set to just above :ref:`AIRSPEED_MIN<AIRSPEED_MIN>`, which should be 25% above stall speed, otherwise the normal cruise airspeed will be used on the approach, greatly increasing the radius of turns and their recovery to the mission path.
- Test the above landing sequence in STIL with the -f quadplane frame, using your parameters for TECS, and pitch/roll angle limits, to assure it can execute the landing pattern correctly.
- Use :ref:`Q_OPTIONS<Q_OPTIONS>`, bit 5 = 1 to assure that RC failsafes while in VTOL modes are treated as QRTLs instead of QLANDs, if not operating on a mission with :ref:`DO_LAND_START<do_land_start>`.
- Set :ref:`Q_ASSIST_SPEED<Q_ASSIST_SPEED>` to be about 3m/s less than :ref:`AIRSPEED_MIN<AIRSPEED_MIN>`, but still above the vehicles stall speed, to avoid undesired assist operation within the normal operating speed envelope.
- Set the transition failure timer :ref:`Q_TRANS_FAIL<Q_TRANS_FAIL>` to an appropriate time and set :ref:`Q_TRANS_FAIL_ACT<Q_TRANS_FAIL_ACT>` to QRTL, unless operating at BVLOS distances. In this case, it might be better to set the action to warning only and decide what do as the situation warrants.
- When setting up the Battery Failsafe, use :ref:`BATT_FS_LOW_ACT<BATT_FS_LOW_ACT>` = 6 for a fixed wing return, loiter down to :ref:`Q_RTL_ALT<Q_RTL_ALT>`, and QLAND.
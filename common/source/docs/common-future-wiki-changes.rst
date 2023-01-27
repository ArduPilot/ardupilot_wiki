.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


[copywiki destination="plane,copter,rover,blimp"]

on :ref:`common-auxiliary-functions`, add:
==========================================

+----------------------+----------------------------+----------+---------+---------+
| **RCx_OPTION value** | **Feature Description**    |**Copter**|**Plane**|**Rover**|
+----------------------+----------------------------+----------+---------+---------+
|       170            |  QSTABILIZE mode           |          |  x      |         |
+----------------------+----------------------------+----------+---------+---------+

on :ref:`common-downloading-and-analyzing-data-logs-in-mission-planner` page:
=============================================================================

under Logging Parameters, change:
- :ref:`LOG_DISARMED<LOG_DISARMED>`: Setting to one will start logging when power is applied, rather than at the first arming of the vehicle. Usefull when debugging pre-arm failures.

to
- :ref:`LOG_DISARMED<LOG_DISARMED>`: Setting to 1 will start logging when power is applied, rather than at the first arming of the vehicle. Usefull when debugging pre-arm failures. Setting to 2 will only log on power application other than USB power to prevent logging while setting up on the bench.

on :ref:`common-powermodule-landingpage`, add:
==============================================

.. toctree::
    :maxdepth: 1

    Synthetic Current Sensor/Analog Voltage Monitor <common-synthetic-current-monitor>



on :ref:`common-uavcan-setup-advanced`, :ref:`mission-planner-initial-setup`, :ref:`common-slcan-f4`, and :ref:`common-slcan-f7h7` pages add the following note:
================================================================================================================================================================
.. note:: SLCAN access via COM port is disabled when armed to lower cpu load. Use SLCAN via MAVLink instead.

on :ref:`common-external-ahrs` add:
===================================

under Supported Systems add:

  - VectorNav VN-100AHRS

under Setup replace with:

VectorNav300 or Parker Lord
---------------------------

    - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 11 (External AHRS)

    - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV) or 2 (Parker Lord)

This will replace ArduPilot’s internally generated INS/AHRS subsystems with the external system

VectorNav100
------------

    - :ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>` = 3 (ArduPilot's EKF3)

    - :ref:`EAHRS_TYPE<EAHRS_TYPE>` = 1 (VectorNAV)

    - :ref:`EAHRS_OPTIONS<EAHRS_OPTIONS>` bit 0 set to 1 ("1" value) to disable its compensation of the sensor biases, letting EKF3 do that (since there is no internal GPS to provide the best estimates)

- for all of the above, set the ``SERIALx_PROTOCOL`` to “36” (AHRS) and ``SERIALx_BAUD`` to “115” (unless you have changed the external unit’s baud rate from its default value) for the port which is connected to the external AHRS unit.

on :ref:`common-efi` page, add:
===============================

In addition, ArduPilot allows the addition of new EFI controller drivers via :ref:`common-lua-scripts`. For examples, see the `HFE CAN EFI driver <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/drivers/EFI_HFE.md>`__ or the `SkyPower CAN driver <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/drivers/EFI_SkyPower.md>`__

[site wiki="plane"]
on :ref:`automatic-takeoff` page, add at bottom a new section:
===============================================================

Catapult Launch without an Airspeed Sensor
==========================================
Taking off without an airspeed sensor using a catapult may cause less than maximum throttle to be used due to high initial climb rates. For heavy vehicles, this may result in a stall due to the long time constants used in TECS to adjust throttle after the initial launch. The parameter :ref:`TKOFF_THR_MAX_T<TKOFF_THR_MAX_T>` can be used to force maximum throttle for a time, irrespective of climb rates from an initial catapult launch to allow the vehicle to obtain sufficient speed.

on :ref:`common-mavlink-mission-command-messages-mav_cmd` page:
===============================================================

under DO_CHANGE_SPEED command add:" For Airspeed, a value of -2.0 indicates return to normal cruise airspeed" to speed parameter description.

on :ref:`acro-mode` page, in section "Acro Locking", add:
=========================================================

It is recommended that it be set to "2", instead of "1", in order to use a quarternion based control system with much better performance than the older system. In order for this to be effective, yaw rate control (:ref:`YAW_RATE_ENABLE<YAW_RATE_ENABLE>`) must be "1" and the yaw rate controller tuned using :ref:`Autotune <automatic-tuning-with-autotune>` for best performance.

on the :ref:`automatic-tuning-with-autotune` page:
==================================================

add in the setup section:
-------------------------

The :ref:`AUTOTUNE_AXES<AUTOTUNE_AXES>` bitmask selects which axes will be tuned while in Autotune. Default is roll, pitch and yaw.

change note in the YAW Controller section to:
---------------------------------------------

.. note:: while AutoTuning with this controller enabled, roll inputs will result in yaw outputs also, allowing more coordinated turns with the yaw controller active. This will normally result in simultaneously tuning the yaw controller with the roll controller, but not necessarily completing the yaw tune when the roll tune finishes. Also, there may be seemingly excessive rudder applied initially in the roll tune on vehicles with large yaw authority, until the tune progresses.

On :ref:`apms-failsafe-function` page:
======================================

Add note to Battery Failsafe Section:

.. note:: the battery low failsafe voltage must be higher than the battery critical failsafe voltage or a pre-arm error will occur.

On :ref:`automatic-landing` page, under Controlling the Flare:
==============================================================

The landing controller sets a point before the touchdown as the expected flare start point. This "flare_aim" point is calculated from the :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>` and :ref:`TECS_LAND_SINK<TECS_LAND_SINK>` for the expected duration of the flare before the actual touchdown. If consistently landing long or short, this point can be adjusted using the :ref:` TECS_FLARE_AIM<TECS_FLARE_AIM>` parameter. If landing too short, decrease the percentage from its default of 50%, conversely, increasing it if landing too long.

The transition from the glide-slope sink rate to the flare sink rate is controlled by the :ref:`TECS_FLARE_HGT<TECS_FLARE_HGT>` parameter. The start of the flare will occur at :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>` and the sink rate will be gradually adjusted to :ref:`TECS_LAND_SINK<TECS_LAND_SINK>` at the :ref:`TECS_FLARE_HGT<TECS_FLARE_HGT>` to avoid a rapid pitch change at the beginning of the flare, which would tend to create a "ballooning" effect at the start of the flare. :ref:`TECS_FLARE_HGT<TECS_FLARE_HGT>` should be lower than :ref:`LAND_FLARE_ALT<LAND_FLARE_ALT>`.

On :ref:`precision-autolanding` page, under the Approach Airspeed section, add:
===============================================================================

The :ref:`LAND_WIND_COMP<LAND_WIND_COMP>` parameter controls how much headwind compensation is used when landing. Headwind speed component multiplied by this parameter is added to :ref:`TECS_LAND_ARSPD<TECS_LAND_ARSPD>` value. Set to 0 to disable this. 

.. note:: The target landing airspeed value is still limited to being lower than :ref:`ARSPD_FBW_MAX<ARSPD_FBW_MAX>`.

on the :ref:`qrtl-mode` page add the following at the end of the page:
======================================================================

Return Altitude
===============

If VTOL motors are active when QRTL is entered (ie in a VTOL mode or QAssist), the vehicle will first climb vertically before returning home. The altitude to which the vehicle climbs depends on the distance from home as shown below given the parameters entered below. The return path can be shown by clicking on the graph anywhere, at the distance from home that QRTL will be entered.

Altitude will be above terrain if terrain following is enabled. If not, altitude will be above home.

.. note:: If the vehicle is within one and a half times :ref:`RTL_RADIUS<RTL_RADIUS>` or :ref:`WP_LOITER_RAD<WP_LOITER_RAD>` (whichever is greater), the vehicle will return in VTOL flight without transitioning into forward flight. Otherwise, it will immediately transition to forward flight at its current altitude, proceed like an RTL and transition back to VTOL for landing, unless :ref:`Q_OPTIONS<Q_OPTIONS>` bit 16 is set.

.. raw:: html

   <canvas id="QRTL_Altitude" style="width:100%;max-width:1200px"></canvas>

   <p>
   <label class="parameter_input_label" for="Q_RTL_ALT">Q_RTL_ALT</label>
   <input id="Q_RTL_ALT" name="Q_RTL_ALT" type="number" step="1" value="15" min="0" onchange="update()"/>

   <label class="parameter_input_label" for="Q_RTL_ALT_MIN">Q_RTL_ALT_MIN</label>
   <input id="Q_RTL_ALT_MIN" name="Q_RTL_ALT_MIN" type="number" step="1" value="10.0" min="0" onchange="update()"/>

   <label class="parameter_input_label" for="RTL_RADIUS">RTL_RADIUS</label>
   <input id="RTL_RADIUS" name="RTL_RADIUS" type="number" step="1" value="0" onchange="update()"/>

   <label class="parameter_input_label" for="WP_LOITER_RAD">WP_LOITER_RAD</label>
   <input id="WP_LOITER_RAD" name="WP_LOITER_RAD" type="number" step="1" value="60" onchange="update()"/>

   <label class="parameter_input_label" for="Q_LAND_FINAL">Q_LAND_FINAL</label>
   <input id="Q_LAND_FINAL" name="Q_LAND_FINAL" type="number" step="0.5" value="6" min="0" onchange="update()"/>
   </p>

   <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>

   <script>
   var chart
   var vehicle_dist = null
   var vehicle_alt = null
   function update() {
      var Q_RTL_ALT = parseFloat(document.getElementById("Q_RTL_ALT").value)
      var Q_RTL_ALT_MIN = parseFloat(document.getElementById("Q_RTL_ALT_MIN").value)
      var RTL_RADIUS = parseFloat(document.getElementById("RTL_RADIUS").value)
      var WP_LOITER_RAD = parseFloat(document.getElementById("WP_LOITER_RAD").value)
      var Q_LAND_FINAL = parseFloat(document.getElementById("Q_LAND_FINAL").value)

      if ((Q_LAND_FINAL + 1) > Q_RTL_ALT) {
         Q_RTL_ALT = Q_LAND_FINAL + 1
         document.getElementById("Q_RTL_ALT").value = Q_RTL_ALT
      }

      if (Q_RTL_ALT_MIN < Q_LAND_FINAL) {
         Q_RTL_ALT_MIN = Q_LAND_FINAL
         document.getElementById("Q_RTL_ALT_MIN").value = Q_LAND_FINAL
      } else if (Q_RTL_ALT_MIN > Q_RTL_ALT) {
         Q_RTL_ALT_MIN = Q_RTL_ALT
         document.getElementById("Q_RTL_ALT_MIN").value = Q_RTL_ALT
      }

      var radius = Math.max(Math.abs(RTL_RADIUS), Math.abs(WP_LOITER_RAD)) * 1.5
      var min_alt_radius = (Q_RTL_ALT_MIN/Q_RTL_ALT) * radius;

      var max_disp_rad = Math.ceil((radius * 1.2) / 10) * 10
      var max_disp_alt = Math.ceil((Q_RTL_ALT+5.0) / 10) * 10

      var alt = [{x:-5,             y:Q_RTL_ALT_MIN},
               {x:min_alt_radius, y:Q_RTL_ALT_MIN},
               {x:radius,         y:Q_RTL_ALT},
               {x:max_disp_rad+5, y:Q_RTL_ALT}]

      var land_final = [{x:-5,             y:Q_LAND_FINAL},
                        {x:max_disp_rad+5, y:Q_LAND_FINAL}]

      var threshold_alt = Math.max(Q_RTL_ALT * (vehicle_dist / Math.max(radius, vehicle_dist)), Q_RTL_ALT_MIN)

      var return_path_VTOL = []
      var return_path_FW = []

      if ((vehicle_dist!= null) && (vehicle_alt != null)) {
         if (vehicle_alt < threshold_alt) {
            return_path_VTOL.push({x:vehicle_dist, y:vehicle_alt})
            return_path_VTOL.push({x:vehicle_dist, y:threshold_alt})
            if (vehicle_dist < radius) {
               return_path_VTOL.push({x:0, y:threshold_alt})
            } else {
               return_path_VTOL.push({x:null, y:null})
               return_path_VTOL.push({x:0, y:Q_RTL_ALT})
               return_path_FW.push({x:vehicle_dist, y:threshold_alt})
               return_path_FW.push({x:0, y:Q_RTL_ALT})
            }
         } else {
            if (vehicle_dist < radius) {
               return_path_VTOL.push({x:vehicle_dist, y:vehicle_alt})
               return_path_VTOL.push({x:0, y:vehicle_alt})
            } else {
               return_path_VTOL.push({x:null, y:null})
               return_path_VTOL.push({x:0, y:Q_RTL_ALT})
               return_path_FW.push({x:vehicle_dist, y:vehicle_alt})
               return_path_FW.push({x:0, y:Q_RTL_ALT})
            }
         }
         return_path_VTOL.push({x:0, y:0})
      }

      if (chart == null) {
         chart = new Chart("QRTL_Altitude", {
            type : "scatter",
            data: {
               datasets: [
                  {
                     label: 'Return Path FW',
                     borderColor: "rgba(255,0,0,1)",
                     pointBackgroundColor: "rgba(255,0,0,1)",
                     data: return_path_FW,
                     fill: false,
                     showLine: true,
                     lineTension: 0,
                  },
                  {
                     label: 'Return Path VTOL',
                     color: "rgba(0,0,0,1)",
                     borderColor: "rgba(0,0,0,1)",
                     pointBackgroundColor: "rgba(0,0,0,1)",
                     data: return_path_VTOL,
                     fill: false,
                     showLine: true,
                     lineTension: 0,
                     spanGaps: false,
                  },
                  {
                     label: 'Return Altitude',
                     borderColor: "rgba(0,0,255,1.0)",
                     pointBackgroundColor: "rgba(0,0,255,1.0)",
                     data: alt,
                     fill: false,
                     showLine: true,
                     lineTension: 0,
                  },
                  {
                     label: 'Q_LAND_FINAL',
                     borderColor: "rgba(0,255,0,0.25)",
                     pointBackgroundColor: "rgba(0,255,0,0.25)",
                     data: land_final,
                     fill: false,
                     showLine: true,
                     lineTension: 0,
                  }
               ]
            },
            options: {
               animation: { duration: 200 },
               scales: {
                  y: {
                     title: { display: true, text: "Altitude (m)" },
                     min: 0.0,
                     max: max_disp_alt
                  },
                  x: {
                     title: { display: true, text: "Distance from home (m)" },
                     min: 0.0,
                     max: max_disp_rad
                  }
               },
               onClick: (e) => {
                  var canvasPosition = Chart.helpers.getRelativePosition(e, chart)
                  const {scales: {x, y}} = chart
                  vehicle_dist = x.getValueForPixel(canvasPosition.x)
                  vehicle_alt = y.getValueForPixel(canvasPosition.y)
                  update()
               },
               plugins: {
                  legend: {
                     labels: {
                        usePointStyle: true,
                        pointStyle: 'rectRounded',
                        pointStyleWidth: 50,
                        filter: function(legendItem, data) {
                           return data.datasets[legendItem.datasetIndex].data.length > 0
                        }
                     },
                     onClick: null
                  }
            }
            }
         });
      } else {
         chart.data.datasets[0].data = return_path_FW
         chart.data.datasets[1].data = return_path_VTOL
         chart.data.datasets[2].data = alt
         chart.data.datasets[3].data = land_final
         chart.options.scales.x.max = max_disp_rad
         chart.options.scales.y.max = max_disp_alt
         chart.update()
      }
   }
   update()
   </script>

[/site]

[site wiki="copter"]

on :ref:`turtle-mode` page, add the following notes:
====================================================


-  Turtle mode cannot be entered unless throttle is zero
-  Upon entry to turtle mode the motors stay disarmed, but the notfiy LEDs flash
-  Raising the throttle, the motors arm, and motors spin. Lowering throttle to zero disarms the motors
-  Motors spin only when throttle is raised


on :ref:`common-transmitter-tuning` page, add:
==============================================

under TUNE parameter table:

+--------+-------------------------+----------------------------------------------------------------------+
|Value	 |Meaning                  | Parameter                                                            |
+========+=========================+======================================================================+
|59      |Position Control Max     |  :ref:`PSC_ANGLE_MAX<PSC_ANGLE_MAX>`                                 |
|        | Lean Angle              |                                                                      |
+--------+-------------------------+----------------------------------------------------------------------+
[/site]

[site wiki="copter"]

on :ref:`common-airspeed-sensor` page, add new param:
=====================================================

- :ref:`ARSPD_ENABLE<ARSPD_ENABLE>` = 1 to allow use of airspeed sensor and to show other airspeed parameters

[/site]
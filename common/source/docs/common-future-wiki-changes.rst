.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================
Will be in future 4.7 release and is currently in master ("latest")

[copywiki destination="plane,copter,rover,blimp"]

New Board Support
=================

New Peripheral Support
======================

New Features
============


[site wiki="plane"]

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

[/site]
[site wiki="rover"]

[/site]

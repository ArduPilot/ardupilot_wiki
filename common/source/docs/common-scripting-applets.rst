.. _common-scripting-applets:
[copywiki destination="plane,copter,rover,dev"]

==============
Script Applets
==============

These are scripts that require no user editing before use, and can be found `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/applets>`_ . Each of these has a .md file of the same name detailing its capabilities, use, and setup.

Below is a list of applets and their use as of this writing. More are being added continually.

==========================================  ===========
Applet Name                                 Description
==========================================  ===========
BattEstimate.lua                            Battery state of charge estimator
Gimbal_Camera_Mode.lua                      Select auto or manual gimbal/camera control mode
Heli_IM_COL_Tune.lua                        Traditional IM_COL_TUNE tuning applet
Heli_idle_control.lua                       Allows manual or automatic rpm control while in ground idle condition
Hexsoon LEDs.lua                            Hexsoon Multicopter LED control script
MissionSelector.lua                         Selects between one of three missions on boot depending on switch state
ONVIF_Camera_Control.lua                    CNIF Camera Control
Param_Controller.lua                        Allows switch selection of parameter sets at boot
QuadPlane_Low_Alt_FW_mode_prevention.lua    Prevents changing to FW at low altitude
RockBlock.lua                               Rockblock modem
Script_Controller.lua                       Allows selection of multiple scripts and missions on SD Card
SmartAudio.lua                              Smart Audio control
UniversalAutoLand.lua                       Allow easy, unpre-planned operation at any location with the protection of a do-land-start autoland sequence
VTOL-quicktune.lua                          VTOL-quicktuning applet
advance-wp.lua                              Advance the current mission waypoint via an RC switch
ahrs-set-origin.lua                         Sets the AHRS/EKF origin to a specified Location
ahrs-source-extnav-optflow.lua              Help vehicles automatically switch between ExternalNav and optical flow
camera-change-setting.lua                   Allows changing some camera settings that are not normally used by the autopilot
copter-deadreckon-home.lua                  Flies towards home on loss of GPS
copter-slung-payload.lua                    Reduce a slung payload's oscillation
copter_terrain_brake.lua                    Useful when flying in LOITER mode in steep terrain
follow-target-send.lua                      Sends the FOLLOW_TARGET mavlink message to allow other vehicles to follow this one
forward_flight_motor_shutdown.lua           Forward flight motor shutdown
leds_on_a_switch.lua                        Control NTF_LED_BRIGHT value using an AUX switch
motor_failure_test.lua                      Motor failure testing script
mount-poi.lua                               Displays the location (lat, lon and altitude) that the gimbal is currently pointing towards
net_webserver.lua                           This implements a web server for boards that have networking support
pelco_d_antennatracker.lua                  Maps AntennaTracker outputs to corresponding Pelco-D messages
plane_aerobatics.lua(in Aerobatics subdir)  Autonomous trajectory precise aerobatics
plane_package_place.lua                     Quadplane payload place script
plane_precland.lua                          Precision landing/loitering system for QuadPlanes
plane_ship_landing.lua                      Script to automate moving vehicle landing with a beacon
repl.lua                                    Interactive REPL (read-evaluate-print-loop) for the Lua scripting
revert_param.lua                            Switch allows you toevert all the parameter changes to the values from startup
rover-quicktune.lua                         QuicK tuning applet for Rovers
runcam_on_arm.lua                           Starts/stops video recording on arm/disarm
video-stream-information.lua                Populate the VIDEO_STREAM_INFORMATION mavlink message 
sport_aerobatics.lua(in Aerobatics subdir)  Autonomous rate based aerobatics
winch-control.lua                           Allows the winch to be deployed or retracted using an auxiliary switch
x-quad-cg-allocation.lua                    Scriptable control allocation matrix to request asymmeterical front and back thrust
==========================================  ===========

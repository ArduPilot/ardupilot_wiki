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
Hexsoon LEDs.lua                            Hexsoon Multicopter LED control script
MissionSelector.lua                         Selects between one of three missions on boot depending on switch state
ONVIF_Camera_Control.lua                    CNIF Camera Control
QuadPlane_Low_Alt_FW_mode_prevention.lua    Prevents changing to FW at low altitude
RockBlock.lua                               Rockblock modem
Script_Controller.lua                       Allows selection of multiple scripts and missions on SD Card
SmartAudio.lua                              Smart Audio control
VTOL-quicktune.lua                          VTOL-quicktuning applet
copter-deadreckon-home.lua                  flies towards home on loss of GPS
forward_flight_motor_shutdown.lua           Forward flight motor shutdown
motor_failure_test.lua                      Motor failure testing script
mount-poi.lua                               Displays the location (lat, lon and altitude) that the gimbal is currently pointing towards
plane_aerobatics.lua(in Aerobatics subdir)  Autonomous trajectory precise aerobatics
plane_package_place.lua                     Quadplane payload place script
plane_ship_landing.lua                      Script to automate moving vehicle landing with a beacon
runcam_on_arm.lua                           Starts/stops video recording on arm/disarm 
sport_aerobatics.lua(in Aerobatics subdir)  Autonomous rate based aerobatics
==========================================  ===========

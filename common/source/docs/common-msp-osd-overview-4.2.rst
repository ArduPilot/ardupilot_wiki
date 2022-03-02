.. _common-msp-osd-overview-4.2:

=======
MSP OSD
=======

ArduPilot supports 2 types of MSP OSDs:

 - Telemetry based OSDs such as DJI FPV Goggles V1/V2, DJI Goggles RE, FatShark ByteFrost, FatShark SharkByte, MWOSD, etc
 - DisplayPort (aka CANVAS MODE) based OSD's such as FatShark SharkByte (fw 09042021 and later) and MWOSD

Telemetry based OSDs will render OSD panel items on screen with their own engine, so ArduPilot has no control of how the items look.
Another limit of telemetry based OSDs is that there's no way for ArduPilot to add new panel items at will, it's the vendor's responsibility to add new features by rolling out new firmware releases.

DisplayPort, on the contrary, is an MSP protocol extension that allows to remotely draw text on compatible external OSDs, DisplayPort is also known as CANVAS MODE.
Basically it’s a remote text only frame buffer that uses local fonts (local to the rendering engine i.e. the OSD hardware) to render strings sent via MSP.

Telemetry based OSD
===================

DJI V1 FPV Goggles

 .. image:: ../../../images/msp_dji_fpv_goggles.jpeg
    :target: ../_images/msp_dji_fpv_goggles.jpeg

 
 DJI Goggles RE
 
 .. image:: ../../../images/msp_dji_goggles_re.jpeg
    :target: ../_images/msp_dji_goggles_re.jpeg
 
Features
--------
 
 - ArduPilot currently supports all of the OSD panel items provided by the V1 and V2 DJI FPV Goggles, details are given in the table below.
 - Changing display units other than metric and imperial are not currently supported.
 - Multiple screens and remote switching of those screens is supported.
 - Displaying statistics on a dedicated screen is supported, see below for details.
 - Warning levels for RSSI, Voltage, etc. currently not supported

Configuration
-------------

To enable MSP OSD, set the following parameters ( example using SERIAL port 2 as the port to attach to the DJI Air unit using both TX and RX lines):

 - :ref:`OSD_TYPE<OSD_TYPE>` = 3
 - :ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>` = 33
 - :ref:`MSP_OPTIONS<MSP_OPTIONS>` = 0 (polling mode)

.. note:: DJI OSD must be enabled: in SETTINGS->DISPLAY->CUSTOM OSD menu of goggles

OSD Panel Items
---------------

+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSD Parameter | Notes                                                                                                                                                                                                                                                                                                |
+===============+======================================================================================================================================================================================================================================================================================================+
| OSDn_ALTITUDE | Home relative altitude                                                                                                                                                                                                                                                                               |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_ARMING   | Arming status, hidden when armed otherwise showing DISARMED                                                                                                                                                                                                                                          |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_ASPEED   | Please refer to OSDn_GSPEED for more info on enabling airspeed display                                                                                                                                                                                                                               |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_BAT_VOLT | First battery voltage                                                                                                                                                                                                                                                                                |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_BATBAR   | First battery remaining percentage rendered as a bar based on declared capacity and consumed mAh                                                                                                                                                                                                     |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_BATUSED  | First battery consumed mAh                                                                                                                                                                                                                                                                           |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_CELLVOLT | First battery average cell voltage, if automatic cell detection fails please override with MSP_OSD_NCELLS                                                                                                                                                                                            |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_CLK      | Realtime clock (requires GPX fix)                                                                                                                                                                                                                                                                    |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_CRSSHAIR | Artificial horizon is not supported so crosshair is often kept hidden                                                                                                                                                                                                                                |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_CURRENT  | First battery current                                                                                                                                                                                                                                                                                |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_ESCTEMP  | On DJI V1/V2 Goggles this will report the highest ESC temperature                                                                                                                                                                                                                                    |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_FLTMODE  | DJI hardware does not support ArduPilot's flight modes! This item will generally be blank and only show !FS! while in failsafe!                                                                                                                                                                      |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_GPSLAT   | GPS Latitude in decimal format                                                                                                                                                                                                                                                                       |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_GPSLONG  | GPS Longitude in decimal format                                                                                                                                                                                                                                                                      |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_GSPEED   | This item shows ground speed unless ``OSDn_ASPEED_EN`` = 1 in which case it will show true airspeed if an airspeed sensor is present or estimated airspeed otherwise. The position on screen is set by ``OSDn_GSPEED_X`` and ``OSDn_GSPEED_Y`` regardless of the value of ``OSDn_ASPEED_EN``         |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_HEADING  | Not supported by DJI V1/V2 Goggles                                                                                                                                                                                                                                                                   |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_HOMEDIR  | Rotating arrow pointing to home                                                                                                                                                                                                                                                                      |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_HOMEDIST | Distance from home                                                                                                                                                                                                                                                                                   |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_HORIZON  | Not supported by DJI V1/V2 Goggles                                                                                                                                                                                                                                                                   |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_MESSAGE  | This will display status text messages as rolling text. Status text messages will be hidden after a couple seconds and the panel will show the current flightmode. If ``OSDn_WIND_EN`` is set to 1, this item also displays wind info next to the current flight mode as a rotating arrow and speed. |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_PITCH    | Pitch angle                                                                                                                                                                                                                                                                                          |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_POWER    | Instant power calculated as voltage * current                                                                                                                                                                                                                                                        |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_ROLL     | Roll angle                                                                                                                                                                                                                                                                                           |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_RSSI     | Rssi as configured in :ref:`RSSI_TYPE`                                                                                                                                                                                                                                                               |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_SATS     | On DJI V1/V2 Goggles when there's no fix it will report 14 sats, this is a known DJI bug                                                                                                                                                                                                             |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_SIDEBARS | Not supported by DJI V1/V2 Goggles                                                                                                                                                                                                                                                                   |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_VSPEED   | Vertical speed                                                                                                                                                                                                                                                                                       |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OSDn_WIND     | Please refer to OSDn_MESSAGE for wind speed and direction rendering                                                                                                                                                                                                                                  |
+---------------+------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+


OSD Panel Item Configuration
----------------------------

Each OSD panel item uses a set of three variables to be set: 

- ``OSDn_<ITEM>_EN`` - activates the respective panel item on screen "n" when set to 1.
- ``OSDn_<ITEM>_X`` and ``OSDn_<ITEM>_Y`` set the horizontal and vertical position of the item, starting with ``X = 0`` and ``Y = 0`` in the upper left corner of your screen. 

.. note::    ArduPilot calculates a sensor-less airspeed estimate that is used if no sensor is present or fails. ARSPD_TYPE must be set to zero in order to display this value as the airspeed item, if no sensor is present.
    

Screens and screen switching 
----------------------------

For multiple screen layouts, each screen's "OSD" parameter label is trailed by a number, starting with "1". For example,  ``OSDn_<ITEM>_x`` is a parameter "x" associated with screen 1's "ITEM" panel. 

- Set ``OSDn_<ITEM>_EN`` =1 to enable screen "n" display of this item. This allows one to set individual items active on one screen but have them switched off on another screen. Up to 4 screens are optionally available, and can be individually enabled.

- Set parameters ``OSDn_CHAN_MIN`` and ``OSDn_CHAN_MAX`` to adjust RC channel pwm limits to use for switching to a respective screen. Be sure to have the ranges non-overlapping.

There are different switch-method options to meet individual RC systems switch layout requirements. These can be set by parameter: :ref:`OSD_SW_METHOD<OSD_SW_METHOD>`.
The options are:

- 0 = switches to next screen if the set RC channel's (:ref:`OSD_CHAN<OSD_CHAN>`) value is changed
- 1 = directly selects a screen based on the set pwm limits for each respective screen. RC channel value must change for new pwm value to be recognized.
- 2 = toggles screens on a low to high transition of set RC channel. keeps toggling to next screen every second while channel value is kept high

Displaying statistics on a dedicated screen
-------------------------------------------

Displaying statistics on a dedicated screen requires enabling at least one extra screen by setting the respective ``OSDn_ENABLE`` to 1.
By default, ArduPilot has only one screen active so in a typical setup one would set (:ref:`OSD2_ENABLE<OSD2_ENABLE>`) = 1 and then enabling the OSD stats panel on screen 2 by setting (:ref:`OSD2_STATS_EN<OSD2_STATS_EN>`) = 1.

When the OSD switches to this screen it will check the value of the :ref:`OSD2_STATS_EN<OSD2_STATS_EN>` parameter and if enabled it will override the default behavior of the following OSD items:

 - OSDn_MESSAGE will display STATS followed by flight time
 - OSDn_ALTITUDE will display max altitude
 - OSDn_BAT_VOLT will display min voltage
 - OSDn_CURRENT will display max current
 - OSDn_GSPEED will display max ground speed (or airspeed if ``OSDn_ASPEED_EN`` is set to 1)
 - OSDn_HOMEDIST will alternates max distance from home and total traveled distance every 2 seconds
 - OSDn_RSSI will display min rssi

DisplayPort OSD
===============

FatShark's SharkByte using ArduPilot custom fonts

.. image:: ../../../images/msp_osd_displayport.jpg
   :target: ../_images/msp_osd_displayport.jpg

Features
--------

DisplayPort OSDs can render all the panel items supported by the ArduPilot's onboard OSD.
Features such as multiple screen switching, multiple units and statistics are supported as well, please refer to the :ref:`onboard OSD documentation <common-osd-overview>`  for more info.

By setting :ref:`MSP_OPTIONS<MSP_OPTIONS>` bit 3 to 1 one can force ArduPilot to impersonate betaflight and use a betaflight compatible font table on the remote OSD system.
This is required if the remote OSD system does not have an ArduPilot compatible fonts table. MWOSD already supports custom fonts and therefore does not require this hack while FatShark's SharkByte will support custom fonts in a future release.
Default behaviour is to use the ArduPilot fonts table.

Configuration
-------------

To enable MSP DisplayPort OSD, set the following parameters (using SERIAL port 2 as the port to attach to the Air unit using both TX and RX lines):

 - :ref:`OSD_TYPE<OSD_TYPE>` = 5
 - :ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>` = 42

Testing OSD with SITL
=====================

OSD functionality can be tested and panel items adjusted without autopilot or video hardware using the :ref:`Software In The Loop (SITL) simulator <dev:sitl-simulator-software-in-the-loop>` setup. Follow those SITL-Instructions to setup a simulation environment. Run the simulator on current source code using ``--osdmsp`` option to build the OSD code into the simulator. For example, for a plane simulation:

::

    sim_vehicle.py -v ArduPlane --console --osdmsp

A graphical DJI style MSP OSD simulation in a separate window will be opened with the other simulation windows using a typical set of OSD panel parameters, located at libraries/AP_MSP/Tools/osdtest.parm . Then the OSD elements can be customized by their parameters using the  MSP OSD emulation program to visualize the OSD. 

.. note:: You could also use these parameters to initially setup the MSP OSD configuration for use with goggles, but you may have to change the ``SERIALx_PROTOCOL`` parameter to match the actual serial port that you will be using.

.. note:: The emulation supports multiple screens and stats, but not yet Display Port

.. note:: The emulation does not support units other than metric

.. image:: ../../../images/msp_osd_python.png
   :target: ../_images/msp_osd_python.png

.. image:: ../../../images/msp_osd_python_stats.png
   :target: ../_images/msp_osd_python_stats.png

By changing the OSD panel items' parameters, a live update on their placement can be seen in this emulator.

Using Mission Planner to Configure the Layout
=============================================

Mission Planner(MP) has a tab in its CONFIG menu to configure the on-board OSD many autopilots integrate. This same configuration tab can be used to configure the OSD panels. In fact, you can do that while the SITL program and MSP OSD emulation window are active by connecting Mission Planner running on the same computer, or networked computer, to MAVProxy, using this command in MAVProxy:

::

    output add <ip address of box running Mission Planner>:14550


.. note:: if MP is running on the same PC, the ip address would be 127.0.0.1 (local host address)

Video
=====

.. youtube:: gT4R3E_7Z_0

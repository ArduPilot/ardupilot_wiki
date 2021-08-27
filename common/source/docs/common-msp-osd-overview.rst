.. _common-msp-osd-overview:

=======
MSP OSD
=======

Ardupilot supports 2 types of MSP OSDs:
 - Telemetry based OSDs such as DJI FPV Goggles V1/V2, DJI Goggles RE, FatShark ByteFrost, FatShark SharkByte, MWOSD, etc
 - DisplayPort (aka CANVAS MODE) based OSD's such as FatShark SharkByte (fw 09042021 and later) and MWOSD

Telemetry based OSDs will render OSD panel items on screen with their own engine, so ArduPilot has no control of how the items look.
Another limit of telemetry based OSDs is that there's no way for ArduPilot to add new panel items at will, it's the vendor's responsability to add new features by rolling out new firmware releases.

DisplayPort, on the contrary, is an MSP protocol extension that allows to remotely draw text on compatible external OSDs, DisplayPort is also known as CANVAS MODE.
Basically it’s a remote text only frame buffer that uses local fonts (local to the rendering engine i.e. the OSD hardware) to render strings sent via MSP.

Telemetry based OSD
=======================

 .. image:: ../../../images/msp_dji_fpv_goggles.jpeg
    :target: ../_images/msp_dji_fpv_goggles.jpeg

 .. image:: ../../../images/msp_dji_goggles_re.jpeg
    :target: ../_images/msp_dji_goggles_re.jpeg
 
Features
--------
 
 - ArduPilot currently supports many, but not all, of the OSD panel items as noted in the section below.
 - Changing display units other than metric and imperial are not currently supported.
 - Multiple screens and remote switching of those screens is not supported.
 - Warning levels for RSSI, Voltage, etc. currently not supported

Configuration
-------------

To enable MSP OSD, set the following parameters (using SERIAL port 2 as the port to attach to the DJI Air unit using both TX and RX lines):

 - :ref:`OSD_TYPE<OSD_TYPE>` = 3
 - :ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>` = 33
 - :ref:`MSP_OPTIONS<MSP_OPTIONS>` = 0 (polling mode)

.. note:: DJI OSD must be enabled: in SETTINGS->DISPLAY->CUSTOM OSD menu of goggles

Panel Items Currently Supported
-------------------------------

- Messages
- Altitude
- Bat_volt
- Bat mah
- Average cell voltage
- RSSI
- Current
- Batused % as horizontal gauge
- Sats
- Fight mode
- Ground speed
- Airspeed (only as override of ground speed)
- Vertical Speed
- Wind Direction and Speed
- Home Direction
- Home Distance
- Roll_angle
- Pitch_angle
- ESC Temperature
- Clock
- Gps_latitude
- Gps_longitude
- Arming status


OSD Panel Item Configuration
----------------------------

Each OSD panel item uses a set of three variables to be set: 

- ``OSDn_<ITEM>_ENA`` - activates the respective item when set to 1.
- ``OSDn_<ITEM>_X`` and ``OSDn_<ITEM>_Y`` set the horizontal and vertical position of the item, starting with ``X = 0`` and ``Y = 0`` in the upper left corner of your screen. 

.. note::    ArduPilot calculates a sensor-less airspeed estimate that is used if no sensor is present or fails. ARSPD_TYPE must be set to zero in order to display this value as the airspeed item, if no sensor is present.
    

Testing OSD with SITL
---------------------

OSD functionality can be tested and panel items adjusted without autopilot or video hardware using the :ref:`Software In The Loop (SITL) simulator <dev:sitl-simulator-software-in-the-loop>` setup. Follow those SITL-Instructions to setup a simulation environment. Run the simulator on current source code using ``--osdmsp`` option to build the OSD code into the simulator. For example, for a plane simulation:

::

    sim_vehicle.py -v ArduPlane --console --osdmsp

A graphical DJI style MSP OSD simulation in a separate window will be opened with the other simulation windows using a typical set of OSD panel parameters, located at libraries/AP_MSP/Tools/osdtest.parm . Then the OSD elements can be customized by their parameters using the  MSP OSD emulation program to visualize the OSD. 

.. note:: You could also use these parameters to initially setup the MSP OSD configuration for use with goggles, but you may have to change the ``SERIALx_PROTOCOL`` parameter to match the actual serial port that you will be using.

.. note:: The emulation does not support multiple screens or units other than metric


.. image:: ../../../images/msp_osd_python.png
   :target: ../_images/msp_osd_python.png

By changing the OSD panel items' parameters, a live update on their placement can be seen in this emulator.

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
This is required if the remote OSD system doesn not have an ArduPilot compatible fonts table. MWOSD already supports custom fonts and therefore does not require this hack while FatShark's SharkByte will support custom fonts in a future release.
Default behaviour is to use the ArduPilot fonts table.

Configuration
-------------

To enable MSP DisplayPort OSD, set the following parameters (using SERIAL port 2 as the port to attach to the Air unit using both TX and RX lines):

 - :ref:`OSD_TYPE<OSD_TYPE>` = 5
 - :ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>` = 42

Using Mission Planner to Configure the Layout
=============================================

Mission Planner(MP) has a tab in its CONFIG menu to configure the on-board OSD many autopilots integrate. This same configuration tab can be used to configure the OSD panels. In fact, you can do that while the SITL program and MSP OSD emulation window are active by connecting Mission Planner running on the same computer, or networked computer, to MAVProxy, using this command in MAVProxy:

::

    output add <ip address of box running Mission Planner>:14550


.. note:: if MP is running on the same PC, the ip address would be 127.0.0.1 (local host address)

Video
=====

.. youtube:: gT4R3E_7Z_0


.. common-osd:

===
OSD
===

Onboard OSD functionality was introduced with chibios port to F4/F7 boards. Mainly aiming at aio-boards with onboard MAX7456-type chips, it enables overlay of FC data using the onboard video system. Once enabled, it uses various panel items that can individually be set active and positioned on the screen using respective parameters. It allows to setup multiple screen layouts and switch between those using a spare RC channel.

.. note:: 

  Being an integral library of ArduPilot firmware, the OSD code provides the technical prerquisites to be run on non-aio-boards too, with the output being redirected to various backends other than a MAX-type chipset. This is still work in progress though.

Parameters
==========

To enable the OSD overlay on MAX7456-type chips, set parameter
::
   OSD_TYPE
to 1, reboot and reload paramaters. This exposes the whole OSD parameter group. 

To set a RC channel for screen switching, use parameter
::
   OSD_CHAN
Screens and screen switching
============================

For multiple screen layouts, each parameter's "OSD" part is trailed by a number, starting with "1"
::
   OSD1_PARAMETER
This number adresses the respective screen layout number, allowing to set individual items active on one screen but have them switched off on another screen. OSD settings default to 4 screens optionally available.

Set parameters
::
   OSDn_CHAN_MIN
and 
::
   OSDn_CHAN_MAX
to adjust RC channel pwm limits to use for switching to a respective screen.

There are different switch-method options to meet individual RC systems switch layout requirements. These can be set by parameter:
::
   OSD_SW_METHOD
The options are:

0 = switches to next screen if the set RC channel's value is changed

1 = directly selects a screen based on the set pwm limits for each respective screen

2 = toggles screens on a low to high transition of set RC channel. keeps toggling to next screen every 1s while channel value is kept high



Panel items
===========

Each OSD panel item uses a set of three variables to be set: 
::
   OSDn_ITEM_ENABLE

activates the respective item when set to 1.
::
   OSDn_ITEM_X
   
and 
::   
   OSDn_ITEM_Y
   
set the horizontal and vertical position of the item, starting with X = 0 and Y = 0 in the upper left corner of your screen. 

.. note::
   
   The typical MAXChip based OSD screen has a visible matrix of 30 horizontal x 13 vertical chars in NTSC standard, while PAL standard has 16 vertical chars. The OSD code enables auto-detection of NTSC vs. PAL to match input signal properties.

.. _other-mission-planner-features:

==============================
Other Mission Planner Features
==============================

Other features that aren't specific to the modes already described in
the menu:

-  The Mission Planner Console:  When you first open Mission Planner you
   will see a small window with a black background with a lot of text
   scrolling by.  When Mission Planner finally opens, the window may
   disappear. That window is called the console. You can keep it alive
   by checking a box at the bottom of the Mission Planner help screen.
-  Shortcut Keys:

   -  F12 - disconnect or connect from/to USB or Telemetry
   -  F5 - Refresh parameters
   -  F2 - change to Flight Data tab
   -  F3 - change to Flight Planner tab
   -  F4 - change to Configuration tab
   -  F5 - Refresh parameter list
   -  CTL-G Nmea Output
   -  CTL-W Wizard config interface

-  You can display video from your aircraft on the Mission Planner's HUD
   if you are digitizing it with the same laptop (with a video capture
   card or USB device). Just select the device in the
   Configuration/Planner screen, chose video format and press "Start" to
   begin recording. If you select the HUD checkbox, the video display
   will replace the artificial horizon in the HUD.
-  You can also record the video that you're displaying to an AVI file.
   Just right-click in the HUD and choose from the menu that will pop
   up.
-  You can send NMEA (GPS) output to the serial port. Press control-g
   and you will get a box asking for a comport and baud rate to output
   standard NMEA data at 2 Hz, to any serial port. One application would
   be to use com2com to emulate a serial port to use with third party
   applications.
-  You can have the Mission Planner load non-standard firmware (any .hex
   files that you've made or got elsewhere) to APM by pressing Control-C
   on the firmware loading screen.
-  Mission Planner can give you audio speech alerts for events that you
   choose, such as hitting waypoints or low battery. Just go to
   **Configuration \| Planner \| Speech \| Enable Speech**
-  Mission Planner can be scripted - see :ref:`Using Python Scripts in Mission Planner <using-python-scripts-in-mission-planner>`
   (planner wiki)

   
   
.. toctree::
    :maxdepth: 1

    Geotagging Images with Mission Planner <common-geotagging-images-with-mission-planner>
    Using Python Scripts in Mission Planner <using-python-scripts-in-mission-planner>
    Swarming <swarming>
    Advanced Tools <common-mp-tools>
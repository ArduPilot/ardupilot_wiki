.. _sitl-with-realflight:

==========================
Using SITL with RealFlight
==========================

.. youtube:: 5XqQ52n_U8M
    :width: 100%

`RealFlight <http://www.realflight.com/>`__ is a commercial flight simulator with a 3D view, and the ability to design and test custom vehicles.

This simulator only runs on Windows and requires RealFlight
version 8. Note that RealFlight-X does not work with ArduPilot.
The reason for the long installation process is to ensure you
have the opportunity to read (and agree) to the terms of use by
RealFlight/KnifeEdge to remove their liability from your
experiemental autopilot flights.

Enabling RealFlight Link Feature
================================

  - `Buy <http://www.realflight.com/where-to-buy.php>`__ and Install the regular version of RealFlight8. Note that you can now get RealFlight8 on steam, which is a very easy way to get it quickly.
  - update it to the latest version using the Technical Support button in the launcher (you need at least version 8.00.019)
  - Connect the RealFlight transmitter to your PC's USB port
  - Start RealFlight8 and in the launcher choose "Technical Support"
  - Click "Enter Tech Support Code"
  - Enter the code: LINK
  - A message box will appear with a new code in the form of "LINK-1234567"
  - enter that code on the `knifeedge signup page <https://www.knifeedge.com/cart/index.php?main_page=product_info&products_id=181>`__
  - Enter your response code in the same location
  - After reading and accepting the legal stuff, purchase the "FlightAxis Link" feature for $0 (Free)
  - RealFlight/KnifeEdge will contact you via email with an unlock code in the form of "LINKUNLOCK12345678"
  - Enter the unlock code into ReafFlight 8 at Launcher -> "Technical Support" -> "Enter Tech Support Code"

Configure RealFlight
====================

  - Start RealFlight (it should look exactly like regular RealFlight, there is no way to visually determine the difference)
  - Download the QuadcopterX from `ArduPilot/SITL_Models/RealFlight/Tridge/MultiRotors/QuadCopterX/Quadcopter X - direct throttles_AV.RFX <https://github.com/ArduPilot/SITL_Models/raw/master/RealFlight/Tridge/MultiRotors/QuadCopterX/Quadcopter%20X%20-%20direct%20throttles_AV.RFX>`__
  - Select Simulation, Import, RealFlight Archive (RFX, G3X) and select the file QuadcopterX downloaded above.  A message, "..was successfully imported" should be displayed
  - Select Aircraft, Select Aircraft, open "Custom Aircraft" section and select "Quadcopter X - direct throttle".  In the current state, the RC inputs come straight from the stick so it is not flyable.

  .. image:: ../images/realflight-select-aircraft.png
    :target: ../_images/realflight-select-aircraft.png

From within RealFlight, Reduce graphics options to improve performance:

  - Simulation, Settings, Graphics
  - Under "Quality" set all values to "No" or "Low" (i.e. set "Clouds" to "No", "Water Quality" to "Low", etc)
  - Under "Hardware" set "Resolution" to "800 x 600 Medium(16 bit) and select "Full Screen" mode
  - Under physics settings, change the option for "pause simulator when in background" to No
   
  .. image:: ../images/realflight-settings-graphics.png
    :target: ../_images/realflight-settings-graphics.png
   
Connecting to Mission Planner's SITL
------------------------------------

  - On Config/Tuning, Planner set the Layout drop-down to "Advanced"
  - On the top menu bar, select Simulation
  - From the "Model" drop-down, select "flightaxis" and push the Multirotor icon
  - on the Full Parameter List or Tree screens, on the right-side select realflight-quad and press load parameters

  .. image:: ../images/realflight-mp-sitl.jpg
    :target: ../_images/realflight-mp-sitl.jpg

On the real-flight controller press the red "reset" button to reset the vehicle's attitude and position and initialise the connection with SITL.
- the message "FlightAxis Controller Device has been activated." should appear and the motors should become quieter

If the vehicle's position is not reset, from within RealFlight:

  - Aircraft, Select Aircraft
  - Custom Aircraft, QuadcopterX - direct throttles
  - press OK
  - after the vehicles position is reset, press the transmitter's "Reset" button again

Connecting to SITL running on a separate (or Virtual) machine:
--------------------------------------------------------------

  - determine the IP address of the Windows machine running RealFlight by opening a console and entering "ipconfig".
    The result will likely be something like 192.168.x.x OR 127.0.0.1 if running sitl on a Windows machine using :ref:`cygwin <building-setup-windows-cygwin>` or :ref:`WSL <building-setup-windows10>`
  - on the separate machine where SITL will run, start SITL with "-f flightaxis:192.168.x.x" or if using a tradition helicopter, "-f heli-dual --model flightaxis:192.168.x.x".

     - cd ArduCopter
     - ../Tools/autotest/sim_vehicle -f flightaxis:192.168.x.x --map --console
  - back on RealFlight push the red "RESET" button on the transmitter
  - after about a minute, the vehicle should be visible on the SITL map
  - the performance of the connection can be checked by opening the "ArduCopter" window (on the machine running SITL), the "FPS" (Frames Per Second) count needs to be over 150 for the vehicle to fly well

Using ready-made models
-----------------------

As mentioned above, RealFlight allows designing your own custom vehicles including choosing the size, weight, appearance, and motor and control surface placement.

A number of custom models have been created by ArduPilot developers and stored in the `ArduPilot/SITL_Models repository <https://github.com/ArduPilot/SITL_Models>`__.
You should be able to :ref:`clone <git-clone>` this repo using ``git clone https://github.com/ArduPilot/SITL_Models.git`` and then load the models into RealFlight.
In the directory for each model there is a .parm file that can be loaded into SITL so that appropriate tunings parameters are set.

To import one of these models:

  - on RealFlight select Simulation >> Import >> RealFlight Archive (RX, G3X) and select the model you're interested in
  - select Aircraft >> Select Aircraft and select the model imported from the above step

  .. image:: ../images/realflight-import-model.png
    :width: 70%
    :target: ../_images/realflight-import-model.png

  - from within SITL type ``param load <filename>``  to load the parameter found in the same directory as the model, i.e. ``param load ../../SITL_Models/RealFlight/Tridge/QuadPlane/BigStickQuadPlane.parm`` to load the quadplane parameters.  In some cases you may need to restart SITL in order for some parameters to take effect.

  .. image:: ../images/realflight-import-parms.png
    :width: 70%
    :target: ../_images/realflight-import-parms.png

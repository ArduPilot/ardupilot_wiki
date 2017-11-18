.. _sitl-with-realflight:

==========================
Using SITL with RealFlight
==========================

.. youtube:: 5XqQ52n_U8M
    :width: 100%

`RealFlight <http://www.realflight.com/>`__ is a commercial flight simulator with a 3D view, and ability to design and test custom vehicles.

This simulator only runs on Windows and requires RealFlight
version 8. Note that RealFlight-X does not work with ArduPilot.

Installing modified RealFlight
==============================

  - `Buy <http://www.realflight.com/where-to-buy.php>`__ and Install the regular version of RealFlight8
  - Connect the RealFlight transmitter to your PC's USB port
  - Start RealFlight8 and in the launcher choose "Technical Support"
  - Click "Enter Tech Support Code"
  - Enter the code: LINK
  - a message box will appear with a new code
  - enter that code on the `knifeedge signup page <https://www.knifeedge.com/cart/index.php?main_page=product_info&products_id=181>`__
  - Enter your response code in the same location
  - You should receive a message indicating that Link functionality is now unlocked.
  - Download the "Quadcopter X - direct throttles_AV.RFX" file to your PC
  - Start RealFlight (it should look exactly like regular RealFlight, there is no way to visually determine the difference)
  - Select Simulation, Import, RealFlight Archive (RFX, G3X) and select the file downloaded above.  A message, "..was successfully imported" should be displayed
  - Select Aircraft, Select Aircraft, open "Custom Aircraft" section and select "Quadcopter X - direct throttle".  In the current state, the RC inputs come straight from the stick so it is not flyable.

  .. image:: ../images/realflight-select-aircraft.png
    :target: ../_images/realflight-select-aircraft.png
  
From within RealFlight, Reduce graphics options to improve performance:

   - Simulation, Settings, Graphics
   - Under "Quality" set all values to "No" or "Low" (i.e. set "Clouds" to "No", "Water Quality" to "Low", etc)
   - Under "Hardware" set "Resolution" to "800 x 600 Medium(16 bit) and select "Full Screen" mode
   
  .. image:: ../images/realflight-settings-graphics.png
    :target: ../_images/realflight-settings-graphics.png
   
From within Mission Planner:

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

To use SITL running on a separate (or Virtual Machine):

   - on the windows machine, determine its IP address by opening a console and entering "ipconfig".  The result will likely be something like 192.168.x.x.
   - on the separate machine, start SITL with "-f flightaxis:192.168.x.x"

       - cd ArduCopter
       - ../Tools/autotest/sim_vehicle -f flightaxis:192.168.x.x --map --console
   - back on RealFlight push the red "RESET" button on the transmitter
   - after about a minute, the vehicle should be visible on the SITL map
   - the performance of the connection can be checked by opening the "ArduCopter" window (on the machine running SITL), the "FPS" (Frames Per Second) count needs to be over 150 for the vehicle to fly well

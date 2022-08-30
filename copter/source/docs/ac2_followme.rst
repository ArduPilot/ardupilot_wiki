.. _ac2_followme:

============================
Follow Me Mode (GCS Enabled)
============================

Follow Me mode makes it possible for you to have your copter follow you
as you move, using a telemetry radio and a ground station.

.. note::

   Unlike other autopilot modes, this feature is implemented in
   the Ground Station. The ground station controls the movement by reading
   the vehicle position using MAVLink Telemetry and sending GUIDED mode
   instructions to move the vehicle appropriately. Currently this sort of
   functionality is supported by *Mission Planner* for Windows laptops, APM
   Planner for OS X laptops, and DroidPlanner for Android devices.

What you'll need
================

#. A Copter with telemetry
#. A laptop (or a phone/tablet with an integrated GPS available to the GCS app)
#. A GPS USB dongle `like this <https://www.amazon.com/ND-100S-GlobalSat-USB-GPS-Receiver/dp/B004K39QTY/>`__
   or Bluetooth GPS module `like this <https://www.amazon.com/GlobalSat-BT-368i-Bluetooth-GPS-Receiver/dp/B0018YZ836/>`__.

Instructions for Mission Planner
================================

#. Set one of your flight modes to "Loiter"
#. Set up your Copter and establish a MAVLink connection
   over wireless telemetry
#. Ensure that your GPS USB dongle or Bluetooth device is plugged into
   to your laptop and showing up as a serial port. Using the software
   that came with the module, make sure that it's working and that you
   have GPS lock.
#. Take off, and once in the air switch to Loiter. (Make sure there's sufficient altitude
   to ensure the drone does not injure you).
#. In Mission Planner Flight Data screen try right-clicking on a
   nearby spot and select "Fly to Here". If this works, you're ready to
   try Follow Me mode.
#. In Mission Planner, enter Control-F to open the
   following window. Click on "Follow Me"

.. image:: ../images/followme1.jpg
    :target: ../_images/followme1.jpg

-  This will bring up this window. Select the serial port that's assigned to your GPS device and the baud rate it uses.

.. image:: ../images/followme2.png
    :target: ../_images/followme2.png

-  Once you click "Connect", Mission Planner will read GPS data
   from your device and send it to your Copter as "fly to here" commands
   every two seconds.

-  Now pick up your laptop and start walking around.
-  The Copter should follow you!

   -  Make sure to set sufficient altitude to prevent any injuries.
   -  If proper altitude is set, you can make it follow you and attempt
      to out run it!
      
.. warning::
   The barometer is used in the altitude calculation meaning that it can
   drift over time and impact rising terrain. Watch for obstacles and 
   elevation changes. The altitude maintaining behavior is similar to loiter mode.

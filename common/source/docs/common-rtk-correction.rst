.. _common-rtk-correction:

===================================
RTK GPS Correction (Fixed Baseline)
===================================

It is possible to increase the normal position reporting accuracy of the GPS using RTK (Real Time Kinematics). Normal GPS accuracy is 3-5 meters using the Ublox M8N generation of GPS. If your global region has SBAS (Satelite Based Augmentation Service) with geo-synchronous satellites reporting the general propagation conditions and corrections to the GPS, then accuracy can improve to the 1 meter range. But centimeter accuracy ranges can be obtained by using Real Time Kinematics correctional data with the newer F9P generation of GPSes.

.. youtube:: wmzCf1th1yE

Propagation and timing corrections (RTCM data) for each satellite can be fed to the vehicle's RTK GPS in several ways:

#. from a local RTK base station connected to the GCS, which is also connected to the vehicle (via MAVLink)
#. by forwarding RTCM correction data from an NTRIP server via the internet-connected GCS, which is also connected to the vehicle (via MAVLink)
#. wirelessly from a local RTK base station directly to the vehicle GPS's secondary UART port



Method 1
========

.. image:: ../../../images/rtk.jpg


Both Mission Planner and QGC allow the connection of an external RTK Base GPS via USB. The correction data is passed from the Base unit through the GCS program to the vehicle's MAVLink connection to the GCS, providing the vehicle's RTK GPS with correction data to enable its position reporting to become more accurate.

Key to the success of this method requires that the RTK Base station's position be precisely known or determined. Usually, this requires that the GCS command the RTK Base GPS to "survey-in" itself. The GPS takes many measurements determining its mean location, applying interim corrections, and continues to refine its location deviations until the measurements are within a set threshold of error (usually a few meters) for a given time period (usually 60 seconds). The GPS then uses that as its location and start outputting the correction data to the GCS for forwarding to the vehicle's GPS.

It is possible to program the RTK Base's exact location directly into it if it is known using the UBlox programming tool or Mission Planner if that location has been previously "surveyed-in".

Setup
-----

Vehicle: normal setup for serial GPS (:ref:`GPS1_TYPE<GPS1_TYPE>` = 1) or DroneCAN GPS (:ref:`GPS1_TYPE<GPS1_TYPE>` = 9)

Attach RTK GPS via USB to PC running Mission Planner or QGC.

You first must "survey-in" the GPS, in Mission Planner.

.. image:: ../../../images/mp_rtk_connect.jpg

After the GPS is attached to the PC, start Mission Planner, enter the COM port to which it is attached and press "Connect". The "Link Status" should start showing data being input and the satellite constellations locked will be shown with green indicators.

.. image:: ../../../images/mp_rtk_valid.jpg

Then press the "RESTART" button (#1 in the picture) to start the "survey-in" process using the SurveyIn Acc(m) and Time(s). Ublox recommends a SurveyIn ACC of 5m or less. 2m is commonly used. In #2. in the picture the survey status progress can be monitored until a "Position is Valid" is obtained.

Once the position is obtained, you can save it in a list for future use, if the Base is placed at that exact location in the future, to avoid another survey procedure. To use an existing position, first ensure that the GPS attached to the Mission Planner PC is indeed in that exact location, then press the "USE" button for that location in the list. RTK updates to the vehicle will begin immediately.

Once RTK data is being processed by the vehicle's GPS, its status will change to "RTK float" and then to "RTK fixed" in the GCS HUD indicating that it is ready for use. "RTK float" means that it is using correction data, but has not moved to the highest precision mode yet.

Method 2
========

.. image:: ../../../images/rtk_b.jpg

There are internet-based services that can provide correction data from bases linked to that service. They provide the correction data over the internet via the NTRIP protocol. Mission Planner and QGC can connect to these servers and forward the data via MAVLink connection to the vehicle.

If the service has a Base station providing real-time correction data within 100 Km or so of the vehicle's location, then it will usually be satisfactory. For every 10 Km away from the vehicle that the source is located, the inaccuracy limits will increase ~ 1-1.5cm. Some services provide interpolated data for any location within their network of stations and this can sometimes be successfully used if a station is further than 100km away from the vehicle location.

Some of these services are free and some are fee-based. 

Setup
-----

First, be sure that the vehicle is connected to the GCS and has a *GPS 3D lock*. Then on the GCS:

.. image:: ../../../images/NTRIP.jpg

#. Select "NTRIP" from the dropdown box and press "Connect". A dialog box (#2.) will appear to enter the login information for an NTRIP server.
#. The login is usually of the form:

.. code:: bash

    http://username:password@host:port/mount


where the "host" is the NTRIP server URL, the port as designated by the service (usually 2101 for RTCM3.1 data streams), and the mount point will be the service's designation of the station that you want to use (the closest one to your location), usually something like "P807_RTCM" or "Evermount234", whatever the service names them.

If the connection occurs, data will start appearing in the Link Status box at the top and be sent to the connected vehicle's RTK GPS.
 
Method 3
========

.. image:: ../../../images/rtk_a.jpg

It is possible to program the GPS used as a Fixed Base, to automatically begin to *survey-in* itself upon power up and start sending RTCM correction data out its UART2 port once completed. By attaching a transparent RF modem, either radio-based or WIFI, you can directly transmit the correction data to the vehicle's UART2 port.

This requires configuring the Base RTK GPS with the UBlox configuration tool. See `Sparkfun's tutorial <https://learn.sparkfun.com/tutorials/setting-up-a-rover-base-rtk-system>`__ for more information.

Setup
-----

See `blog post on setup <https://discuss.ardupilot.org/t/simple-portable-rtk-base-station>`__

.. note:: some RTK GPSes are already configured for dedicated BASE operation with automatic "survey-in" being done every power up. The base model of the `Blicube GRTK <https://wiki.blicube.com/grtk/>`__ system is an example.

Acknowledgment
==============

Thanks to `Qiotek <https://www.qio-tek.com>`__ for providing the autopilot and GPSes used for this article.

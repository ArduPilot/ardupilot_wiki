.. _common-positioning-landing-page:

==========================
GPS/Compass (landing page)
==========================

Copter/Plane/Rover support integration with GPS, Compass and other
positioning technologies:

.. figure:: ../../../images/gps-landing-page.jpg
   :target: ../_images/gps-landing-page.jpg


GPS/Compass
===========
.. toctree::
    :maxdepth: 1
    
    Avionics Anonymous GNSS + Compass<common-avanon-gnss-mag>
    Avionics Anonymous Compass<common-avanon-mag>
    Beitain BN-220 GPS / BN-880 GPS + Compass Module <common-beitian-gps>
    CUAV Neo v2 Pro DroneCAN GPS <common-cuav-neo-v2-pro>
    CUAV Neo 3 Pro DroneCAN GPS <common-cuav-neo-3-pro>
    CUAV Neo 3（M9N) GPS <common-cuav-neo-3>
    Hex Here 2 DroneCAN GPS/Gyro/IMU/Baro <http://www.proficnc.com/all-products/152-gps-module.html>
    Holybro Micro M8N GPS Module <https://shop.holybro.com/micro-m8n-gps_p1009.html> 
    Holybro Nano Ublox M8 5883 GPS Module <https://shop.holybro.com/nano-ublox-m8-5883-gps-module_p1236.html> 
    Holybro M8N (Pixhawk 4) GPS Module <https://shop.holybro.com/pixhawk-4-gps-module_p1094.html>
    Holybro M9N GPS Module <http://www.holybro.com/product/holybro-m9n-gps/>
    Matek M8Q-CAN/DroneCAN GPS+Compass+Baro+Airspeed I2C port <common-matek-m8q>
    mRo GPS, GPS+Compass,RTK, and DroneCAN modules <https://store.mrobotics.io/category-s/109.htm>
    mRo Locaton One DroneCAN GPS/Compass <common-mrobotics-location-one-gps>
    Qiotek DroneCAN RM3100 Compass <http://www.qio-tek.com/RM3100_CAN>
    Zubax GNSS 2: GNSS + Compass + Barometer <common-zubax-gnss-positioning-module-gps-compass-and-barometer>

RTK GPS
=======

These GPS can incorporate real time kinematic data, either internally generated or externally provided, to improve the precision of the position estimate from the normal GPS meter range down to the cenitmeter range. This increased precision also allows for "Moving Baseline" yaw estimates using two devices on a vehicle with certain RTK GPSes (F9P based). See :ref:`common-gps-for-yaw`.

.. toctree::
    :maxdepth: 1
    
    ArduSimple RTK GPS <common-gps-ardusimple>
    Blicube RTK GPS <https://github.com/ThomasVon2021/ardupilot_rtk>
    CUAV C-RTK 9P RTK Receiver <common-cuav-c-rtk-9p-gps>
    CUAV C-RTK2 PPK and RTK receiver<common-cuav-c-rtk2>
    Emlid Reach RTK Receiver <common-reach-rtk-receiver>
    Hex Here+ RTK Receiver <common-here-plus-gps>
    Holybro RTK M8P Family <common-holybro-rtk-m8p>
    Holybro RTK F9P Family <common-holybro-rtk-f9p>
    Qiotek DroneCAN RTK-F9P GPS <https://www.qio-tek.com/RTK_F9P>
    Swift Navigation's Piksi Multi RTK GPS Receiver <common-piksi-multi-rtk-receiver>
    Septentrio AsteRx-mUAS RTK GPS <common-gps-septentrio>
    Synerx MDU-2000 RTK + LTE GPS <common-synerex-mdu-2000>
    Trimble BD930 RTK GPS <common-gps-trimble>

.. note:: It is important that a GPS be connected to the first SERIALx port that has its ``SERIALx_PROTOCOL`` parameter set to "5" (GPS) since it will stop searching for GPS during bootup if not found on the first port configured for GPS protocol.

.. note:: If the GPS is only capable of 115.2Kbaud serial data rates, set :ref:`GPS_DRV_OPTIONS<GPS_DRV_OPTIONS>` bit 2 to enable its operation.

When using two GPS units there are a number of switching options that can be selected with :ref:`GPS_AUTO_SWITCH <GPS_AUTO_SWITCH>`.

- 0: Use Primary
    Always use the primary GPS, this can be either the first or second as set with :ref:`GPS_PRIMARY <GPS_PRIMARY>`

- 1: Use best
    Automatically select the best GPS, this is done based on GPS fix status (2D / 3D / rtk) if both GPSs have the same fix status the one with the larger number of satellites is used.

- 2: Blend
    Blend is best suited for use with two identical GPS units, see :ref:`GPS blending<common-gps-blending>`

- 4: Use primary if 3D fix or better
    Use primary GPS as set with :ref:`GPS_PRIMARY <GPS_PRIMARY>` if it has a 3D fix or better. This should be used when two dissimilar GPS units are used, one high quality primary unit, and a secondary less accurate unit. In this case the 
    high quality GPS will often have a better quality fix even if it has fewer satellites. Using 'Use best' in this case would result in using the poorer quality GPS and result in more switching between GPS units. Unlike 'Use Primary' this option 
    still allows falling back to the secondary GPS if 3D fix is lost on the primary.

An additional variation with GPS switching is :ref:`EKF3 affinity and lane switching <common-ek3-affinity-lane-switching>`. An EKF lane can be setup to use either GPS and the whole EKF lane is then switched based on its health. If the :ref:`GPS_PRIMARY <GPS_PRIMARY>` is used for a lane, and :ref:`GPS_AUTO_SWITCH <GPS_AUTO_SWITCH>` is enabled, then the lane will use GPS info as determined by the :ref:`GPS_AUTO_SWITCH <GPS_AUTO_SWITCH>` setting.

Advanced Uses
=============

.. toctree::
    :maxdepth: 1

    GPS Blending (aka Dual GPS) <common-gps-blending>
    GPS for Yaw (aka Moving Baseline) <common-gps-for-yaw>
    GPS – How it Works <common-gps-how-it-works>
    U-Blox F9P Firmware Update Procedure <common-gps-ublox-firmware-update>

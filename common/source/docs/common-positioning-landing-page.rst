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
    CUAV Neo 3X(Water proof) GPS <common-cuav-neo-3x>
    CubePilot Here 2 DroneCAN GPS/Gyro/IMU/Baro <https://docs.cubepilot.org/user-guides/here-2/updating-here-2-firmware>
    Holybro DroneCAN M8/M9 GPS <https://shop.holybro.com/dronecan-m8n-m9n-gps_p1335.html>    
    Holybro Micro M8N GPS Module <https://shop.holybro.com/micro-m8n-gps_p1009.html> 
    Holybro Nano Ublox M8 5883 GPS Module <https://shop.holybro.com/nano-ublox-m8-5883-gps-module_p1236.html> 
    Holybro M8N (Pixhawk 4) GPS Module <https://holybro.com/collections/gps-rtk-systems/products/m8n-gps>
    Holybro M9N GPS Module <https://holybro.com/collections/gps-rtk-systems/products/m9n-gps>
    Holybro DroneCAN RM3100 Professional Grade Compass <https://holybro.com/products/dronecan-rm3100-compass>
    Matek DroneCAN AP_Periph GNSS M10-L4-3100 <http://www.mateksys.com/?portfolio=m10-l4-3100>
    Matek SAM-M8Q <http://www.mateksys.com/?portfolio=sam-m8q>
    Matek M8Q-CAN/DroneCAN GPS+Compass+Baro+Airspeed I2C port <common-matek-m8q>
    mRo GPS, GPS+Compass,RTK, and DroneCAN modules <https://store.mrobotics.io/category-s/109.htm>
    mRo Locaton One DroneCAN GPS/Compass <common-mrobotics-location-one-gps>
    Qiotek DroneCAN GNSS M10 QMC5883  <https://www.qio-tek.com/index.php/product/qiotek-m10s-gnss-module/>
    Qiotek GNSS M10 QMC5883 <https://www.qio-tek.com/index.php/product/qiotek-m10s-gnss-module/>
    Qiotek DroneCAN RM3100 Compass <https://www.qio-tek.com/index.php/product/qiotek-rm3100-dronecan-module/>
    Zubax GNSS 2: GNSS + Compass + Barometer <common-zubax-gnss-positioning-module-gps-compass-and-barometer>

RTK GPS
=======

These GPS can incorporate real time kinematic data, either internally generated or externally provided, to improve the precision of the position estimate from the normal GPS meter range down to the centimeter range (see :ref:`common-rtk-correction`). This increased precision also allows for "Moving Baseline" yaw estimates using two devices on a vehicle with certain RTK GPSes (F9P based) or dedicated Moving Baseline GPSes. See :ref:`common-gps-for-yaw`.

.. toctree::
    :maxdepth: 1
    
    ArduSimple RTK GPS <common-gps-ardusimple>
    Ark RTK GPS <https://arkelectron.com/product/ark-rtk-gps/>
    Blicube RTK GPS (Single Unit Moving Baseline NMEA) <https://wiki.blicube.com/grtk/>
    CUAV C-RTK 9P RTK Receiver <common-cuav-c-rtk-9p-gps>
    CUAV C-RTK 9Ps RTK Receiver <https://store.cuav.net/shop/c-rtk-9ps/>
    CUAV C-RTK2 PPK and RTK receiver<common-cuav-c-rtk2>
    CUAV C-RTK2 HP  Heading and RTK receiver<common-cuav-c-rtk2-hp>
    Emlid Reach RTK Receiver <common-reach-rtk-receiver>
    CubePilot HERE 3/3+ DroneCAN RTK GPS/IMU/Compass <https://docs.cubepilot.org/user-guides/here-3/here-3-manual>
    CubePilot HERE 4 DroneCAN RTK GPS/IMU/Compass (see note below) <https://docs.cubepilot.org/user-guides/here-4/here-4-manual>
    CubePilot Here+ RTK Base/Rover Receiver <common-here-plus-gps>
    CubePilot HEREPRO DroneCAN F9P RTK GPS/Compass <https://docs.cubepilot.org/user-guides/herepro/herepro-manual>
    Freefly RTK GPS Ground Station <https://store.freeflysystems.com/products/rtk-gps-ground-station>
    Foxteck AEROFOX F9P-RTK <https://www.foxtechfpv.com/aerofox-f9p-rtk-unit.html>
    Hitec PositionPro GNSS <https://www.hiteccs.com/drones/peripheral-details/positionpro-gnss-receiver/>
    Holybro DroneCAN H-RTK F9P Rover <https://holybro.com/collections/gps-rtk-systems/products/dronecan-h-rtk-f9p-rover>
    Holybro DroneCAN H-RTK F9P Helical <https://holybro.com/collections/gps-rtk-systems/products/dronecan-h-rtk-f9p-helical>
    Holybro RTK M8P Family <common-holybro-rtk-m8p>
    Holybro RTK F9P Family <common-holybro-rtk-f9p>
    Holybro RTK F9P Ultralight <https://holybro.com/products/h-rtk-f9p-ultralight>
    Holybro RTK Unicore UM982 GPS <https://holybro.com/products/h-rtk-unicore-um982>
    LOCOSYS HAWK R1 RTK GNSS / R2 RTK GNSS + Compass<common-gps-locosys>
    Qiotek DroneCAN RTK-F9P GPS <https://www.qio-tek.com/index.php/product/qiotek-zed-f9p-rtk-and-compass-dronecan-module>
    Swift Navigation's Piksi Multi RTK GPS Receiver <common-piksi-multi-rtk-receiver>
    Septentrio AsteRx-m UAS RTK GPS <common-gps-septentrio>
    Synerx MDU-2000 RTK + LTE GPS <common-synerex-mdu-2000>
    Trimble BD930 RTK GNSS <common-gps-trimble-bd930>
    Trimble PX-1 RTX GNSS+INS <common-gps-trimble-px1>

.. note:: a version of firmware for the CubePilot HERE 4 DroneCAN RTK GPS is available `here <https://firmware.ardupilot.org/AP_Periph/latest/Here4AP/>`__ that includes evolving ArduPilot improvements.

.. warning:: It is important that a GPS be connected to the first SERIALx port that has its ``SERIALx_PROTOCOL`` parameter set to "5" (GPS) since it will stop searching for GPS during bootup if not found on the first port configured for GPS protocol.


Moving Baseline (GPS for Yaw) Capable
=====================================

    Ark RTK GPS <https://arkelectron.com/product/ark-rtk-gps/>    Blicube RTK GPS (Single Unit Moving Baseline) <https://wiki.blicube.com/grtk/>
    CUAV C-RTK 9P RTK Receiver <common-cuav-c-rtk-9p-gps>
    CUAV C-RTK 9Ps RTK Receiver <https://store.cuav.net/shop/c-rtk-9ps/>
    CUAV C-RTK2 PPK and RTK receiver<common-cuav-c-rtk2>
    CUAV C-RTK2 HP  Heading and RTK receiver<common-cuav-c-rtk2-hp>
    CubePilot HERE 4 DroneCAN RTK GPS/IMU/Compass <https://docs.cubepilot.org/user-guides/here-4/here-4-manual>
    CubePilot HEREPRO DroneCAN RTK GPS/Compass <https://docs.cubepilot.org/user-guides/herepro/herepro-manual>
    Foxteck AEROFOX F9P-RTK <https://www.foxtechfpv.com/aerofox-f9p-rtk-unit.html>
    Freefly RTK GPS Ground Station <https://store.freeflysystems.com/products/rtk-gps-ground-station>
    Holybro DroneCAN H-RTK F9P Rover <https://holybro.com/collections/gps-rtk-systems/products/dronecan-h-rtk-f9p-rover>
    Holybro DroneCAN H-RTK F9P Helical <https://holybro.com/collections/gps-rtk-systems/products/dronecan-h-rtk-f9p-helical>
    Holybro RTK F9P Family <common-holybro-rtk-f9p>
    Holybro RTK F9P Ultralight <https://holybro.com/products/h-rtk-f9p-ultralight>
    Holybro RTK Unicore UM982 GPS <https://holybro.com/products/h-rtk-unicore-um982>
    Qiotek DroneCAN RTK-F9P GPS <https://www.qio-tek.com/index.php/product/qiotek-zed-f9p-rtk-and-compass-dronecan-module>
    Synerx MDU-2000 RTK + LTE GPS <common-synerex-mdu-2000>

GPS Driver Options
==================

Several GPS operating options are provided by the :ref:`GPS_DRV_OPTIONS<GPS_DRV_OPTIONS>` parameter. This parameter is a bit mask and allows multiple option selections at the same time:

- bit 0: if set, will send RTK correction data from the first GPS directly to the second via the second's UART port for :ref:`common-gps-for-yaw`, instead via the autopilot.
- bit 1: if set, enable SBF moving baseline yaw using custom base and GPS_MB1 offsets.
- bit 2: if set, use 115.2Kbaud for max serial data rate for those GPSes not capable of higher rates.
- bit 3: if set, routes RTK data between two CAN GPSes via CAN instead of via the autopilot.
- bit 4: if set, GPS reports altitude in ellipsoid height instead of height AMSL.

GPS Auto Switch
===============

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

    Fixed Baseline RTK GPS Correction <common-rtk-correction>
    GPS Blending (aka Dual GPS) <common-gps-blending>
    GPS for Yaw (aka Moving Baseline) <common-gps-for-yaw>
    GPS – How it Works <common-gps-how-it-works>
    U-Blox F9P Firmware Update Procedure <common-gps-ublox-firmware-update>

.. _common-gremsy-pixyu-gimbal:

====================================================
Gremsy Mio, Pixy, S1, T3, T7, and ZIO 3-Axis Gimbals
====================================================

Gremsy `Mio <https://gremsy.com/products/mio>`__, `Pixy F <https://gremsy.com/products/pixy-f>`__, `Pixy U <https://gremsy.com/products/pixy-u>`__, `Pixy WP <https://gremsy.com/products/pixy-wp>`__,  `S1 <https://gremsy.com/products/gremsy-s1v3>`__, `T3 <https://gremsy.com/products/gremsy-t3v3>`__, `T7 <https://gremsy.com/products/gremsy-t7>`__ and `ZIO <https://gremsy.com/zio>`__ 3-axis gimbals can communicate with the autopilot using the MAVLink protocol and are compatible with a range of cameras for real-time video or mapping purposes.

.. image:: ../../../images/gremsy-pixyu-gimbal.png
    :target: https://gremsy.com/products/pixy-u

.. warning::

   MIO gimbals are unable to communication with ArduPilot.  We hope for a fix with Gremsy software version 7.7.3.

   T3V3 gimbals must be running 7.7.3-Preview (or higher).

   PixyU gimbals must be running 7.7.1 or 7.7.2.  7.7.3 does not work correctly.  We hope for a fix with Gremsy version 7.7.4 (or higher).

   PixyWE gimbals are not supported.

Where to Buy
============

These gimbals can be purchased from the `Gremsy store <https://gremsy.com/online-store>`__

Connecting to the Autopilot (4.3 or higher)
===========================================

If using ArduPilot 4.3 (or higher) please follow these setup instructions

.. image:: ../../../images/gremsy-pixyu-autopilot.png
    :target: ../_images/gremsy-pixyu-autopilot.png
    :width: 450px

Connect the gimbals's COM2 port to one of the autopilot's Serial/Telemetry ports like Telem2 as shown above.

Connect to the autopilot with a ground station and set the following parameters, if using the first mount:

- :ref:`MNT1_TYPE <MNT1_TYPE>` to "6" for "Gremsy" and reboot the autopilot
- If using a ZIO with ArduPilot-4.5.0 (or higher) set :ref:`CAM1_TYPE <CAM1_TYPE>` to "6" (MAVLinkCamV2) and reboot the autopilot
- :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` to "115" for 115200 bps.  "SERIAL2" can be replaced with another serial port (i.e. SERIAL1) depending upon the physical connection
- :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` to 2 for "MAVLink2"
- :ref:`SERIAL2_OPTIONS <SERIAL2_OPTIONS>` to 1024 for "Don't forward mavlink to/from"

See the "Control with an RC transmitter" section of :ref:`this page <common-mount-targeting>` for details on parameter changes required to control the gimbal through an RC Transmitter (aka "RC Targeting")

When the autopilot has successfully connected to the gimbal, "Mount: GREMSY PixyU fw:7.7.1.0" (or similar) will be sent to the ground station.  Looking for this message may be useful in determining if the autopilot and gimbal are communicating successfully.

Configuring the Gimbal
----------------------

- Download and install `gTune Desktop <https://github.com/Gremsy/gTuneDesktop/releases>`__
- Connect the gimbal to your Desktop PC using a USB cable and power on the gimbal
- Use gTune Desktop to check the gimbal firmware version

  - Open gTune Desktop application, "Found your device" should be displayed
  - Select "CONNECT"
  - Select "INFO" and confirm the gimbal is running Firmware "7.7.1" or higher

  .. image:: ../../../images/gremsy-firmware-version-check.png
      :target: ../_images/gremsy-firmware-version-check.png
      :width: 450px

  - If the gimbal firmware is older than 7.7.1 download the latest .hex for `Mio <https://github.com/Gremsy/Mio-Firmware/releases>`__,  `Pixy F <https://github.com/Gremsy/PixyF-Firmware/releases>`__, `Pixy U <https://github.com/Gremsy/PixyU-Firmware/releases>`__,  `Pixy WP <https://github.com/Gremsy/PixyW-Firmware/releases>`__,  `S1 <https://github.com/Gremsy/S1V3-Firmware/releases>`__, `T3 <https://github.com/Gremsy/T3V3-Firmware/releases>`__, `T7 <https://github.com/Gremsy/T7-Firmware/releases>`__ or `ZIO <https://github.com/Gremsy/Zio-Firmware/releases>`__
  - Select "UPGRADE", "BROWSE" and select the file downloaded above
  - Press the other "UPGRADE" button and the upgrade should complete within 30 seconds

  .. image:: ../../../images/gremsy-settings-upgrade.png
      :target: ../_images/gremsy-settings-upgrade.png
      :width: 450px

- Use gTune Desktop to configure the gimbal

  - Select "SETTINGS", "CONTROLS" and ensure "SYNC" is selected so the gimbal communicates with the autopilot using MAVLink

  .. image:: ../../../images/gremsy-settings-sync.png
      :target: ../_images/gremsy-settings-sync.png
      :width: 450px

  - Select "Settings", "REDUCE DIRFT by DRONE"

  .. image:: ../../../images/gremsy-settings-reduce-drift-by-drone.png
      :target: ../_images/gremsy-settings-reduce-drift-by-drone.png
      :width: 450px

- Select the "STIFFNESS" tab and adjust the Tilt, Roll, and Pan gains so that the gimbal holds the camera in position without shaking

Control and Testing
-------------------

See :ref:`Gimbal / Mount Controls <common-mount-targeting>` for details on how to control the gimbal using RC, GCS or Auto mode mission commands

Connecting Two Gimbals
----------------------

If two Gremsy gimbals are used, each gimbal can be directly connected to one of the autopilot's serial ports or they can can be daisy-chained together with only one connected to one of the autopilot's serial ports.

- Use the gTune desktop application set the second gimbal's MAVLink component id to "GIMBAL_COMPID2"

  .. image:: ../../../images/gremsy-settings-compid.png
      :target: ../_images/gremsy-settings-compid.png
      :width: 450px

- Connect a GCS to your autopilot and set

    - :ref:`MNT2_TYPE <MNT2_TYPE>` to "6" for "Gremsy" and reboot the autopilot
    - set the appropriate SERIALx_BAUD, SERIALx_PROTOCOL and SERIALx_OPTIONS parameters as described above

Connecting to the Autopilot (4.2 or earlier)
============================================

.. image:: ../../../images/gremsy-pixyu-autopilot.png
    :target: ../_images/gremsy-pixyu-autopilot.png
    :width: 450px

We recommend connecting the Gimbals's COM2 port to one of the autopilot's Serial/Telemetry ports like Telem2 as shown above.

Connect with a ground station and set the following parameters, if using the first mount:

- :ref:`MNT1_TYPE <MNT1_TYPE>` to "4" for "SToRM32 MavLink" and reboot the autopilot
- :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` to "115" for 115200 bps.  "SERIAL2" can be replaced with another serial port (i.e. SERIAL1) depending upon the physical connection
- :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` to 2 for "MAVLink2"
- :ref:`SR2_EXTRA1 <SR2_EXTRA1>` to 10
- :ref:`SR2_POSITION <SR2_POSITION>` to 10

The gimbal's maximum angles can be set using these parameters (shown for the first mount):

- :ref:`MNT1_ROLL_MIN <MNT1_ROLL_MIN>` to -30 to allow leaning left up to 30deg
- :ref:`MNT1_ROLL_MAX <MNT1_ROLL_MAX>` to 30 to allow leaning right up to 30deg
- :ref:`MNT1_PITCH_MIN <MNT1_PITCH_MIN>` to -90 to allow pointing 90deg down
- :ref:`MNT1_PITCH_MAX <MNT1_PITCH_MAX>` to 30 to allow pointing 30deg up
- :ref:`MNT1_YAW_MIN <MNT1_YAW_MIN>` to -180 to allow turning around to the left
- :ref:`MNT1_YAW_MAX <MNT1_YAW_MAX>` to 180 to allow turning around to the right

To control the gimbal's lean angles from a transmitter set the RC controls for roll, pitch, or yaw using ``RCx_OPTION`` 212 (Mount1 Roll), 213 (Mount1 Pitch), 214 (Mount1 Yaw) for the first mount, or 215-217 for the second mount, respectively.

Gremsy's instructions can be found below:

- `How to setup Gremsy gimbal with Pixhawk Cube <https://support.gremsy.com/support/solutions/articles/36000189926-how-to-setup-gremsy-gimbal-with-pixhawk-cube>`__
- `Control Gremsy Gimbal with Herelink & Cube <https://support.gremsy.com/support/solutions/articles/36000222529-control-gremsy-gimbal-with-herelink-cube-pilot>`__

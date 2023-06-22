.. _common-xacti-gimbal:

[copywiki destination="plane,copter,rover"]

=============
Xacti Gimbals
=============

The `Xacti Camera Gimbals <https://xacti-co.com/service/drone_camera/>`__ are 3-axis camera gimbals which communicate with ArduPilot using the DroneCAN protocol.

.. image:: ../../../images/xacti-gimbal.png

.. note::

    Support for these gimbals is available in ArduPilot 4.5.0 (and higher)

Some images courtesy of xacti-co.com

Where to Buy
------------

These gimbals can be purchased directly from `Xacti <https://xacti-co.com/service/drone_camera/>`__

Connecting to the Autopilot
---------------------------

.. image:: ../../../images/xacti-gimbal-autopilot.png
    :target: ../_images/xacti-gimbal-autopilot.png
    :width: 450px

.. image:: ../../../images/xacti-gimbal-pinout.png
    :target: ../_images/xacti-gimbal-pinout.png
    :width: 450px

The top image shows how to connect the gimbal's CANL, CANH and GND pins to one of the autopilot's CAN ports when using the gimbal mounting plate option.  The bottom image shows the camera's pinout which may be used with a custom designed mount.

Connect to the autopilot with a ground station and set the following parameters and then reboot the autopilot.  The params below assume the autopilot's CAN1 port is used,

- :ref:`CAN_D1_PROTOCOL <CAN_D1_PROTOCOL>` to 1 (DroneCAN)
- :ref:`CAN_P1_DRIVER <CAN_P1_DRIVER>` to 1 (First driver)
- :ref:`MNT1_TYPE <MNT1_TYPE>` to 10 (Xacti) and reboot the autopilot
- :ref:`MNT1_PITCH_MIN <MNT1_PITCH_MIN>` to -90
- :ref:`MNT1_PITCH_MAX <MNT1_PITCH_MAX>` to 25
- :ref:`MNT1_YAW_MIN <MNT1_YAW_MIN>` to -90
- :ref:`MNT1_YAW_MAX <MNT1_YAW_MAX>` to 90
- :ref:`CAM1_TYPE <CAM1_TYPE>` to 4 (Mount)
- :ref:`MNT1_RC_RATE <MNT1_RC_RATE>` to 30 (deg/s) to control speed of gimbal when using RC targetting
- :ref:`RC6_OPTION <RC6_OPTION>` = 213 ("Mount Pitch") to control the gimbal's pitch angle with RC channel 6
- :ref:`RC7_OPTION <RC7_OPTION>` = 214 ("Mount Yaw") to control the gimbal's yaw angle with RC channel 7

- Optionally these auxiliary functions are also available

  - :ref:`RC9_OPTION <RC9_OPTION>` = 166 ("Camera Record Video") to start/stop recording of video
  - :ref:`RC9_OPTION <RC9_OPTION>` = 168 ("Camera Manual Focus") to adjust focus in and out
  - :ref:`RC9_OPTION <RC9_OPTION>` = 169 ("Camera Auto Focus") to trigger auto focus

Control and Testing
-------------------

See :ref:`Gimbal / Mount Controls <common-mount-targeting>` for details on how to control the gimbal using RC, GCS or Auto mode mission commands

Videos
------

..  youtube:: jZszQ4OmfVQ
    :width: 100%

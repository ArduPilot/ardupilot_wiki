.. _common-simplebgc-gimbal:

=====================================
SimpleBGC Gimbal (aka AlexMos gimbal)
=====================================

The SimpleBGC is a popular brushless gimbal controller that can communicate with ArduPilot (Copter, Plane and Rover) using a custom serial protocol.
More details on the capabilities of this gimbal can be found at `basecamelectronics.com <https://www.basecamelectronics.com/>`__

.. note::

   Support for this gimbal is included in Copter 3.4 (and higher). 

Where to Buy
============

The SimpleBGC controller and accompanying 2-axis and 3-axis gimbals can be purchased from `basecamelectronics.com <https://www.basecamelectronics.com/>`__ and many other retailers.

Connecting the gimbal to the Flight Controller
==============================================

.. image:: ../../../images/simplebgc-gimbal-pixhawk.png
    :target: ../_images/simplebgc-gimbal-pixhawk.png

Although the SimpleBGC can be connected using PWM (similar to the Tarot gimbal) we recommend using the serial interface connected to one of the flight controller's Serial/Telemetry ports like Telem2 as shown above.

However, if you connect the gimbal as shown in the above diagram it is NOT possible to establish a connection to the GUI on a PC via USB and have it connected simultaneously to the flight controller since the UART1 ("serial" in the picture above, present on "Regular", "Extended" and "Tiny" controllers) is "paralleled to the onboard USB-UART converter (excepting the "Tiny" board that has a dedicated USB). Simultaneous operation of the GUI and the external IMU is not possible for that reason" (https://www.basecamelectronics.com/files/v3/SimpleBGC_32bit_manual_2_6x_eng.pdf). If you accidentially connect both the GUI via USB and a flight controller via UART1 you might end up with corrupt data on the flash of the gimbal. This can result in unexpected motor movement!

**Option 2:** If you want to connect to the GUI while a flight controller via MavLink is also connected (firmware version 2.60 or above needed). Connect the flight controller as shown above but connect the rx wire (orange) to the RC-R an the tx wire (green) to the RC-Y pin on the gimbal. The ground wire (black) can be connected to any GND pin. In this connection set-up you have to take care of the following options in the GUI: - Activate RC_SERIAL by setting "RC_ROLL pin mode" = "Serial port (Serial API, etc.)" on the RC tab On the tab "External IMU" in the field "External IMU configuration" - Select the Model: "MavLin FC vhannel 1 - Connection: disabled On the same tab in the field "MavLink connection" for Channel 1 (leave Channel 2 disabled) - Serial port: "RC_serial" - System Id/Component Id: 1, 154 - Port setting: 115200, none parity - Options: Send heartbeat (checked), Query RC data (checked) - MavLink control mode: "Controls ROLL and PITCH axes only" If everything is set correctly you should see something like the following in the next field: AHRS: OK (40ms), GPS: OK (106ms), RC: OK, Control: OK CH1: Packets received: 257502, lost: 0, parse errors: 3 CH2: Packets received: 0, lost: 0, parse errors: 0

In Ardupilot/APM Planner/Mission planner set the following variables:
- :ref:`MNT_TYPE <MNT_TYPE>` to 4 / "Mount Type (None, Servo or MavLink)"
- :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` to 1 / "MavLink" (Notee "SERIAL2" should be "SERIAL1" if using Telem1 port, SERIAL4 if using Serial4/5, etc)
- :ref:`SR2_EXTRA1 <SR2_EXTRA1>` to 20
- :ref:`SR2_POSITION <SR2_POSITIOIN>` to 10
- :ref:`SR2_RC_CHAN <SR2_RC_CHAN>` to 20 and all other SR2_* variables to 0.

If you wish to control the pitch angle manually you can set
- :ref:`MNT_RC_IN_TILT <MNT_RC_IN_TILT>` to 6
- In the SimpleBCG GUI in the tab "RC Settings" in the field "Input Configuration" set PITCH to "API_VIRT_CH6".

Setup through the Ground Station
================================

Set the following parameters through your ground station and then reboot the flight controller:

- :ref:`MNT_TYPE <MNT_TYPE>` to 3 / "AlexMos-Serial"
- :ref:`SERIAL2_PROTOCOL <SERIAL2_PROTOCOL>` to 7 / "AlexMos Gimbal Serial"  (Notee "SERIAL2" should be "SERIAL1" if using Telem1 port, SERIAL4 if using Serial4/5, etc)

If you are unable to connect you may wish to set the following parameters although normally this should not be required:

- :ref:`SERIAL2_BAUD <SERIAL2_BAUD>` to 115 (means use serial baudrate of 115200)
- :ref:`BRD_SER2_RTSCTS <BRD_SER2_RTSCTS>` to 0 to disable flow control on Telem2 (use BRD_SER1_RSCTS if connecting to Serial1, Serial4/5 never uses flow control)

The gimbal's maximum lean angles can be set using these parameters:

- :ref:`MNT_ANGMIN_ROL <MNT_ANGMIN_ROL>`, :ref:`MNT_ANGMAX_ROL <MNT_ANGMAX_ROL>` to -3000 and 3000 to limit the roll angle to 30 degrees in each direction
- :ref:`MNT_ANGMIN_TIL <MNT_ANGMI_TIL>`, :ref:`MNT_ANGMAX_TIL <MNT_ANGMAX_TIL>` to -9000 and 0 to limit the gimbal to point between straight down (-90 degrees) and straight forward (0 degrees)

To control the gimbal's lean angles from a transmitter set:

- :ref:`MNT_RC_IN_TILT <MNT_RC_IN_TILT>` to 6 to control the gimbal's tilt (aka pitch angle) with the transmitter's Ch6 tuning knob

For a 3-axis gimbal with 360 degrees of yaw set:

- :ref:`MNT_ANGMIN_PAN <MNT_ANGMIN_PAN>`, :ref:`MNT_ANGMAX_PAN <MNT_ANGMAX_PAN>` to -18000 and 18000 to get a full 360 degrees of yaw range



.. _common-simplebgc-gimbal_testing_the_gimbal_moves_correctly:

Testing the gimbal moves correctly
==================================

**Testing the pilot's control of pitch**

Once powered the gimbal should point to it's Tilt Angle Min (i.e. straight down) when your transmitter's channel 6 tuning knob is at its minimum PWM value (perhaps around 1000) and the camera should point to its maximum tilt angle (i.e. straight forward) when the tuning knob is at its maximum (perhaps around 2000). 
The Mission Planner Radio calibration page can be used to check the Ch6's current input PWM value.

**Testing ROI**

**You must have GPS lock to test ROI.** 
The ROI feature points the vehicle and/or camera to point at a target. 
The instructions above describe setting up the APM/Pixhawk so that it only controls the Tilt (i.e. pitch) of the Tarot gimbal so when a :ref:`Do-Set-ROI <copter:mission-command-list_do-set-roi>` command is received Copter will attempt to turn the vehicle's nose to point in the direction of the target and tilt camera depending upon the vehicle's distance and altitude. 
You can test the gimbal tilt moves correctly by connecting with the mission planner, then on the Flight Data screen's map, right-mouse-button-click on a point about 50m ahead of the vehicle (the orange and red lines show the vehicle's current heading), select **Point Camera Here** and input an altitude of -50 (meters). 
This should cause the gimbal to point down at about 45 degrees.

.. image:: ../../../images/Tarot_BenchTestROI.jpg
    :target: ../_images/Tarot_BenchTestROI.jpg

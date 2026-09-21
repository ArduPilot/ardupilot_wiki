.. _common-external-ahrs-sensaition:

==============================
Kebni SensAItion External AHRS
==============================

The Kebni SensAItion range includes three sensor models:

- **IMU**: Measures acceleration, rotation (gyro rates), magnetic field and air pressure
- **AHRS**: As above, but also estimates the orientation (roll, pitch and yaw/heading) of the sensor
- **INS**: As above, but with two built-in satellite navigation receivers to get position (using Fixed Base RTK for higher precision if provided with correction data), UTC time and heading (if using two antennas with a sufficient baseline between them)

The SensAItion range of sensors contain high-performance gyros and accelerometers that are individually calibrated and 
temperature compensated for the interval -40 to +70 :math:`^\circ\text{C}` and thus do not need any warm-up period to 
provide precise measurements. The built-in sensor fusion engine also uses a barometer and magnetometer to improve the 
estimation of altitude and heading, respectively. The INS version of the sensor further includes satellite positioning 
in the sensor fusion, with precise internal synchronization between the satellite position fixes and the IMU measurements 
to get the best performance.

For easier integration into various customer applications, each model exists in two hardware versions: 
a rugged version in an IP67 aluminium housing or a low-profile OEM version that can easily be mounted on an 
application-specific circuit board. Details and specifications for each model can be found on 
`Kebni's product homepage <https://www.kebni.com/products/inertial-sensing/sensaition-imu-ins/>`__.

The SensAItion External AHRS driver can work in two modes:

- **IMU**: It should be connected to the IMU or AHRS model of SensAItion and will feed raw sensor data to the ArduPilot EKF3 estimator, which will fuse it with GPS positions from another source and compute a navigation solution.
- **Interleaved INS**: It should be connected to the INS model of SensAItion, which will run sensor fusion and provide a full navigation solution, replacing the EKF3 estimator. In order not to overload the serial port, the high-rate IMU messages are interleaved with lower-rate positions and orientations. This interleaving requires the sensor to have firmware version 2.69 or newer.

Hardware setup
==============
Connect the SensAItion Data UART pins (TX, RX and ground) to an available serial port on the ArduPilot platform, 
such as GPS or TELEM. Power the sensor as described in the User Manual. 
Both the rugged and OEM version of the sensor have the required serial port connections available. 
Please note that the rugged sensor version has output voltage levels +/-5 V, 
while the OEM sensor version has output voltage levels of 0 and +3.3 V.

The SensAItion sensor can be mounted in any orientation and the output can be rotated to match the local 
coordinate system if needed - see the configuration section below. 
As with any IMU, it is important to avoid excessive vibration levels and it is beneficial to use a 
mechanical damper between the sensor and the platform frame.

Configuration of the sensor
============================
The SensAItion sensor has a number of configuration registers that control the baudrate, output data format, 
coordinate system rotation and much more. The configuration registers can be read and written using 
a text-based protocol via the User UART port as described in the User Manual. 
But Kebni also provides, on request, the Kebni INSight graphical user interface that helps the user 
set all configuration registers and can also be used to read and plot sensor data and perform a magnetometer calibration. 
For complete control of the sensor configuration, it is recommended to use INSight. 

Contact `Kebni <https://www.kebni.com/contact/>`__ for access to the User Manual or Kebni INSight.

To get started with the default configuration, it is sufficient to connect to the User UART port of the sensor with a terminal program.

Start by doing a factory reset of all configuration registers, saving to flash and doing a reboot:

.. code-block:: bash

    $PKEBF*5A
    $PKEBS*4F
    $PKEBB*5E

After getting a reboot message from the sensor, send the following commands to enable Data UART output and set the baudrate to 460800:

.. code-block:: bash

    $PKEBW,7,2*4E
    $PKEBW,9,46080*48

For an IMU or AHRS sensor, send the following command that sets an output rate of 1000 Hz and determines the contents of 
each message (acceleration, gyro rates, temperature, magnetic field and air pressure):

.. code-block:: bash

    $PKEBW,3,o0001s240030020010000130120110100230220210200330320310300430420410400530520510500910900A10A00B10B00C10C00D30D20D10D0x*1B

For an INS sensor, send the following command that interleaves IMU data at 500 Hz with AHRS and INS data at 100 Hz:

.. code-block:: bash

    $PKEBW,3,o0002i00R0B00F01F02F03F04F05F0930A30B30C30DFxo000Ai01R044CF4DF4EF4FFxo000Ai02R142BF2FF30131F32F33F34F36F35F40143F47545F46861F62F63F64F66F65Fx*XX

Then save the updated register values to flash and reboot to make them active:

.. code-block:: bash

    $PKEBS*4F
    $PKEBB*5E


Configuration of ArduPilot
==========================
The following parameters must be set in IMU mode:

=========================================== ======== ===============================================
Parameter                                   Value    Comment
=========================================== ======== ===============================================
:ref:`EAHRS_TYPE<EAHRS_TYPE>`               11       Use SensAItion as external AHRS
:ref:`EAHRS_SENSORS<EAHRS_SENSORS>`         14       Use IMU, barometer and compass
:ref:`EAHRS_RATE<EAHRS_RATE>`               1000     Requested rate for AHRS device
:ref:`EAHRS_OPTIONS<EAHRS_OPTIONS>`         0        0: IMU mode  4: INS mode
:ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>`         3        Use EKF3 for sensor fusion
:ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>`   36       External AHRS on TELEM 2 (change as needed)
:ref:`SERIAL2_BAUD<SERIAL2_BAUD>`           460      460800 baud (change as needed)
=========================================== ======== ===============================================


The following parameters must be set in INS mode:

=========================================== ======== ===============================================
Parameter                                   Value    Comment
=========================================== ======== ===============================================
:ref:`EAHRS_TYPE<EAHRS_TYPE>`               11       Use SensAItion as external AHRS
:ref:`EAHRS_SENSORS<EAHRS_SENSORS>`         15       Use GPS, IMU, barometer and compass
:ref:`EAHRS_RATE<EAHRS_RATE>`               500      Requested rate for AHRS device
:ref:`EAHRS_OPTIONS<EAHRS_OPTIONS>`         4        0: IMU mode  4: INS mode
:ref:`AHRS_EKF_TYPE<AHRS_EKF_TYPE>`         11       Use External AHRS for sensor fusion
:ref:`SERIAL2_PROTOCOL<SERIAL2_PROTOCOL>`   36       External AHRS on TELEM 2 (change as needed)
:ref:`SERIAL2_BAUD<SERIAL2_BAUD>`           460      460800 baud (change as needed)
=========================================== ======== ===============================================

Also remember to build the ArduPilot binary with these flags: 

``--enable-AHRS_EXT --enable-EXTERNALAHRS_COMPASS --enable-EXTERNALAHRS_BARO``.

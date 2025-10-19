SAAMPIXV1_1 (STM32F427) Flight Controller
=========================================

Overview
--------
The **SaamPix 1.0 / SAAMPIXV1_1** is a high-performance open-source flight controller designed by **Saam Drones** (Gurgaon, India).  
It is based on the **STM32F427** ARM Cortex-M4 FPU MCU and is compatible with both **ArduPilot** and **PX4** firmware stacks.  
With its rich I/O, dual IMUs, and wide protocol support, it targets multirotors, fixed-wings, VTOLs, and hybrid aircraft for research, defense, and industrial UAV applications.

Quick Specs
-----------
- **MCU:** STM32F427 (32-bit ARM Cortex-M4 FPU @ 168 MHz)
- **Memory:** 256 KB SRAM, 2 MB Flash
- **Board ID:** 7130
- **IMU & Sensors:**
  - ICM-20602 — 6-axis gyro + accelerometer  
  - MPU-9250 — 9-axis gyro + accelerometer + magnetometer  
  - LIS3MDL — 3-axis magnetometer  
  - MS5611 — high-resolution barometer
- **PWM Outputs:** 16 total (8 main + 8 auxiliary)
- **Telemetry:** Up to 3 serial links (MAVLink supported)
- **Expansion Ports:** I2C, UART, CAN, SWD
- **Storage:** microSD card for flight logs
- **Receiver Support:** PPM, SBUS, DSM, DSM2, DSMX
- **FrSky Telemetry:** D and S.Port supported
- **Safety:** Safety switch, buzzer, and LED indicators
- **Power Input:** via Power Module (with voltage and current sensing)

Hardware Connections
--------------------
.. image:: /images/saampixv1_1_layout.png
   :alt: SaamPix 1.0 Flight Controller layout
   :width: 500px

**External Ports and Connectors:**
- **Power Module:** Main power input and voltage/current sense  
- **ADC, GPS:** Dedicated inputs for analog and GPS modules  
- **USB1 / USB2:** Dual USB interfaces (for GCS and data links)  
- **I2C Port:** Connect external sensors (e.g., lidar or rangefinders)  
- **CAN:** For DroneCAN / UAVCAN devices and peripherals  
- **SWD:** Debug/program interface  
- **Telemetry 1 / Telemetry 2:** Serial ports for MAVLink radios  
- **SD Card:** microSD for log storage  
- **Serial 4, 5:** Additional UART / Telemetry ports  

Building Firmware
-----------------
.. code-block:: bash

   ./waf distclean
   ./waf configure --board SAAMPIXV1_1
   ./waf copter     # or plane / rover

Flashing
--------
Use **Mission Planner** (ArduPilot) or **QGroundControl** (PX4) to flash the firmware.  
Select *Custom Firmware* and load the `.apj` image built for **board ID 7130**.

USB Identification
------------------
- **Manufacturer:** Saam Drones  
- **Product:** SaamPix 1.0 Flight Controller  

Receiver & Radio Compatibility
------------------------------
Supports PPM, SBUS, DSM, DSM2, DSMX receivers.  
FrSky telemetry is available over D and S.Port.

Peripheral Connections
----------------------
- **GPS & Compass:** via dedicated GPS port (ICM / LIS3MDL)  
- **ESCs / Servos:** 8 main + 8 aux PWM outputs  
- **Telemetry / Data Links:** TELEM1, TELEM2, UART ports  
- **Additional Sensors:** I2C or CAN ports (for Lidar, Rangefinder, Optical Flow)  

Setup and First Flight
----------------------
1. Install **Mission Planner** or **QGroundControl**.  
2. Connect via micro-USB; the board appears as a COM port.  
3. Flash firmware (ArduCopter, ArduPlane, or PX4 Multicopter).  
4. Calibrate accelerometer, gyroscope, magnetometer, barometer, and RC input.  
5. Configure power monitoring and failsafe options.  
6. Perform pre-arm checks and test safety switch/buzzer before flight.  

Notes
-----
- Dual IMU redundancy for robust attitude estimation.  
- Built-in support for DroneCAN nodes and companion computers.  
- Ensure secure mounting and isolation from vibration for IMU performance.

Vendor / Company Info
----------------------
**Saam Drones Private Limited**  
Gurgaon, Haryana, India  
Website: https://www.saamdrones.com/  

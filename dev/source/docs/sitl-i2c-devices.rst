.. _sitl-i2c-devices:

================
SITL I2C Devices
================

The SITL (software in the loop) simulates many different i2c devices, including:

- MaxSonar i2c-connected sonar
- various SMBus battery monitors
- LED devices
- temperature sensors
- airspeed sensors
- IMU drivers

Overview
========

``SITL/SIM_I2C.cpp`` contains the instantitions and initialisation of the buses and drivers.  It also contains the hook used by the SITL HAL to make simulated i2c bus transactions with the device, ``I2C::ioctl``

``SITL/SIM/I2CDevice.h`` declares the I2CDevice class from which all simulated I2C drivers inherit from.  The general structure of an I2CDevice is:

 - an init() method
 - an ``update`` method called periodically to simulate whatever the device accomplishes on its own internal clock
 - a ``rdwr`` call which is called when ArduPilot attempts an i2c transaction

i2c devices vary in the structure of the on-wire protocol.  Some examples of protocol shapes include:

- simply stream bytes onto the i2c bus; when a read is attempted from one of these devices a response is received with bytes filled in, regardless of what transfer is made to the device.  SIM_Airspeed_DLVR is an example of an implementation of this sort of device, never allowing i2c writes to the device and always returning 4 bytes of data.

- respond to a simple i2c "take reading command" by making available a reading on a subsequent i2c read transaction.  The SIM_MaxSonarI2CXL is an example of this sort of device, responding to a command of 0x51 by taking a measurement and allowing a subsequent read of 2 bytes to return that measurement.

- a register-based interface where the ArduPilot driver writes to registers at specific offsets to configure the device and reads data back via the same interface.  i2c transactions are usually a pair of couple transactions, one to specify the registers to read and the other for the simulated device to return the data.  The simulated IS31FL3195 LED driver is an example of this, presenting a number of control registers used for setting the LED patterns and device behaviour.  These i2c devices often have a "WHOAMI" register to determine the device type.  SMBus devices also often speak this protocol, but using 16-bit registers rather than the usual 8-bit.

Choosing a Device Style
=======================

When implementing a new simulator, care must be taken to choose the correct "style" of simulator to implement.  Several base classes can be inheritted from to prevent warrantless duplication.  As always, find a good example of something similar to the device you wish to simulate and "do what that thing does".  Working out what the protocol is requires study of the datasheet for the device.

For devices which are strictly register-based, the ``I2CRegisters_8Bit`` (or ``I2CRegisters_16Bit``) base class should be inheritted from.  This allows your new simulator to describe the registers present on the device, and simply fill the register values in based on how ArduPilot manipulates other registers.  It is also possible to create simulates devices with both 8 and 16 bit registers - see ``I2CRegisters_ConfigurableLength``

For devices which are polled to start some sort of measurement and subsequently make available a 16-bit quantity for reading ``I2CCommandResponseDevice`` may be used to write a simple driver.

For devices which simply return fixed-length readings when an i2c "read" is made - and for any other protocol which doesn't yet have an abstraction - your simulation will need to implement the virtual method ``rdwr(I2C::i2c_rdwr_ioctl_data *&data)`` inheritted from SITL::I2CDevice.


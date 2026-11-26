.. _code-overview-sensor-drivers:

==============
Sensor Drivers
==============

ArduPilot supports a wide variety of sensors from many different manufacturers.  One clear example of this can be seen in the :ref:`list of range finders <copter:common-rangefinder-landingpage>` (aka sonars, lidars).
This page attempts to explain how sensor drivers are written and integrated into the vehicle code.

Supported Protocols
===================

`I2C <https://en.wikipedia.org/wiki/I%C2%B2C>`__, `SPI <https://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus>`__,
`UART (aka Serial) <https://en.wikipedia.org/wiki/Universal_asynchronous_receiver/transmitter>`__ and `CANBUS <https://en.wikipedia.org/wiki/CAN_bus>`__ (in particular UAVCAN/DroneCAN) protocols are supported.

If you plan to write a new driver, you will likely need to refer to the sensor's datasheet in order to determine which protocol it uses. Alternatively, you could utilize an intermediary node to move the necessary data back to the autopilot. 

I2C
---

- one master, many slaves possible
- a relatively simple protocol which is good for communicating over short-distances (i.e. less than 1m).
- bus runs at 100kHz or 400kHz but the data rate is relatively low compared to other protocols.
- only 4 pins are required (VCC, GND, SDA, SCL)

.. image:: ../images/code-overview-sensor-driver-i2c1.png
.. image:: ../images/code-overview-sensor-driver-i2c2.png

SPI
---

- one master, one slave
- 20Mhz+ speed meaning it is very fast especially compared to I2C
- only works over short distances (10cm)
- requires at least 5 pins (VCC, GND, SCLK, Master-Out-Slave-In, Master-In-Slave-Out) + 1 slave select pin per slave

.. image:: ../images/code-overview-sensor-driver-spi1.png
.. image:: ../images/code-overview-sensor-driver-spi2.png

Serial / UART
-------------

- one master, one slave
- character based protocol good for communicating over longer distances compared to I2C and SPI (i.e. 1m)
- relatively fast at 57Kbps ~ 1.5Mbps
- at least 4 pins required (VCC, GND, TX, RX), plus 2 optional pins (Clear-To-Send, Clear-To-Receive)

CAN bus with UAVCAN
-------------------

- multimaster bus, any node can initiate transmission of data when they need to
- packet based protocol for very long distances
- high speed, typically 1 Mb (however only 50% of the bus bitrate can really be used without major collisions)
- at least 3 pins required (GND, CAN HI, CAN LO). Optionally VCC can be used to power nodes
- point-to-point topology. Star or stubs topolgy is not advised
- termination is required at each end of the bus

.. image:: ../images/code-overview-can-bus.png

FrontEnd / BackEnd Split
========================

An important concept within the sensor driver architecture is the front-end / back-end split.

.. image:: ../images/code-overview-sensor-drivers-febesplit.png

The vehicle code only ever calls into the Library's (aka sensor driver's) front-end.

On start-up the front-end creates one or more back-ends based either on automatic detection of the sensor (i.e. probing for a response on a known I2C address)
or by using the user defined _TYPE params (i.e. RNGFND_TYPE, RNGFND_TYPE2).  The front-end maintains pointers to each back-end which are normally held within an array named _drivers[].

User settable parameters are always held within the front-end.

How and When the driver code is run
===================================

.. image:: ../images/copter-code-overview-architecture2.png

The image above shows a zoomed in view of the ardupilot architecture.  The top-left blue box illustrates how the sensor driver's back-ends are run in a background thread.
Raw data from the sensors is collected, converted into standard units and then held within buffers within the driver.

The vehicle code's main thread runs regularly (i.e. 400hz for copter) and accesses the latest data available through methods in the driver's front-end.
For example in order to calculate the latest attitude estimate, the AHRS/EKF would pull the latest accelerometer, gyro and compass information from the sensor drivers' front-ends.

The image is a slight generalization, for drivers using I2C or SPI, they must run in the background thread so that the highrate communication with the sensor does not affect
the main loop's performance but for driver's using a serial (aka UART) interface, it is safe to run in the main thread because the underlying serial driver itself collects data
in the background and includes a buffer.

Vehicle Code and Front-End Example
==================================

The example below shows how the Copter vehicle code pulls data from range finder (aka sonar, lidar) drivers.
The Copter code's `scheduler <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/Copter.cpp#L107>`__ calls the vehicle's read_rangefinder() method at 20Hz.
Below is implementation of this method, the latest version can be seen in the `sensors.cpp <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/sensors.cpp>`__ file.
The rangefinder.update() method is a call into the driver's front-end.

.. code-block:: cpp
    :linenos:
    :emphasize-lines: 1,3

    void Copter::read_rangefinder(void)
    {
        rangefinder.update();

        rangefinder_state.update();
        rangefinder_up_state.update();

    #if HAL_PROXIMITY_ENABLED
        if (rangefinder_state.enabled_and_healthy() || rangefinder_state.data_stale()) {
            g2.proximity.set_rangefinder_alt(rangefinder_state.enabled, rangefinder_state.alt_healthy, rangefinder_state.alt_m_filt.get() * 100.0);
        }
    #endif
    }

Below is the range finder driver's back-end `update method <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RangeFinder/AP_RangeFinder.cpp#L249>`__.
This gives the driver a chance to do any general processing it might want to within the main thread.
Each back-end's update method is called in turn.

.. code-block:: cpp
    :linenos:
    :emphasize-lines: 1

    void RangeFinder::update(void)
    {
        for (uint8_t i=0; i<num_instances; i++) {
            if (drivers[i] != nullptr) {
                if ((Type)params[i].type.get() == Type::NONE) {
                    // allow user to disable a rangefinder at runtime
                    state[i].status = Status::NotConnected;
                    state[i].range_valid_count = 0;
                    continue;
                }
                drivers[i]->update();
            }
        }
    #if HAL_LOGGING_ENABLED
        Log_RFND();
    #endif
    }

UART/Serial Back-End Example
============================

Next is the update method of the `LightWare back-end <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RangeFinder/AP_RangeFinder_LightWareSerial.cpp>`__ using the serial protocol.
As described on the `user wiki <https://ardupilot.org/copter/docs/common-lightware-sf10-lidar.html#serial-connection>`__ the serial range finder can be connected to any of the flight controller's serial ports but the user must specify which serial port, and what baud rate is used by setting the SERIALX_BAUD and SERIALX_PROTOCOL parameters.

.. image:: ../images/code-overview-sensor-driver-uart1.png

Within the serial driver's `backend code <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_RangeFinder/AP_RangeFinder_Backend_Serial.cpp>`__, it first finds which UART the user would like to use via the serial_manager class which looks for the parameters settings described above.

.. code-block:: cpp
    :linenos:
    :emphasize-lines: 3

    void AP_RangeFinder_Backend_Serial::init_serial(uint8_t serial_instance)
    {
        uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder,    serial_instance); // Get UART Interface
        if (uart != nullptr) {
            uart->begin(initial_baudrate(serial_instance), rx_bufsize(), tx_bufsize());
        }
    }

Each time the driver's back-end update() method is called it calls the get_reading() method which checks if new characters have arrived from the sensor and then decodes them.

As mentioned above, because the serial protocol implements its own buffering, the processing of any data (see get_reading method) from the sensor is run here in the main thread.
I.e. there is no "register_periodic_callback" like you will see in I2C and SPI drivers.

.. code-block:: cpp
    :linenos:
    :emphasize-lines: 3

    void AP_RangeFinder_Backend_Serial::update(void)
    {
        if (get_reading(state.distance_m)) {
            state.signal_quality_pct = get_signal_quality_pct();
            // update range_valid state based on distance measured
            state.last_reading_ms = AP_HAL::millis();
            update_status();
        } else if (AP_HAL::millis() - state.last_reading_ms > read_timeout_ms()) {
            set_status(RangeFinder::Status::NoData);
        }
    }

.. code-block:: cpp
    :linenos:
    :emphasize-lines: 3

    bool AP_RangeFinder_LightWareSerial::get_reading(float &reading_m)
    {
         if (uart == nullptr) { // Check if UART is Available 
            return false;
    }

    float sum = 0;              // sum of all readings taken
    uint16_t valid_count = 0;   // number of valid readings
    uint16_t invalid_count = 0; // number of invalid readings

    // max distance the sensor can reliably measure - read from parameters
    const auto distance_cm_max = max_distance()*100;

    // read any available lines from the lidar
    for (auto i=0; i<8192; i++) {
        uint8_t c;
        if (!uart->read(c)) { // Read the Available Data
            break;
        }
        // Use Proper Protocol to convert data into readings 
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::LEGACY) {
            if (c == '\r') {
                linebuf[linebuf_len] = 0;
                const float dist = strtof(linebuf, nullptr);
                if (!is_negative(dist) && !is_lost_signal_distance(dist * 100, distance_cm_max)) {
                    sum += dist;
                    valid_count++;
                    // if still determining protocol update legacy valid count
                    if (protocol_state == ProtocolState::UNKNOWN) {
                        legacy_valid_count++;
                    }
                } else {
                    invalid_count++;
                }
                linebuf_len = 0;
            } else if (isdigit(c) || c == '.' || c == '-') {
                linebuf[linebuf_len++] = c;
                if (linebuf_len == sizeof(linebuf)) {
                    // too long, discard the line
                    linebuf_len = 0;
                }
            }
        }

        // use binary protocol
        if (protocol_state == ProtocolState::UNKNOWN || protocol_state == ProtocolState::BINARY) {
            bool msb_set = BIT_IS_SET(c, 7);
            if (msb_set) {
                // received the high byte
                high_byte = c;
                high_byte_received = true;
            } else {
                // received the low byte which should be second
                if (high_byte_received) {
                    const int16_t dist = (high_byte & 0x7f) << 7 | (c & 0x7f);
                    if (dist >= 0 && !is_lost_signal_distance(dist, distance_cm_max)) {
                        sum += dist * 0.01f;
                        valid_count++;
                        // if still determining protocol update binary valid count
                        if (protocol_state == ProtocolState::UNKNOWN) {
                            binary_valid_count++;
                        }
                    } else {
                        invalid_count++;
                    }
                }
                high_byte_received = false;
            }
        }
    }

    // protocol set after 10 successful reads
    if (protocol_state == ProtocolState::UNKNOWN) {
        if (binary_valid_count > 10) {
            protocol_state = ProtocolState::BINARY;
        } else if (legacy_valid_count > 10) {
            protocol_state = ProtocolState::LEGACY;
        }
    }

    uint32_t now = AP_HAL::millis();
    if (last_init_ms == 0 ||
        (now - last_init_ms > 1000 &&
         now - state.last_reading_ms > 1000)) {
        // send enough serial transitions to trigger LW20 into serial
        // mode. It starts in dual I2C/serial mode, and wants to see
        // enough transitions to switch into serial mode.
        uart->write("www\r\n");
        last_init_ms = now;
    } else {
        uart->write('d');
    }

    // return average of all valid readings
    if (valid_count > 0) {
        reading_m = sum / valid_count;
        no_signal = false;
        return true;
    }

    // all readings were invalid so return out-of-range-high value
    if (invalid_count > 0) {
        reading_m = MAX(LIGHTWARE_DIST_MAX_CM, distance_cm_max + LIGHTWARE_OUT_OF_RANGE_ADD_CM);
        no_signal = true;
        return true;
    }

    // no readings so return false
    return false;
    }

I2C Back-End Example
====================

This example shows the back-end for the Lightware I2C driver.
In this case, the front-end gets the I2C bus and passes it to the back-end during initialisation.

The driver first obtains the I2C device using the configured bus number and sensor address.

.. code-block:: cpp
   :caption: Getting the I2C device

   _dev = hal.i2c_mgr->get_device(
       HAL_RANGEFINDER_LIGHTWARE_I2C_BUS,
       _address[instance]);

After obtaining the device, the driver registers a periodic callback that invokes ``timer()`` every 50 ms (20 Hz).

.. code-block:: cpp
   :caption: Registering a periodic callback

   void AP_RangeFinder_LightWareI2C::init()
   {
       // call timer() at 20Hz
       _dev->register_periodic_callback(50000,
           FUNCTOR_BIND_MEMBER(&AP_RangeFinder_LightWareI2C::timer, bool));
   }

Finally, the driver reads the sensor data over I2C and converts the received big-endian value into centimeters.

.. code-block:: cpp
   :caption: Reading data from the sensor
   :emphasize-lines: 10,13

   bool AP_RangeFinder_LightWareI2C::get_reading(uint16_t &reading_cm)
   {
       be16_t val;

       if (ranger._address[state.instance] == 0) {
           return false;
       }

       // read the high and low byte distance registers
       bool ret = _dev->read((uint8_t *)&val, sizeof(val));
       if (ret) {
           // combine results into distance
           reading_cm = be16toh(val);
       }

       return ret;
   }


The back-end's init method then registers its "timer" method to be called at 20hz.  Within the timer method (not shown) the get_reading() method is called which reads bytes from the sensor and converts the distance to centimeters.

SPI Back-End Example
====================

This example shows the back-end for the MPU9250 IMU which includes a gyro, accelerometer and compass.
The front-end gets the SPI bus and passes it to the back-end during initialisation.

The backend is created by obtaining the SPI device and probing the sensor.

.. code-block:: cpp
   :caption: Obtaining the SPI device

   void AP_InertialSensor::detect_backends(void)
   {
       _add_backend(AP_InertialSensor_MPU9250::probe(*this,
                    hal.spi->get_device(HAL_INS_MPU9250_NAME)));
   }

   

the start() method is called during initialisation and configures the sensor.
It uses semaphores to ensure no interference with other SPI devices on the same bus.

.. code-block:: cpp
   :caption: Initializing the sensor and registering the periodic callback
   :emphasize-lines: 3,8,9,12

   void AP_InertialSensor_MPU9250::start()
   {
       if (!_dev->get_semaphore()->take(1)) {
           return;
       }

       // initially run the bus at low speed
       _dev->set_speed(AP_HAL::Device::SPEED_LOW);
       _dev->get_semaphore()->give();

       // start the timer process to read samples
       _dev->register_periodic_callback(1000,
           FUNCTOR_BIND_MEMBER(&AP_InertialSensor_MPU9250::_read_sample, bool));
   }

The _read_sample method is registered so that it is called at 1000hz.  Note there is no need to take/give semaphores within the _read_sample method because that is done as part of the periodic callback code.

The ``_read_sample()`` method processes the raw data read from the sensor.

.. code-block:: cpp
   :caption: Decoding the sensor data

   bool AP_InertialSensor_MPU9250::_read_sample()
   {
       uint8_t n_samples;
       uint16_t bytes_read;
       uint8_t *rx = _fifo_buffer;
       bool need_reset = false;

       if (!_block_read(MPUREG_FIFO_COUNT, rx, 2)) {
           goto check_registers;
       }

       bytes_read = uint16_val(rx, 0);
       n_samples = bytes_read / MPU_SAMPLE_SIZE;
   }

the _block_read method shows how data can be read from the sensor's registers.

The ``_block_read()`` method reads data from the sensor registers over SPI.

.. code-block:: cpp
   :caption: Reading sensor registers

   bool AP_InertialSensor_MPU9250::_block_read(uint8_t reg,
                                               uint8_t *buf,
                                               uint32_t size)
   {
       return _dev->read_registers(reg, buf, size);
   }




Additional Advice
=================

When writing a sensor driver, never include any wait or sleep code because this will either delay the main thread or the background thread associated with the bus being used.

If a new library is written, it must be added to the wscript file in the vehicle directory (i.e. /ardupilot/ArduCopter/wscript) in order for it to be linked into the final binary

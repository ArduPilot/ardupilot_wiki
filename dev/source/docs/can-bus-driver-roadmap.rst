.. _can-bus-driver-roadmap:

=====================================
CAN bus driver implementation roadmap
=====================================

The base for hardware CAN bus support is located in AP_HAL library and
consist of two classes:

- CANHal class that is responsible for representation of one physical interface on board.
  This class manages opening, setup and operation of the interface and is a main connection
  point between software and hardware
  
- CANManager class is wrapping all physical interfaces.
  It does enumeration of interfaces, provides access to them and also holds connection
  point for accessing UAVCAN managing class.

CANHal class
============

New class should be based on AP_HAL::CANHal class.

The class is responsible for managing the hardware and also managing of RX and TX queues.
Apart from this also the time management is provided.

Necessary methods are following:

- Constructor

- int init(const uint32_t bitrate, const OperatingMode mode)

  Initializes the hardware CAN controller with specific bitrate and operating mode.
  The driver should try to match the specified bitrate as close as possible.
  Operating mode is either Normal that allows sending frames or Silent - only for
  receiving of frames.
  This method is used internally only by begin() method.
  
- bool begin(uint32_t bitrate)

  This method should try to initialize CAN interface with specified bitrate.
  
- void end()

  Finishes the instance of CAN interfaces
  
- void reset()

  Reinitialize the interface with previous settings
  
- bool is_initialized()

  Returns true if the CAN interface was initialized successfully
  
- int32_t available()

  This method should return the length of RX queue.
  
- int32_t tx_pending()

  The number of pending messages to be transmitted is returned by this method.

- bool canAcceptNewTxFrame(const uavcan::CanFrame& frame)

  This method returns true if new message for transmitting can be placed in TX queue.
  
- bool isRxBufferEmpty()

  Method checks if the RX queue is empty.

- uint64_t getErrorCount()

  Total number of hardware failures and other kinds of errors (e.g. queue overruns).
  May increase continuously if the interface is not connected to the bus.
  
- uint32_t getVoluntaryTxAbortCount()

  Number of times the driver exercised library's requirement to abort transmission on first error.

- unsigned getRxQueueLength()

  Returns the number of frames pending in the RX queue.

- bool hadActivity()

  Returns whether this iface had at least one successful IO since the previous call of this method.
  This is designed for use with iface activity LEDs.

- int16_t send(const uavcan::CanFrame& frame, uavcan::MonotonicTime tx_deadline, uavcan::CanIOFlags flags)

  The method is responsible for putting a new frame to transmitting queue.
  The maximum transmitting deadline is also provided and it is a responsibility of driver to
  remove frame from queue if the deadline has passed.
  The flags can be combination of none or Loopback or AbortOnError.

- int16_t receive(uavcan::CanFrame& out_frame, uavcan::MonotonicTime& out_ts_monotonic, uavcan::UtcTime& out_ts_utc, uavcan::CanIOFlags& out_flags)

  The method fills all the data in the passed references with the first message in RX queue.

CANManager class
================

New class should be based on AP_HAL::CANManager class.

Necessary methods are following:

- Constructor

- int init(const uint32_t bitrate, const PX4CAN::OperatingMode mode, uint8_t can_number)

  Initializes the specified CAN interface with specific bitrate and operating mode.
  This method is used internally only by begin() method.

- bool begin(uint32_t bitrate, uint8_t can_number)

  This method should try to initialize specified CAN interface with specified bitrate.

- uavcan::CanSelectMasks makeSelectMasks(const uavcan::CanFrame* (&pending_tx)[uavcan::MaxCanIfaces])

  This function returns select masks indicating which interfaces are available for read/write.

- PX4CAN* getIface(uint8_t iface_index)

  Returns reference to the specified interface
  
- uint8_t getNumIfaces()

  Returns number of interfaces
  
- bool hadActivity();

  Whether at least one iface had at least one successful IO since previous call of this method.
  This is designed for use with iface activity LEDs.

- bool is_initialized() override;

  Returns true if the CAN manager was initialized successfully

- void set_UAVCAN(AP_UAVCAN \*uavcan)

  Method stores the pointer to the UAVCAN instance

- AP_UAVCAN \*get_UAVCAN(void)

  Method returns the pointer to the UAVCAN instance that is connected with this manager

.. _learning-ardupilot-threading:

=========
Threading
=========

Understanding ArduPilot threading
---------------------------------

Once you have learned the basic of the ArduPilot libraries it is time
for you to understand how ArduPilot deals with threading. The
setup()/loop() structure that was inherited from arduino may make it
seem that ArduPilot is a single threaded system, but in fact it isn't.

The threading approach in ArduPilot depends on the board it is built
for. Some boards (such as the APM1 and APM2) don't support threads, so
make do with a simple timer and callbacks. Some boards (PX4 and Linux)
support a rich Posix threading model with realtime priorities, and these
are used extensively by ArduPilot.

There are a number of key concepts related to threading that you need to
understand in ArduPilot:

-  The timer callbacks
-  HAL specific threads
-  driver specific threads
-  ardupilot drivers versus platform drivers
-  platform specific threads and tasks
-  the AP_Scheduler system
-  semaphores
-  lockless data structures

The timer callbacks
===================

Every platform provides a 1kHz timer in the AP_HAL. Any code in
ArduPilot can register a timer function which is then called at 1kHz.
All registered timer functions are called sequentially. This very
primitive mechanism is used as it is extremely portable, and yet very
useful. You register a timer callback by calling the
hal.scheduler->register_timer_process() like this:

::

      hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_Baro_MS5611::_update));

that particular example is from the MS5611 barometer driver. The
AP_HAL_MEMBERPROC() macro provides a way to encapsulate a C++ member
function as a callback argument (bundling up the object context with the
function pointer).

When a piece of code wants something to happen at less than 1kHz then it
should maintain it's own "last_called" variable and return immediately
if not enough time has passed. You can use the hal.scheduler->millis()
and hal.scheduler->micros() functions to get the time since boot in
milliseconds and microseconds to support this.

You should now go and modify an existing example sketch (or create a new
one) and add a timer callback. Make the timer increment a counter then
print the value of the counter every second in the loop() function.
Modify your function so that it increments the counter  every 25
milliseconds.

HAL specific threads
====================

On platforms that support real threads the AP_HAL for that platform
will create a number of threads to support basic operations. For
example, on Pixhawk the following HAL specific threads are created:

-  The UART thread, for reading and writing UARTs (and USB)
-  The timer thread, which supports the 1kHz timer functionality
   described above
-  The IO thread, which supports writing to the microSD card, EEPROM and
   FRAM

Have a look in Scheduler.cpp inside each AP_HAL implementation to see
what threads are created and what the realtime priority of each thread
is.

If you have a Pixhawk then you should also now setup a debug console
cable and attach to the nsh console (the serial5 port). Connect at
57600. When you have connected, try the "ps" command ad you will get
something like this:

::

    PID PRI SCHD TYPE NP STATE NAME
     0 0 FIFO TASK READY Idle Task()
     1 192 FIFO KTHREAD WAITSIG hpwork()
     2 50 FIFO KTHREAD WAITSIG lpwork()
     3 100 FIFO TASK RUNNING init()
     37 180 FIFO TASK WAITSEM AHRS_Test()
     38 181 FIFO PTHREAD WAITSEM <pthread>(20005400)
     39 60 FIFO PTHREAD READY <pthread>(20005400)
     40 59 FIFO PTHREAD WAITSEM <pthread>(20005400)
     10 240 FIFO TASK WAITSEM px4io()
     13 100 FIFO TASK WAITSEM fmuservo()
     30 240 FIFO TASK WAITSEM uavcan()

In this example you can see the "AHRS_Test" thread, which is running
the example sketch from libraries/AP_AHRS/examples/AHRS_Test. You can
also see the timer thread (priority 181), the UART thread (priority 60)
and the IO thread (priority 59).

Additionally you can see the px4io, fmuservo, uavcan, lpwork, hpwork and
idle tasks. More about those later.

Other AP_HAL ports have more or less threads depending on what is
needed.

One common use of threads is to provide drivers a way to schedule slow
tasks without interrupting the main autopilot flight code. For example,
the AP_Terrain library needs to be able to do file IO to the microSD
card (to store and retrieve terrain data). The way it does this is it
calls the function hal.scheduler->register_io_process() like this:

::

    hal.scheduler->register_io_process(AP_HAL_MEMBERPROC(&AP_Terrain::io_timer));

The sets up the AP_Terrain::io_timer function to be called regularly.
That is called within the boards IO thread, meaning it is a low realtime
priority and is suitable for storage IO tasks. It is important that slow
IO tasks like this not be called on the timer thread as they would cause
delays in the more important processing of high speed sensor data.

Driver specific threads
=======================

It is also possible to create driver specific threads, to support
asynchronous processing in a manner specific to one driver. Currently
you can only create driver specific threads in a manner that is platform
dependent, so this is only appropriate if your driver is intended to run
only on one type of autopilot board. If you want it to run on multiple
AP_HAL targets then you have two choices:

-  you can use the register_io_process() and
   register_timer_process() scheduler calls to use the existing timer
   or IO threads
-  you can add a new HAL interface that provides a generic way to create
   threads on multiple AP_HAL targets (please contribute patches back)

An example of a driver specific thread is the ToneAlarm thread in the
Linux port. See AP_HAL_Linux/ToneAlarmDriver.cpp

ArduPilot drivers versus platform drivers
=========================================

You may notice some driver duplication in ArduPilot. For example, we
have a MPU6000 driver in
libraries/AP_InertalSensor/AP_InertialSensor_MPU6000.cpp, and another
MPU6000 driver in PX4Firmware/src/drivers/mpu6000.

The reason for this duplication is that the PX4 project already provides
a set of well tested drivers for hardware that comes with Pixhawk boards,
and we enjoy a good collaborative relationship with the PX4 team on
developing and enhancing these drivers. So when we build ArduPilot for
PX4 we take advantage of the PX4 drivers by writing small "shim" drivers
which present the PX4 drivers with the standard ArduPilot library
interface. If you look at
libraries/AP_InertialSensor/AP_InertialSensor_PX4.cpp you will see a
small shim driver that asks the PX4 what IMU drivers are available on
this board and automatically makes all of them available as part of the
ArduPilot AP_InertialSensor library.

So if we have an MPU6000 on the board we use the
AP_InertialSensor_MPU6000.cpp driver on non-Pixhawk/NuttX platforms, and the
AP_InertialSensor_PX4.cpp driver on NuttX based platforms.

The same type of split can also happen for other AP_HAL ports. For
example, we could use Linux kernel drivers for some sensors on Linux
boards. For other sensors we use the generic AP_HAL I2C and SPI
interfaces to use the ArduPilot "in-tree" drivers which work across a
wide range of boards.

Platform specific threads and tasks
===================================

On some platforms there will be a number of base tasks and threads that
will be created by the startup process. These are very platform specific
so for the sake of this tutorial I will concentrate on the tasks used on
PX4 based boards.

In the "ps" output above we saw a number of tasks and threads that were
not started by the AP_HAL_PX4 Scheduler code. Specifically they are:

-  idle task - called when there is nothing else to run
-  init - used to start up the system
-  px4io - handle the communication with the PX4IO co-processor
-  hpwork - handle thread based PX4 drivers (mainly I2C drivers)
-  lpwork - handle thread based low priority work (eg. IO)
-  fmuservo - handle talking to the auxillary PWM outputs on the FMU
-  uavcan - handle the uavcan CANBUS protocol

The startup of all of these tasks is controled by the PX4 specific
`rc.APM script <https://github.com/ArduPilot/ardupilot/blob/master/mk/PX4/ROMFS/init.d/rc.APM>`__.
That script is run when the PX4 boots, and is responsible for detecting
what sort of PX4 board we are using then loading the right tasks and
drivers for that board. It is a "nsh" script, which is similar to a
bourne shell script (though nsh is much more primitive).

As an exercise, try editing the rc.APM script and adding some sleep and
echo commands. Then upload a new firmware and connect to the debug
console while the board is booting. Your echo commands should show up on
the console.

Another very useful way of exploring the startup of the PX4 is to boot
without a microSD card in the slot. The `rcS script <https://github.com/ArduPilot/ardupilot/blob/master/mk/PX4/ROMFS/init.d/rcS>`__,
which runs just before rc.APM, detects if a microSD is inserted and
gives you a bare nsh console on the USB port if it isn't. You can then
manually run all the steps of rc.APM yourself on the USB console to
learn how it works.

Try the following exercise after booting a Pixhawk without a microSD
card and connecting to the USB console:

::

    tone_alarm stop
    uorb start
    mpu6000 start
    mpu6000 info
    mpu6000 test
    mount -t binfs /dev/null /bin
    ls /bin
    perf

Try playing with the other drivers. Have a look in /bin to see what is
available. The source code for most of these commands is in
`PX4Firmware/src/drivers <https://github.com/ArduPilot/PX4Firmware/tree/master/src/drivers>`__.
Have a look through the mpu6000 driver to get an idea of what is
involved.

Given we are on the topic of threads and tasks, a brief description of
threads in the PX4Firmware git tree is worth mentioning. If you look in
the mpu6000 driver you will see a line like this:

::

    hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&MPU6000::measure_trampoline, this);

that is the equivalent of the hal.scheduler->register_timer_process()
function in the AP_HAL, but is PX4 specific and is also much more
flexible. It says that it wants the HRT (high resolution timer)
subsystem of the PX4 to call the MPU6000::measure_trampoline function
every 1000 microseconds.

Using hrt_call_every() is the common method used for regular events in
drivers where the operations are very fast, such as SPI device drivers.
The operations are typically run with interrupts disabled, and should
take only a few tens of microseconds at most.

If you compare this to the hmc5883 driver, you will instead see a line
like this:

::

    work_queue(HPWORK, &_work, (worker_t)&HMC5883::cycle_trampoline, this, 1);

that uses an alternative mechanism for regular events which is suitable
for slower devices, such as I2C devices. What this does is add the
cycle_trampoline function to a work queue within the hpwork thread that
you saw above. Calls made within HPWORK workers should run with
interrupts enabled and may take up to a few hundred microseconds. For
tasks which will take longer than that the LPWORK work queue should be
used, which runs them in the lower priority lpwork thread.

The AP_Scheduler system
========================

The next aspect of ArduPilot threading and tasks to understand is the
AP_Scheduler system. The AP_Scheduler library is used to divide up
time within the main vehicle thread, while providing some simple
mechanisms to control how much time is used for each operation (called a
'task' in AP_Scheduler).

The way it works is that the loop() function for each vehicle
implementation contains some code that does this:

-  wait for a new IMU sample to arrive
-  call a set of tasks between each IMU sample

It is a table driven scheduler, and each vehicle type has a
AP_Scheduler::Task table. To learn how it works have a look at the
`AP_Scheduler/examples/Scheduler_test.cpp <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scheduler/examples/Scheduler_test/Scheduler_test.cpp>`__
sketch.

If you look inside that file you will see a small table with a set of 3
scheduling tasks. Associated with each task are two numbers. The table
looks like this:

::

    static const AP_Scheduler::Task scheduler_tasks[] PROGMEM = {
     { ins_update, 1, 1000 },
     { one_hz_print, 50, 1000 },
     { five_second_call, 250, 1800 },
    };

The first number after each function name is the call frequency, in
units controlled by the ins.init() call. For this example sketch the
ins.init() uses RATE_50HZ, so each scheduling step is 20ms. That means
the ins_update() call is made every 20ms, the one_hz_print() function
is called every 50 times (ie. once a second) and the
five_second_call() is called every 250 times (ie. once every 5
seconds).

The third number is the maximum time that the function is expected to
take. This is used to avoid making the call unless there is enough time
left in this scheduling run to run the function. When scheduler.run() is
called it is passed the amount of time (in microseconds) available for
running tasks, and if the worst case time for this task would mean it
wouldn't fit before that time runs out then it won't be called.

Another point to look at closely is the ins.wait_for_sample() call.
That is the "metronome" that drives the scheduling in ArduPilot. It
blocks execution of the main vehicle thread until a new IMU sample is
available. The time between IMU samples is controlled by the arguments
to the ins.init() call.

Note that tasks in AP_Scheduler tables must have the following
attributes:

-  they should never block (except for the ins.update() call)
-  they should never call sleep functions when flying (an autopilot,
   like a real pilot, should never sleep while flying)
-  they should have predictable worst case timing

You should now go and modify the Scheduler_test example and add in your
own tasks to run. Try adding tasks that do the following:

-  read the barometer
-  read the compass
-  read the GPS
-  update the AHRS and print the roll/pitch

Look at the example sketches for each library that you worked with
earlier in this tutorial to understand how to use each sensor library.

Semaphores
==========

When you have multiple threads (or timer callbacks) you need to ensure
that data structures shared by the two logical threads of execution are
updated in a way that prevents corruption. There are 3 principle ways of
doing this in ArduPilot - semaphores, lockless data structures and the
PX4 ORB.

AP_HAL Semaphores are just wrappers around whatever semaphore system is
available on the specific platform, and provide a simple mechanism for
mutual exclusion. For example, I2C drivers can ask for the I2C bus
semaphore to ensure that only one I2C device is used at a time.

Go and have a look at the hmc5843 driver in
libraries/AP_Compass/AP_Compass_HMC5843.cpp and look for the
\get_semaphore() call. Look at all the places it is used, and see if you
can work out why it is needed.

Lockless Data Structures
========================

The ArduPilot code also contains examples of using lockless data
structures to avoid the need for a semaphore. This can be a lot more
efficient than semaphores.

Two examples of lockless data structures in ArduPilot are:

-  the \_shared_data structure in
   libraries/AP_InertialSensor/AP_InertialSensor_MPU9250.cpp
-  the ring buffers used in numerous places. A good example is
   libraries/DataFlash/DataFlash_File.cpp

Go and have a look at these two examples, and prove to yourself that
they are safe for concurrent access. For DataFlash_File look at the use
of the \_writebuf_head and \_writebuf_tail variables.

It would be nice to create a generic ring buffer class which could be
used instead of the separate ringbuffer implementations in several
places in ArduPilot. If you want to contribute that then please do a
pull request!

The PX4 ORB
===========

Another example of this type of mechanism is the PX4 ORB. The ORB
(Object Request Broker) is a way of providing data from one part of the
system to another (eg. device driver -> vehicle code) using a
publish/subscribe model that is safe in a multi-threaded environment.

The ORB provides a nice mechanism for declaring structures which will be
shared in this way (all defined in
`PX4Firmware/src/modules/uORB/ <https://github.com/ArduPilot/PX4Firmware/tree/master/src/modules/uORB>`__).
Code can then "publish" data to one of these topics, which is picked up
by other pieces of code.

An example is the publication of actuator values so the uavcan ESCs can
be used on Pixhawk. Have a look at the \_publish_actuators() function
in AP_HAL_PX4/RCOutput.cpp. You will see that it advertises a
"actuator_direct" topic, which contains the speed desired for each ESC.
The uavcan code these watches for changes to this topic in
`PX4Firmware/src/modules/uavcan/uavcan_main.cpp <https://github.com/ArduPilot/PX4Firmware/blob/master/src/modules/uavcan/uavcan_main.cpp>`__\ and
outputs the new values to the uavcan ESCs.

Two other common mechanisms for communicating with PX4 drivers are:

-  ioctl calls (see the examples in AP_HAL_PX4/RCOutput.cpp)
-  /dev/xxx read/write calls (see \_timer_tick in
   AP_HAL_PX4/RCOutput.cpp)

Please talk to the ardupilot development team on the drones-discuss
mailing list if you are not sure which mechanism to use for new code.

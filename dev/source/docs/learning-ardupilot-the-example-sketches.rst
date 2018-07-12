.. _learning-ardupilot-the-example-sketches:

========================
Library Example Sketches
========================

The first step in exploring the code for yourself is to use the example
sketches for the libraries. Following the arduino tradition we have
example sketches for most libraries. A 'sketch' is just a main program,
written as a cpp file.

Knowing the library API and conventions used in ArduPilot is essential
to understanding the code. So using the library example sketches is a
great way to get started. As a start you should read, build and run the
example sketches for the following libraries:

-  libraries/AP_GPS/examples/GPS_AUTO_test
-  libraries/AP_InertialSensor/examples/INS_generic
-  libraries/AP_Compass/examples/AP_Compass_test
-  libraries/AP_Baro/examples/BARO_generic
-  libraries/AP_AHRS/examples/AHRS_Test

For example, the following will build and install the AP_GPS example
sketch on a Pixhawk:

::

    cd $ARDUPILOT_HOME # the top-level of an AruPilot repository
    ./waf configure --board=px4-v2
    ./waf build --target examples/INS_generic --upload

waf can list the examples it can build:

::

   cd $ARDUPILOT_HOME
   ./waf list | grep 'examples'

Once you have uploaded the example you can look at the output by
attaching to the console. What the console is depends on the type of
board. 
On Pixhawk boards it is the USB connector. 
So just connect to the USB device with your favourite serial program (the
baudrate doesn't matter).

For example, if you have mavproxy installed, you could do this to
connect to a Pixhawk on Linux:

::

    mavproxy.py --setup --master /dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00

Using the --setup option puts mavproxy into raw serial mode, instead of
processed MAVLink mode. That is what you need for the example sketches.

Understanding the example sketch code
=====================================

When you are reading the example sketch code (such as the
`GPS_AUTO_test <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_GPS/examples/GPS_AUTO_test/GPS_AUTO_test.cpp>`__
code) you will notice a few things that may seem strange at first:

-  it declares a 'hal' variable as a reference
-  the code is quite rough and not well commented
-  the setup() and loop() functions

The hal reference
-----------------

Every file that uses features of the AP_HAL needs to declare a hal
reference. That gives access to a AP_HAL::HAL object, which provides
access to all hardware specific functions, including things like
printing messages to the console, sleeping and talking to I2C and SPI
buses.

The actual hal variable is buried inside the board specific AP_HAL_XXX
libraries. The reference in each file just provides a convenient way to
get at the hal.

The most commonly used hal functions are:

-  hal.console->printf() to print strings
-  AP_HAL::millis() and AP_HAL::micros() to get the time
   since boot
-  hal.scheduler->delay() and hal.scheduler->delay_microseconds() to
   sleep for a short time
-  hal.gpio->pinMode(), hal.gpio->read() and hal.gpio->write() for
   accessing GPIO pins
-  I2C access via hal.i2c
-  SPI access via hal.spi

Go and have a look in the libraries/AP_HAL directory now for the full
list of functions available on the HAL.

The setup() and loop() functions
--------------------------------

You will notice that every sketch has a setup() function and loop()
function. The setup() function is called when the board boots. The
actual call comes from within the HAL for each board, so the main()
function is buried inside the HAL, which then calls setup() after board
specific startup is complete.

The setup() function is only called once, and typically initialises the
libraries, and maybe prints a "hello" banner to show it is starting.

After setup() is finished the loop() function is continuously called (by
the main code in the AP_HAL). The main work of the sketch is typically
in the loop() function.

Note that this setup()/loop() arrangement is only the tip of the iceberg
for more complex boards. It may make it seem that ArduPilot is single
threaded, but in fact there is a lot more going on underneath, and on
boards that have threading (such as Pixhawk and Linux based boards) there
will in fact be lots of realtime threads started. See the section on
understanding ArduPilot threading below.

The AP_HAL_MAIN() macro
-----------------------

You will notice a extra line like this at the bottom of every sketch:

::

    AP_HAL_MAIN();

That is a HAL macro that produces the necessary code to declare a C++
main function, along with any board level initialization code for the
HAL. You rarely have to worry about how it works, but if you are curious
you can look for the #define in the AP_HAL_XXX directories in each
HAL. It is usually in AP_HAL_XXX_Main.h.

Rough Example code
------------------

You will notice that the example sketches are quite rough, and badly
commented. This is your opportunity to make a contribution to the code!
As you read through the example sketches and explore how they work add
some comments to the code to explain the APIs and then :ref:`submit a pull request <submitting-patches-back-to-master>` so others can
benefit from your study.

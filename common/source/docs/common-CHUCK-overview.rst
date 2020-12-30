.. _common-CHUCK-overview:

======================================
C.H.U.C.K Autopilot (for Parrot Disco)
======================================

The heart of the Parrot Disco is the C.H.U.C.K autopilot, an orange
box which is a general purpose autopilot. It is perfectly possible to
use the C.H.U.C.K in a different airframe.

To use the C.H.U.C.K outside of a Disco the first thing you will
notice is it has 7 servo/motor outputs available. There are 6 PWM
3-pin servo connectors, and one connector from the I2C ESC that can be
used to drive a brushless motor (it drives the motor on the Disco).

The mapping between ArduPilot output channel numbers and the 7 outputs
was chosen with ease of integration with the Disco in mind, which
resulted in a fairly strange pin ordering.

Servo rail pin numbers in the list below are from left to right when
looking at the C.H.U.C.K from the back, so pin1 on the servo rail is
closest to the first 'C' in 'C.H.U.C.K' on the case.

* channel 1 : servo rail pin 1
* channel 2 : servo rail pin 6
* channel 3 : I2C ESC motor output
* channel 4 : servo rail pin 2
* channel 5 : servo rail pin 5
* channel 6 : servo rail pin 3
* channel 7 : servo rail pin 4

Note that this pin ordering means you will need some extra parameter
settings if you are not using the I2C ESC motor controller for
throttle output. To take the simple example of a 4 channel fixed wing
plane (aileron, elevator, throttle, rudder) you could configure and
wire it like this:

* servo rail pin1 : ailerons (on Y-lead if need be for 2 servos)
* servo rail pin6 : elevator
* servo rail pin5 : throttle
* servo rail pin2 : rudder
* set SERVO5_FUNCTION=70 or RC5_FUNCTION=70 to make channel 5 output throttle
* set SERVO5_MIN/RC5_MIN, SERVO5_MAX/RC5_MAX and SERVO5_TRIM/RC5_TRIM for the throttle range of your motor
  
Apart from that pin mapping, setting up a C.H.U.C.K with another
airframe is the same as with any aircraft with ArduPilot. It could be
used with any vehicle type supported by ArduPilot, including gliders,
quadplanes, petrol planes, multicopters or rovers.

For R/C input there is a 3-pin servo lead connector on the left side
of the C.H.U.C.K which accepts the following widely used R/C
protocols:

* SBUS
* DSM/Skektrum
* SUMD (Graupner)
* ST24 (Yuneec)

The use of the airspeed sensor and sonar will be dependent on the
physical shape of the airframe and whether the positioning of these
sensors is suitable for the layout of the C.H.U.C.K.

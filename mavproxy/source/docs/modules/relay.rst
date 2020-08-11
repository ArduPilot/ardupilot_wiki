================
Relay Management
================

.. code:: bash

    module load relay
    
Control the operation of the output servos. 

Note this should only be
used for the non-flight-control servos. An example of usage would be
camera shutter controls. Use ``servo set [SERVO_NUM] [PWM]`` to set an
output servo to a particular PWM value. Use
``servo repeat [SERVO_NUM] [PWM] [COUNT] [PERIOD]`` to make the servo
repeat between its current and specified PWM values.

If relay controls are available, the ``relay set [RELAY_NUM] [0|1]`` to
set a particular relay to 0 or 1. Similar to the servos, use
``relay repeat [RELAY_NUM] [COUNT] [PERIOD]`` to setup a repeating
relay.

For running motor tests (useful for multicopters), the ``motortest`` command
can be used:

.. code:: bash

    motortest motornum type value timeout count
    
Where the motor number ``motornum`` with type 0 (percent), 1 (PWM) or 2 (RC passthrough) 
and value depending on the type selection.

Timeout is the number of seconds the test should run for.

Count is an optional parameter specifying how many times the test should run for.



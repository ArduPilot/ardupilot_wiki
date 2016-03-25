.. _trad-heli-connecting-your-rc-input-servos-and-motors:

===============================================================
Traditional Helicopter - Connecting RC Input, Servos and Motors
===============================================================

Basic Configuration of your transmitter/receiver
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The basic radio set-up is very similar to the Multirotors. Although you
will be flying a helicopter, you should setup your radio in normal
airplane mode. Some radios call this "Acro" or "Fixed wing" mode. The
Copter electronics and software takes care of all needed swash plate
mixing.

As shown in the picture below you should connect your 8-channel receiver
to the APM's inputs. Alternatively you may use an :ref:`PPM-Sum receiver <common-connecting-the-radio-receiver-apm2_connecting_a_ppm_sum_receiver>`.

The APM2's input channel mapping is:

Channel 1: Aileron (aka Roll)

Channel 2: Elevator (aka Pitch)

Channel 3: Collective Pitch

Channel 4: Rudder (aka Yaw)

Channel 5: Flight Mode

Channel 6: In-flight tuning knob

Channel 7: Aux function switch (i.e. Camera trigger, Sonar on/off)

Channel 8: Main rotor speed

Powering the APM/Pixhawk
~~~~~~~~~~~~~~~~~~~~~~~~

-  If your main  battery is a 3S or 4S (i.e. under 18V) and the total
   current draw will be no more than 90amps it is highly recommended
   that you use the `APM2.5 Power unit with XT60 connectors <http://store.jdrones.com/APM25_PSU_XT60_p/pwrapm25x1.htm>`__. 
   If the voltage or current is higher than this, you will need a
   separate 5V BEC to power the APM/Pixhawk and receiver.
-  On the APM, remove the JP1 jumper to separate the output pin's power
   from the APM's CPU.  The servos will be powered from the main rotor's
   ESC (see below)

Connecting the servos
~~~~~~~~~~~~~~~~~~~~~

-  Outputs #1,#2 and #3 should be connected to the 3 swash plate
   servos.  The order is not critical although set-up is slightly easier
   later if you connect #1 to the front right servo, #2 to the front
   left servo and #3 to the swash plate's rear most servo.
-  Output 4 should be connected to the rudder servo.
-  Main rotor's ESC should be connected to Output #8.

.. image:: ../images/APM_RX_Serrvo_Connections.jpg
    :target: ../_images/APM_RX_Serrvo_Connections.jpg

Notes:

-  An external tail gyro is not required but if you choose to use one
   you should connect the gyro's gain wire to APM's output #7.  The
   gyro's main signal wire should connect to the APM's output #4 and of
   course the tail servo should be connected to the gyro itself.
-  If using a direct drive variable pitch tail the tail rotor's ESC
   should be connected to output #7

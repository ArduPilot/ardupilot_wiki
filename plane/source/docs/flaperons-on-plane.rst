.. _flaperons-on-plane:

======================
Flaperon Configuration
======================

`Flaperons <https://en.wikipedia.org/wiki/Flaperon>`__ use your 2
ailerons (using one channel each) as both flaps and ailerons. This
article shows how to set up Flaperons in Plane.

Input/Output Channels
=====================

A brief discussion on input and output channels may help.  The channels
from the transmitter in your hand, to the receiver on the vehicle, and
then into the autopilot on the vehicle are your INPUT channels.  The
channels from the autopilot to your servo's etc are the OUTPUT
channels. The input and output channels may not directly map to one
another.

Flaperons are the classic example of a setup where input and output
channels do not map directly.  The autopilot will use the input from
the aileron (INPUT channel 1) AND the input from the flap channel
(INPUT channel 5 - in the example below) and "mix" them to calculate
how the flaperons on the plane should move. The result is sent out to
each flaperon OUTPUT channel (channels 5 and 6 in the example below).

Flaperon setup
==============

-  Do not do any aileron mixing on your transmitter. 
-  INPUTS:

   -  Leave the standard aileron input on channel 1.
   -  You need to add an input channel on your transmitter to control
      the flaps. You can configure any unused input channel for this
      however we are going to use servo output channel 5.  Configure your transmitter to
      use Channel 5 for flaps (either a switch or a rotary button) and
      set ``FLAP_IN_CHANNEL`` to 5.
   -  Move your ailerons to 2 spare output channels on the autopilot
      that you aren't using.  In this example we are using channels 5
      and 6.

-  OUTPUTS:

   -  Set :ref:`SERVO5_FUNCTION <SERVO5_FUNCTION>` and :ref:`SERVO6_FUNCTION <SERVO6_FUNCTION>`
      to 24 and 25 (Flaperon 1 and flaperon 2 respectively - which
      channel is which does not matter).
   -  Check that
      :ref:`SERVO5_MIN <SERVO5_MIN>`, :ref:`SERVO5_MAX <SERVO5_MAX>`,
      :ref:`SERVO5_TRIM <SERVO5_TRIM>` has the
      correct range set if you haven't used them previously.  If your
      unsure usually 1000, 2000, 1500 will work fine.  Do the same for
      the SERVO6 equivalents.

-  Switch to FBWA or CRUISE.  Roll your plane back and forth and make
   sure the ailerons move in the correct direction (aileron goes down on
   the wing that you roll down). If they don't then use
   :ref:`SERVO5_REVERSED<SERVO5_REVERSED>` and :ref:`SERVO6_REVERSED<SERVO6_REVERSED>` to reverse channels as needed.
-  Once this works, try your flaps control on your transmitter and make
   sure flaps go down and not up. If they go the wrong way you will
   need to swap the two output channels and correct the reversals.
-  Confirm that when in FBWA and your roll the plane the ailerons move
   in the correct direction, and that your flaps go down.
-  Now try the ailerons stick on your transmitter. If they go the wrong
   way, you can use :ref:`RC1_REVERSED<RC1_REVERSED>` to change the direction of the input channel. If you
   put your stick left, the left aileron should go up.

Tuning
======

-  Go to failsafe setup in *APM Planner 2* or *Mission Planner*, and
   make sure the max/min values match ``SERVO5_MIN``/``SERVO5_MAX`` (or adjust
   them) so that your flaps move all the way ( :ref:`SERVO1_TRIM <SERVO1_TRIM>` should also be set to 1500). - Setting the :ref:`FLAP_SLEWRATE <FLAP_SLEWRATE>`
   to 100 allows moving flaps from 0 to 100% in one second.  Lower this
   to make your flaps move more slowly.
-  Adjust `FLAP_x_PERCNT|SPEED` as desired for auto modes - see `Automatic Flaps <automatic-flaps>`. Note you can ignore the
   comment on that page saying "parameter for the channel function for
   the channel you are using for flaps to a value of 3".
   ``FLAP_IN_CHANNEL`` is already set for this. - Have a look at :ref:`TKOFF_FLAP_PCNT <TKOFF_FLAP_PCNT>`
   and :ref:`LAND_FLAP_PERCNT <LAND_FLAP_PERCNT>` if they are relevant to you.
-  When you are flying in manual mode, it can be helpful to setup an
   elevator down mix on your TX when you set flaps i.e. the more flaps
   you send, the more elevator down you should send to correct pitch up
   from flaps. If possible set up the mix value on a rotary switch so
   that you can control the elevator down correction during a test
   flight.  If you set too much elevator down as a fixed value in your
   mix, you'll be stuck not being able to use flaps for landing if you
   put too much elevator down.

.. tip::

   Don't fly until you've rechecked that FBWA/CRUISE moves the
   ailerons in the right direction and that ailerons also go in the right
   direction in manual mode.

Crow flaperons
==============

You can also setup crow flaps (where the ailerons go up, not down) by
swapping the two output channels and setting the channel reversal as
needed.

Notes
=====

-  Manual flaps input is mixed into auto modes. That means if you're
   landing in manual mode with flaps set to full on your transmitter,
   and you flip the mode to RTL or some other mode to abort the landing
   and go back to an auto mode, flaps will stay full. You need to
   retract them on your transimitter.
-  ``SERVOx_MIN`` and ``SERVOx_MAX`` for Flaperon output channels limit
   deflection of Flaperons and you can use the TRIM value to move the
   neutral position in case you want more down travel than up travel.
-  ``SERVO1_TRIM`` acts as normal aileron trim.  ``SERVO1_MIN`` and
   ``SERVO1_MAX`` should match the transmitter setting

.. _flaperons-on-plane:

======================
Flaperon Configuration
======================

`Flaperons <https://en.wikipedia.org/wiki/Flaperon>`__ use your 2
ailerons (using one channel each) as both flaps and ailerons. This
article shows how to set up Flaperons in Plane.

.. note::

   This page was tested on Plane 3.3, but should be relevant to older
   versions. It was originally created from `this blog post <http://marc.merlins.org/perso/rc/post_2015-08-04_Using-Flaperons-With-Ardupilot.html>`__.

Input/Output Channels
=====================

A brief discussion on input and output channels may help.  The channels
from the transmitter in your hand, to the receiver on the vehicle, and
then into the autopilot on the vehicle are your INPUT channels.  The
channels from the autopilot to your servo's etc are the OUTPUT
channels. The input and output channels may not directly map to one
another.

Flaperons are the classic example of a setup where input and output
channels do not map directly.  The autopilot will use the input from the
aileron (INPUT channel 1) AND the input from the flap channel (INPUT
channel 5 - in the example below) and "mix" them to calculate how the
flaperons on the plane should move. The result is sent out to each flap
OUTPUT channel (channels 5 and 6 in the example below).

Flaperon setup
==============

-  Do not do any aileron mixing on your transmitter. 
-  INPUTS:

   -  Leave the standard aileron input on channel 1.
   -  You need to add an input channel on your transmitter to control
      the flaps. You can configure any unused input channel for this
      however we are going to use RC5.  Configure your transmitter to
      use Channel 5 for flaps (either a switch or a rotary button) and
      set :ref:`FLAP_IN_CHANNEL <FLAP_IN_CHANNEL>` to 5.
   -  Move your ailerons to 2 spare output channels on the autopilot
      that you aren't using.  In this example we are using channels 5
      and 6.

-  OUTPUTS:

   -  Set :ref:`RC5_FUNCTION <RC5_FUNCTION>` and :ref:`RC6_FUNCTION <RC6_FUNCTION>`
      to 24 and 25 (Flaperon 1 and flaperon 2 respectively - which
      channel is which does not matter).
   -  Check that
      :ref:`RC5_MIN <RC5_MIN>`, :ref:`RC5_MAX <RC5_MAX>`,
      :ref:`RC5_TRIM <RC5_TRIM>` has the
      correct range set if you haven't used them previously.  If your
      unsure usually 1000, 2000, 1500 will work fine.  Do the same for
      the RC6 equivalents.

-  Set :ref:`FLAPERON_OUTPUT <FLAPERON_OUTPUT>` to 1 initially.
-  Switch to FBWA or CRUISE.  Roll your plane back and forth and make
   sure the ailerons move in the correct direction (aileron goes down on
   the wing that you roll down). If they don't, try setting
   ``FLAPERON_OUTPUT`` to 4.
-  Once this works, try your flaps control on your transmitter and make
   sure flaps go down and not up. If they go the wrong way, change
   ``RC1_REV`` from 1 to -1 (or the other way around) and test again. 
   Once working go back and check ``FLAPERON_OUTPUT`` in the step above
   as it may also need to change again.
-  Confirm that when in FBWA and your roll the plane the ailerons move
   in the correct direction, and that your flaps go down.
-  Now try the ailerons stick on your transmitter. If they go the wrong
   way, reverse channel 1 on the transmitter ONLY.  Test again.  If you
   put your stick left, the left aileron should go up.

Tuning
======

-  Go to failsafe setup in *APM Planner 2* or *Mission Planner*, and
   make sure the max/min values match ``RC5_MIN``/``RC5_MAX`` (or adjust
   them) so that your flaps move all the way (`:ref:`RC1_TRIM`` should also be set to 1500). - Setting the :ref:`FLAP_SLEWRATE <FLAP_SLEWRATE>`
   to 100 allows moving flaps from 0 to 100% in one second.  Lower this
   to make your flaps move more slowly.
-  Adjust `FLAP_x_PERCNT|SPEED` as desired for auto modes - see `Automatic Flaps <automatic-flaps>`. Note you can ignore the
   comment on that page saying "parameter for the channel function for
   the channel you are using for flaps to a value of 3".
   :ref:`FLAP_IN_CHANNEL <FLAP_IN_CHANNEL>` is already set for this. - Have a look at :ref:`TKOFF_FLAP_PCNT <TKOFF_FLAP_PCNT>`
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

If you need Crow flaps (i.e if your ailerons must go up, not down), you
can use these instructions and reverse ``RC1_REV`` so that when you send
flaps input, ailerons go up instead of down. Then you should be able to
set your flaps channels as :ref:`FLAP <channel-output-functions_flap>` or
:ref:`FLAP_AUTO <channel-output-functions_flap_auto>`.

See :ref:`How would I setup crow flaps? <fixed-wing-faq_how_would_i_setup_crow_flaps>` (Fixed Wing
FAQ) for more information.

Notes
=====

-  Manual flaps input is mixed into auto modes. That means if you're
   landing in manual mode with flaps set to full on your transmitter,
   and you flip the mode to RTL or some other mode to abort the landing
   and go back to an auto mode, flaps will stay full. You need to
   retract them on your transimitter.
-  ``RCx_MIN`` and ``RCx_MAX`` for Flaperon output channels limit
   deflection of Flaperons and you can use the TRIM value to move the
   neutral position in case you want more down travel than up travel.
-  ``RC1_TRIM`` acts as normal aileron trim.  ``RC1_MIN`` and
   ``RC1_MAX`` should match the transmitter setting

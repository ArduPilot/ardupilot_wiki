.. _guide-flaperons:

=========
Flaperons
=========

Flaperons provide a way for an to incorporate the functionality of
flaps into ailerons. You setup flaperons by connecting separate servo
output channels to your left and right aileron servos.

Before starting with flaperons, you should first read the guide on
:ref:`four channel planes <guide-four-channel-plane>`. Once you have
carefully read through that guide, come back to the rest of this page.

Setting Up Your Flaperons
=========================

You need to use the SERVOn_FUNCTION parameters to set the function of
your left flaperon to 24 and the function of your right flaperon
to 25.

.. raw:: html

   <table border="1" class="docutils">
   <tr><th>Parameter</th><th>Value</th><th>Meaning</th></tr>
   <tr><td>SERVO5_FUNCTION</td><td>24</td><td>left flaperon</td></tr>
   <tr><td>SERVO6_FUNCTION</td><td>25</td><td>right flaperon</td></tr>
   </table>

Now you need to setup an input channel to control the flaps, to give
yourself a way to test the flaps. Choose an unused input channel on
your transmitter, and assign it to a knob or switch. If you chose
channel 6 as the input channel then you would set the parameter
FLAP_IN_CHANNEL=6.

Now test your aileron and flap functionality. Make sure that the
ailerons move in the right direction. You can swap the two flaperon
channels or set SERVOn_REVERSED as need to get the ailerons moving
correctly.

Then test your flap functionality using your transmitter to control
the flap input channel.

Next you should read the page on :ref:`automatic flap deployment <automatic-flaps>`.

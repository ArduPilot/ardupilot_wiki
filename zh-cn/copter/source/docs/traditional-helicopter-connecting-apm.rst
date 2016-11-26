.. _traditional-helicopter-connecting-apm:

===========================================
Traditional Helicopter — Connecting the APM
===========================================

Transmitter programming versus APM programming
==============================================

The APM does a lot of processing internally to fly your heli while the
Tx stick inputs are only \ *pilot requests* which the APM uses to figure
out what to do with the servos. Putting radio mixes into the APM, or
trying to run an ESC manually goes against the whole idea that the
APM \ **is in control**. You can use some \ **expo** , or possibly
dual-rates, in your Tx but that's it, everything else must be done
inside the APM.

Basic configuration of your receiver and servos
===============================================

The basic radio set-up is very similar to
the \ :ref:`Multirotors <assembly-instructions>`. Although you will be
flying a helicopter, the setup is easier if you do not use your RC
transmitter in helicopter mode. Instead, setup your radio in normal
airplane mode. Some radios call this "Acro" or "Fixed wing" mode. The
Copter electronics and software takes care of all needed swash plate
mixing.

Differences compared to the Multirotor set-up are:

-  APM Output Channels 1~4 are connected to servos and the servos draw
   significant power from the power rails. Ensure your ESC and/or
   voltage regulator can provide enough power for the APM electronics
   and 4 power hungry servos.
-  APM \ **Output** Channel 7 can be used as the (optional) external
   gyro gain
-  APM Channel 8 can be used for the motor throttle using the Rotor
   Speed Control (RSC) function. This requires the rotor control signal
   from the Tx to be sent to the Input Ch8, and the ESC or governor to
   be plugged in to the Output Ch8.

.. note::

   Check that your ESC outputs 5V (and not 6V!). If it's 6V you will
   need to place a 5V regulator between the ESC and receiver (this is
   required because although servos are happy with 6V, you risk burning out
   the APM if you power them with 6V).

Connect up your Radio and ArduPilot Mega generally as shown in the
diagram below for a Futaba radio system.

.. image:: ../images/RX-to-AMP-Connection.jpg
    :target: ../_images/RX-to-AMP-Connection.jpg

Note the \ **very important** cross-overs: Rx Channel 6 is connected to
APM Channel 3 and Rx Channel 3 is connected to APM Channel 8.

Here is a comprehensive connection table for a Futaba Rx:

+-----------+-------------+----------------------------------------------+
| Channel   | Futaba Rx   | AMP2.5 Input                                 |
+-----------+-------------+----------------------------------------------+
| 1         | Aileron     | Aileron                                      |
+-----------+-------------+----------------------------------------------+
| 2         | Elevator    | Elevator                                     |
+-----------+-------------+----------------------------------------------+
| 3         | Throttle    | Collective Pitch (i.e. swash plate height)   |
+-----------+-------------+----------------------------------------------+
| 4         | Rudder      | Rudders                                      |
+-----------+-------------+----------------------------------------------+
| 5         | Aux1        | Aux1                                         |
+-----------+-------------+----------------------------------------------+
| 6         | Pitch       | Aux2                                         |
+-----------+-------------+----------------------------------------------+
| 7         | Aux2        | Aux3                                         |
+-----------+-------------+----------------------------------------------+
| 8         | Aux3        | Throttle                                     |
+-----------+-------------+----------------------------------------------+

NB: We need info on channel settings for other radio systems! Some
systems route different functions to different channels, and have little
flexibility to move them within the radio program. In this case, you
will need to make different channel cross-overs to suit your system.
What is important is to match up the correct control function to the
correct APM input!

For example, the Futaba 9CAP/CHP outputs the Collective Pitch control on
Channel 6 so you need to connect that to input 3 on the APM. The
throttle/ESC control is on channel 3 of the Rx, so that needs to be
connected directly to the ESC or throttle servo or in v2.9.1 of the
software to Channel 8. Please consult your radio manual to determine
which function is routed on which Rx channel.

The diagram below might be useful in trying to understand how to connect
your particular receiver to the APM. It has generic names for the inputs
which might help you relate them to your system.

.. image:: ../images/APM_2_5_PINOUT-Revised.jpg
    :target: ../_images/APM_2_5_PINOUT-Revised.jpg

If you can, it’s best to use an 8 channel receiver because the APM2.5
has 8 input channels all of which have important functions.

APM Channel 5 (Rx Ch5): Aux 1
-----------------------------

If you don't understand what \ **Mission Planner** is then this section
may be worth returning to once you have that software loaded and you've
familiarised yourself with it.

Everything you need to know about downloading Mission Planner for use
with your Apm2.5+ can be :ref:`found here! <common-install-mission-planner>`.

Ch 5 should be set up using a 3 position switch with possible mixing
added later to get more than three outputs and this is used to select
the mode you want to fly. There are 12 modes from 0 to 11.

0=stab 1=acro 2=alt_hold 3=auto 4=guided 5=loiter 6=RTL 7=circle
8=pos_Hold 9=land 10=of_loiter 11=Toy

Please note that of_loiter is for the optical flow sensor. Don't use it
if you don't have one.

You set the modes you want using Channel 5 on your Tx and can find much
more information on this \ :ref:`here! <flight-modes>`.

APM Channel 6 and 7 (Rx Ch7 and 8): Aux 2 and Aux 3
---------------------------------------------------

Please refer back to the layout diagram above to remind yourself of the
wiring of Rx Ch7 and Ch8 to APM channel 6 and 7.

In Mission Planner go to Configuration Tab>Standard Params>Copter Config
and you will see at the bottom-middle of the screen a \ **Ch6 Opt**\ and
a \ **Ch7 Opt** pull down menu. It is best to set Ch7 in your Tx up with
a knob or slider. You can then choose a parameter you want to test over
a range from the drop down menu and set the Min and Max values just
below the Ch6 Opt drop down to the extreme values of the parameter range
you want to test.

Ch8 in your Tx should be set up with a two position switch and you can
activate the function in the drop down menu of the \ **Ch7 Opt** with
this switch. Have a look at the Mission Planner options for
Ch7 \ :ref:`here! <channel-7-and-8-options>`

Rotor Speed Control (RSC) using Output Channel 8 on APM2.5
----------------------------------------------------------

To use the APM to control the rotor speed (ie.throttle/ESC), all you
have to do is send whatever throttle signal you want in on Ch8(in) and
then plug the throttle servo or ESC into Ch8(out) on the APM. Ch8
throttle control is important because it forces you to arm the APM
before you can fly. Without arming, the motor will not start nor will
the collective servos work. So Ch8 is used for switching the
motor/collective on and off something like a throttle-hold. See a
detailed explanation of the RSC set
up \ :ref:`here! <traditional-helicopter-configuration>` which is part of
the Configuration section of this Wiki.

Updating the PPM encoder on the APM2.5+
=======================================

If you are using an 8 channel Rx it is important to have the latest
version of the APM PPM encoder firmware iinstalled on the APM board. If
you are having difficulties with it, information on upgrading the PPM
encoder can be found on the Copter wiki.

+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| **NOTE TO FUTABA USERS:**                                                                                                                                                                                          |
+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| If you experience a "twitch" while flying, it is likely because of an issue where the Futaba Rx is not working well with the PPM Encoder firmware. Please be sure your PPM Encoder firmware is properly updated.   |
+--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+

External Gyro
=============

An external gyro is not required with v2.9.1 of the Copter software and
the external gyro performance is unknown and has not been tested. We
don't recommend using an external gyro and in time support for it will
be removed.

Advanced Configuration of your Receiver and Servos
==================================================

Copter can be configured to function properly with your radio set up
with a helicopter profile. The reason for using this setup is to gain
the benefit of helicopter functions pre-programmed into your radio, such
as Idle-Ups and Throttle Hold. It is mandatory to set your swash plate
to a non-CCPM mixing type. This is because the Copter program performs
the CCPM mixing internally, and it must see discreet radio commands for
Aileron, Elevator, Rudder and Collective Pitch.

The full details of this setup is outside the scope of this Wiki page.
Chances are if you have been flying RC helicopters for a while, you know
exactly what you need to do already.

With a Futaba radio this would be an H-1 or SW1 depending on your radio
model. Other brands of radio will use a different nomenclature such as
NOR, NORM, or 1SERVO. It is critical that CCPM mixing is turned off,
please consult your radio manual to ensure you have it set up correctly.

**Please note** that many of the helicopter mixing functions in your
radio (such as Revo Mixing) need to be turned off. This is why this
setup is only recommended for advanced users.

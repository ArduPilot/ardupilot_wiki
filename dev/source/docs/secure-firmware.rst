.. _secure-firmware:

=============================
Creating Tamperproof Firmware
=============================

It is possible to create highly tamper-proof firmware for your vehicles using signed bootloaders and signed firmware. Once loaded, only firmware signed with one of the public-private key pairs will run on the autopilot, preventing unsigned firmware from being loaded and run. Only firmware created with your private key will run (ArduPilot also has its own private key, that would allow only a selected senior ArduPilot developer to provide "rescue" services to our partners in emergencies.)

.. warning:: It is possible to render the autopilot inoperative if the procedures below are not followed exactly!

The information on signing bootloaders and firmware is provided in a README file at https://github.com/ArduPilot/ardupilot/tree/master/Tools/scripts/signing

A video is also provided:

.. youtube:: KO700k-SxzU


Overview of Steps
=================

Study the above video and README!

#. Make sure you have a :ref:`build environment<building-the-code>` setup and working for normal firmware builds.
#. Make sure you have the :ref:`latest MAVProxy<mavproxy-downloadinstall>` installed.
#. Create a branch to develop the firmware locally.
#. Create your public-private key pairs (normally up to two are allowed for your use).
#. Build a securely signed bootloader for the autopilot.
#. Build a securely signed firmware and load it onto the autopilot.
#. Use MAVProxy to flash the securely signed bootloader contained in the firmware you just loaded as the new bootloader.
#. Verify that the new secure bootloader has been installed.

At this point only securely signed firmware built using one of the key pairs will boot and run on the autopilot.

Reverting to UnSigned Bootloader
================================

Instructions are provided in above referenced README.md file for reverting to a normal bootloader using a private key whose corresponding public key is included in the bootloader.


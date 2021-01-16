.. _common-when-problems-arise:

===================
When Problems Arise
===================

ArduPilot, is extremely capable and flexible. But with high performance and flexibility comes a lot of configurations, parameters, and complexity.

This WIKI documentation attempts to reduce the effort of configuring and operating your ArduPilot based vehicle by providing as much accurate information on configuration, parameters, and operating modes as possible, and is continuously being updated as new releases occur or areas needing further explanation arise. Your assistance in that effort is welcomed and solicited. See :ref:`common_wiki_editing_guide`

What to do if you have an issue
===============================

1. Make sure you have followed the vehicle's "First Time Setup" and "First Flight/Drive" Sections and carefully read the provided documentation. If dealing with advanced configuration or hardware options, thoroughly read the appropriate documentation.

2. If this does not help you resolve the issue, then seek help on the `Discuss Forum <https://discuss.ardupilot.org/>`__ section appropriate to your vehicle or ground station. Do not enter issues on the GitHub software repositories or Gitter Developer Channels unless it has been confirmed as an actual issue in the code or documentation. Support will be given in the appropriate Discuss Forum.

3. Having a :Ref:`dataflash log <common-diagnosing-problems-using-logs>` will help you, or someone helping you, to diagnose the issue.

.. note:: WatchDog resets ("WDG:") should be reported `on this page <https://github.com/ArduPilot/ardupilot/issues/15915>`_ , Internal Errors ("Internal Error:") should be reported `here <https://github.com/ArduPilot/ardupilot/issues/15916>`_


[site wiki="copter"]


Copter Common Problems
======================

-  new copter flips immediately upon take-off.  This is usually caused
   by the motor order being incorrect or spinning in the wrong direction
   or using an incorrect propeller (clockwise vs counter-clockwise). 
   Check the rc connections for your autopilot.
-  copter wobbles on roll or pitch axis.  This usually means the Rate P
   values are incorrect.  See :ref:`common-tuning` section for some hints as to
   how to adjust these gains.
-  copter wobbles when descending quickly.  This is caused by the copter
   falling through its own prop wash and is nearly impossible to  tune
   out although raising the Rate Roll/Pitch P values may help.
-  copter yaws right or left 15degrees on take-off.  Some motors may not
   be straight or the :ref:`ESCs have not been calibrated <esc-calibration>`.
-  copter always tends to fly in one direction even in a windless
   environment.  Try :ref:`SaveTrim or AutoTrim <autotrim>` to level the
   copter.
-  copter climbs rapidly even if the pilot pulls the throttle down. This is likely caused by high vibrations. See https://ardupilot.org/copter/docs/common-vibration-damping.html for methods to improve vibration isolation.
-  occasional twitches in roll or pitch.  Normally caused by some kind
   of interference on the receiver (for example FPV equipment placed too
   close to the receiver) or by ESC problems that may be resolved by
   :ref:`calibrating them <esc-calibration>`.
-  sudden flips during flight.  This is nearly always caused by
   :ref:`mechanical failures <common-diagnosing-problems-using-logs_mechanical_failures>`
   of the motor or ESCs.

[/site]

H7 AutoPilot Will Not Initialize
================================

AutoPilots utilizing the H7 series of processors can, on rare occasions, get into a state where they will no longer complete initialization. Symptoms are: never exiting the bootloader (rapidly flashing led right after power application never stops) or the autopilot freezes during initialization, and connection to it is impossible. 

It is believed that this may be a memory corruption problem which can be caused by interrupting a flash memory write (as when changing parameters). Unfortunately, due to the processor's architecture, there is no way in the firmware to correct this automatically. If the autopilot  seems "bricked", try this to completely reset the autopilot to a fully un-programmed state. This should allow the firmware to be installed and the corruption issue resolved.

- First, program the entire 2MB flash space with zeros by loading `this file <https://firmware.ardupilot.org/Tools/STM32-tools/2MByte_allzero.bin>`_ which contains all zero data. Use the instructions :ref:`here <common-loading-firmware-onto-chibios-only-boards>` but use the above file.

- Next, download the ArduPilot bootloader for your AutoPilot from `here <https://firmware.ardupilot.org/Tools/Bootloaders/>`__. Then repeat the above step using that bootloader file. This will place the bootloader on the autopilot. Cycle the power on the autopilot. At this point it will power up and remain in the bootloader until operational firmware is installed.

- Finally, use Mission Planner's SETUP/Install Firmware tab or the `Uploader <https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/scripts/uploader.py>`__ python script, to load the desired ArduPilot firmware revision. 

This should resolve issues caused by memory corruption and normal operation will resume.
.. _basic-setup:

===============
Basic FPV Plane
===============


Introduction
------------

With ArduPilot (AP) it is possible to use a fully functional, powerful autonomous flight system on inexpensive, highly
integrated autopilot boards, including onboard battery monitoring and an onboard OSD. Due to ArduPilot's 
multitude of options and parameters, a first time setup may seem daunting. 
This guide attempts to provide the necessary instructions to get airborn on a typical ArduPlane setup with autonomous 
flight fleatures and is intentionally focussing only a small part of what ArduPilot has to offer. 
For a more in-depth explanation of flight features see the `full setup guide <arduplane-setup.html>`__.
ArduPilot's additional features and options (terrain following, airspeed use, autoland features, logging,
compass setup etc.) are left for you to pursue in the future. The goal in here is to get you in the air,
mission capable, with a well flying plane that you can expand on.


Hardware
--------

There are many choices and options for ArduPilot FCs, see `this list of currently supported boards <common-autopilots.html>`__. 
Omnibus F4 Pro or Matek F405-Wing are good choices for a basic FPV plane
configuration, since they integrate OSD and power module functions into one board. This guide will assume a compareable integrated peripheral setup containing onboard battery monitoring and OSD. Additionally, the following items are required:

-  GPS
-  RX
-  ESC & Servos
-  VTX and Camera


This guide presumes you are capable to correctly wire all components using the readily available
resources for the selected board. RCGroups provides forums for hardware help on any of the supported boards.
Other peripherals (compass, pitot, telemetry, etc.) will not be covered here as they are not necessarily
required to get you in the air and flying. They can well be added and configured in the future, subject to personal preferences.


Software
--------

This is the part that likely gives potential new users the most concern, that's why this guide
will focus this.

-  Firmware

Commercially targeted boards like Pixhawk will usually already have some version of ArduPilot loaded, along with an 
ArduPilot compatible bootloader that will allow updating the firmware using Mission Planner or other GCS software (ground control station) immediately.
Boards like Omnibus F4 Pro usually don't come with an ArduPilot bootloader by default, so you will need to
follow `this straight forward procedure to load ArduPilot on your board for the first time <common-loading-firmware-onto-chibios-only-boards.html>`__.

-  Mission Planner

Mission Planner (MP) is a ground control station for AP, but also, the primary tool for firmware updates and, most importantly, parameters configuration. MP can be downloaded here:
https://ardupilot.org/planner/docs/mission-planner-overview.html

There are alternatives,for non-PC users:
https://ardupilot.org/planner2/index.html

Once the firmware is installed and you are connected to MP or its alternatives, software setup can be begin. 
Parameters may be changed manually using the Configuration / Tuning Tabs.

Note that some parameters can only be set while dis-armed, and that many parameter groups are not visible until enabled and refreshed, such as the RSSI and OSD parameter groups.

Setup
-----

-  GPS

After connecting to MP, verify that your GPS is recognized and you eventually get a satellite fix. Number of sats and
HDOP are shown in the Flight Data tab of MP. If no gps is detected, you will continue to see a message in the HUD display stating "No GPS". If it detects it, it will initially show "GPS: No Fix" until it obtains a lock.

That means you have hardware debug to do!

Possible issues:

1. Connected to wrong serial port (see your hardware docs for which port AP expects) or incorrectly connected (TX/RX swapped).
2. Serial port settings were changed from default. ArduPlane expects GPS on Serial 3 at 38.4Kbaud per default.
3. UBLOX, NMEA, and many other GPS receivers are supported by ArduPilot, but your GPS receiver is not currently supported.

.. Note::
    The board's hardware UART number may NOT necessarily be the same as the SERIAL port number in ArduPilot firmware / MP.
    See your respective board's `hardware page <common-autopilots.html>`__ for wiring guides and UART to SERIAL mapping. GPS defaults
    to SERIAL3 in the parameter listing and you should not have to worry about UART mapping if you attach the GPS to the port
    suggested in the wiring guide.

-  Radio Calibration

Go to SETUP and move to Mandatory Hardware's Radio Calibration tab. You should see
the respective bars respond accordingly to your TX stick movements for each input (pitch is opposite). 
If not, you have hardware debug to do! 
ArduPilot expects AETR on channels 1-4 by default. Adjust your TX channel assignments accordingly or alternatively
use the RCMAP_n parameters. You CAN change the direction of each input (ie reverse the channel) with a check
box in MP, if you cannot change it in your TX. Adjust your TX subtrims to 1500uS for neutral as close as possible, perfection is not required. The next step will compensate for residual offsets. However, set the throttle trim such that
at low throttle, it is 50-75uS GREATER than its lowest value possible. This will be used to
allow you to setup a throttle failsafe value lower than your throttle stick's low position. After this, NEVER adjust your
TX trims or subtrims again, all further trim procederes will be performed on your FC exclusively!

Follow the RC Calibration steps as prompted.

-  Flight Modes

Now setup your TX to provide the ability to output six different levels on Channel 8. This will be your flight mode channel.
(the default flight mode channel can be adjusted by the :ref:`FLTMODE_CH<FLTMODE_CH>` param, it defaults to channel 8). 
See setup instructions `for various transmitters <common-rc-transmitter-flight-mode-configuration.html>`__.
Now that the TX can select 6 modes (usually using combining two three-position-switches), set your flight modes for each switch position.
It is strongly recommended to initally use two positions for MANUAL mode, allowing you to "bailout" to MANUAL no matter what
position the second switch is in. It's a lot easier to remember to just slam one switch down( or up)
without worrying about the state of the other mode switch. Then set the TX switches to produce the following modes:

SW1  -  SW2  -  Mode
  
Hi   -  Hi   -  Auto
  
Hi   -  Mid  -  AutoTune (then Loiter after autotuning)
  
Hi   -  Lo   -  Manual
  
Lo   -  Hi   -  Cruise
  
Lo   -  Mid  -  Stabilize (or FBWA)
  
Lo   -  Lo   -  Manual (low on SW2 ALWAYS drives Manual - that's your bailout!)

Set these up in the Flight Modes subtab under SETUP -> Mandatory Hardware.

After initial flights and tuning, you can change modes to whatever are desired.

-  Accel Calibration

Go to the SETUP -> Mandatory Hardware -> Accelerometer Calibration tab and
perform the full calibration. For the level position carefully level the wings laterally, and have the wing chord 
set a few degrees (~ 3 deg) nose up, since this is the normal cruise attitude for level flight for most planes.
This can be reset using the LEVEL only calibration button at any time. Also check that the orientation of the 
autopilot is correct. Moving the plane should be correctly reflected in the HUD display of MP. 
Otherwise, you will need to manually change the :ref:`AHRS_ORIENTATION<AHRS_ORIENTATION>` parameter appropriately.

-  Servo Functions

Now configure the outputs of the FC to drive the servos and your ESC. This is done using the
SERVOx_FUNCTION params for however you want to connect your equipment to the FC
outputs: normal plane, v-tail, elevons, etc.
You should try to get the servo-arms centered when the channel driving it is in neutral
position if possible. You can tweak this using the SERVOx_TRIM params. You can do this
manually or via the SETUP -> Mandatory Hardware -> Servo tab. You can also set the
output range in this tab. Default is 1100 to 1900uS. Depending on your individual airframe's requirements, these endpoints 
can be adjusted. Values of 1000/2000uS usually provide full throws.

Now check that the servos move in the correct directions to level the plane when moved around while in STAB or FBWA mode. 
Reverse the sevos with the reverse box for a servo if needed. If you have correctly adjusted the TX channel directions in part B above, then they will move in the correct directions also in MANUAL mode. 

-  OSD

For autopilots with integrated OSDs, this should be automatically enabled. You
can use the Onboard OSD subtab in MP CONFIG tab to setup the display configuration.
Note that the elements of each OSD screen are visible only after having enabled that screen
and rebooting/reconnecting to the FC.

-  Receiver RSSI

ArduPilot supports either analog RSSI or PWM RSSI (also known as "Digital RSSI") embedded in an RC channel. For
analog (voltage-type), set :ref:`RSSI_TYPE<RSSI_TYPE>` =1 and reboot, for PWM set :ref:`RSSI_TYPE<RSSI_TYPE>` =2. The remaining RSSI
parameters will not appear until this parameter is set and saved and your FC rebooted.

    :ref:`RSSI_ANA_PIN<RSSI_ANA_PIN>` =x (see :ref:`this page for details <common-rssi-received-signal-strength-indication>`)
    
    :ref:`RSSI_PIN_HIGH<RSSI_PIN_HIGH>` =3.3 (Note that most RSSI input pins are tolerant only up to 3.3V)
    
    :ref:`RSSI_PIN_LOW<RSSI_PIN_LOW>` =0

For PWM adjust these parameters:

    :ref:`RSSI_CHAN_HIGH<RSSI_CHAN_HIGH>` =2000
    
    :ref:`RSSI_CHAN_LOW<RSSI_CHAN_LOW>` =1000
    
    :ref:`RSSI_CHANNEL<RSSI_CHANNEL>` =<channel that has RSSI pwm>

.. Note:: 
    RSSI can be monitored in MP by adding the user defined screen item in the HUD view (right click): rxrssi.
    
- Failsafe

ArduPilot has many options for initiating failsafe. The only one of concern for this guide
is receiver failsafe. Failsafe will always be entered if the RC input signal to the FC is lost. 
In addition, for Sbus-type receivers, radio failsafe will be triggered by specific failsafe bits sent in the Sbus output stream. For PPM-type receivers, you set the RX's failsafe to use either throttle failsafe or no signal output ( DON'T USE HOLD!). Setting up throttle failsafe is described `here <apms-failsafe-function.html>`__.

The option to activate throttle failsafe by a separate TX switch that forces the throttle channel to the failsafe value avoids having to set RTH as an additional flight mode and is a good way to check faislafe behaviour while flying.

On the bench while connected to MP and with propellor removed, make sure that FS is entered when you power off the TX.

-  Battery Monitor

Do NOT use the SETUP tab to setup the Battery Monitor for the newer
Chibios boards. These boards have the default configuration already loaded when installing
the firmware.
You will probably have to slightly adjust the :ref:`BATT_AMP_OFFSET<BATT_AMP_OFFSET>` and :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>`
parameters. Most systems will draw 400 to 600 ma when the motor is not running. This is set
with the :ref:`BATT_AMP_OFFSET<BATT_AMP_OFFSET>` parameter. You can adjust the :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` parameter to match the capacity used 
during your flight by taking the amount of current you recharge the battery with, and the displayed amount of mah used 
using the following formula:

    new :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` = old :ref:`BATT_AMP_PERVLT<BATT_AMP_PERVLT>` * MAH -recharged/ MAH shown as used.

.. Note:: 
    this isn't 100% accurate due to several factors, but is close enough. You may have to iterate a few times. 
    Also be sure to set the :ref:`BATT_CAPACITY<BATT_CAPACITY>` parameter.

- Compass

Tradition fixed wing Arduplane does not need a compass for good performance,  as opposed to Copter or
Quadplane which require a compass for yaw alignment. Even if you have a compass, disable it until you have
everything else working. Then you can expand to it. Uncheck "Use this compass" for every compass in Mission Planner's SETUP/Compass screen.

- Airspeed

Arduplane does not need an airspeed sensor for basic performance. A fairly accurate synthetic airspeed estimate is calculated and gives
good basic performance. In order to display this in the OSD, you will need to set :ref:`ARSPD_TYPE<ARSPD_TYPE>` =0. Feel free to add/enable a pitot sensor later to improve cruise flight target airspeed precision, or automatic landing airspeed control.

- Other Parameters


1. Set :ref:`SERVO_AUTO_TRIM<SERVO_AUTO_TRIM>` =1. This will automatically adjust your servo trims as required for level flight. Unless you have your mechanical trims WAY OFF, this eliminates the need to manually trim the plane. You should never trim using the TX trims, as stated above.

2. If you have an overpowered plane, you might want to set :ref:`THR_MAX<THR_MAX>` to value lower than 100%, ie 75%. Otherwise climbs will be performed at max throttle, like during AUTOTAKEOFFs.


3. Set :ref:`TRIM_THROTTLE<TRIM_THROTTLE>` to the expected cruise throttle. Usually a little below midthrottle.

4. If you have a small flying wing (like Z-84), it might be required to decrease your default :ref:`PTCH2SRV_P<PTCH2SRV_P>` value if it is too aggressive and causes flutter. In that case, reduce the default by half.


5. ARMING: Leave all arming parameters at default. There is no reason to disable these safety checks. You should be able to get a GPS lock even indoors with modern GPS units. Inability to arm due to one of these checks failing means something has to be corrected. This adds noticeable safety by keeping you from accidentally starting your flight without your autopilot being in a fully functional state.

All other parameters can be left to default. However, after you get some flights, you might want to play with:
:ref:`LIM_PITCH_MAX<LIM_PITCH_MAX>` , :ref:`LIM_ROLL_CD<LIM_ROLL_CD>` , and :ref:`FBWB_CLIMB_RATE<FBWB_CLIMB_RATE>` . These are pretty docile at default values.

-  ESC Calibration

Calibration is easy. Remove prop. Power up on the bench with radio on using USB.
When GPS is locked, ARM the plane by giving full right rudder for a few seconds and move 
throttle to high. Attach battery. When ESC beeps its throttle set sequence, lower the throttle.
Disconnect power, and re-attach prop.
If it does not arm, something has not been setup correctly above, or (if you have an SD
card for logging) the SD card is not inserted. Diagnostic messages will be displayed on the
OSD and in MP messages tab.

First Flight
------------

First, go to the flight planner page of MP and create a waypoint anywhere, set it to TAKEOFF type. 
Set it for 100 to 150ft altitude and pitch of no more than 15deg for the first flight. Write it to the FC.
Read back to make sure it has been stored correctly.
At the field, power the plane, check again that all control surfaces move correctly accordingly to your TX stick input in
MANUAL mode, AND that when switched into STAB mode, the surfaces move correctly to level the plane when you move it around.

.. Warning::
    THIS IS CRITICAL! Flying with control surface movement setup incorrectly will result in a crash!

Also recheck your battery is properly placed to for the desired CG. ARM the plane and get ready to launch it. 
Switch into AUTO mode. Now the TAKEOFF command will be activated and the plane will go to :ref:`THR_MAX<THR_MAX>` even though the throttle stick is at idle. Toss it and it will climb straight up to desired altitude. It will then go into RTL since no other waypoint is loaded.
Be sure to move the throttle stick from idle to midstick after launch to avoid unexpected
throttle idle if you have to switch to STAB or MANUAL for some reason. Also be prepared for
another :ref:`THR_MAX<THR_MAX>` climb to the RTL altitude (ALT_HOLD_RTL) if your TAKEOFF altitude is below this.

Now switch to CRUISE mode and let the airframe cruise level without input for several intervals of ten seconds.
This allows the :ref:`SERVO_AUTO_TRIM<SERVO_AUTO_TRIM>` function to adjust the servo trims accordingly. After having performed level flight with no inputs for a sufficiently long accumulated periods (trim is updated every 10 seconds of flight with no pilot inputs), switch into Manual mode to verify correct trims are now set.

Then check all the other flight modes one by one. Check FS behaviour also.

At this point the plane should be flyable and well trimmed. Now you can explore all the other features of ArduPilot and tweak your setup to personal preferences. If you elect to use AUTOTUNE, be sure to read its `documentation <automatic-tuning-with-autotune.html>`__ thoroughly. Failing to run the autotune procedures as recommended bears the risk to decrease , rather than increase, your airframe's flight handling characteristics.

Mind that the default settings do work fairly well for most standard sized aircraft.

Finally, backup all the parameters to a file using the Write To File feature of MP in its Config/Tuning -> Full Parameters subtab.

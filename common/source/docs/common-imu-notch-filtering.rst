.. _common-imu-notch-filtering:

[copywiki destination="copter,plane"]

============================================================================
Managing Gyro Noise with the Static Notch and Dynamic Harmonic Notch Filters
============================================================================

As discussed under the :ref:`Vibration Damping<common-vibration-damping>` topic, managing vibration in ArduPilot autopilot installations is extremely important in order to yield predictable control of an aircraft. Typically installations utilize mechanical vibration damping for the autopilot, internally or externally, in order to remove the worst of the vibration. However, mechanical damping can only go so far and software filtering must be used to remove further noise.

To the autopilot, vibration noise looks like any other disturbance (e.g. wind, turbulence, control link slop, etc.) that the autopilot must compensate for in order to control the aircraft. This prevents optimum tuning of the attitude control loops and decreased performance.

For multicopters, virtually all vibrations originate from the motor's rotational frequency.  For helicopters and planes, the vibrations are linked to the main rotor/prop speed.

ArduPilot has support for two different types of notch filters - a single :ref:`static notch filter<common-imu-notch-filtering-static-notch>` that can be set at a fixed frequency and a dynamic harmonic notch filter that can be linked to the motor rotational frequency for multicopter motors or the rotor speed for helicopters and provides notches at the primary frequency and its harmonics. 

The dynamic notch is enabled overall by setting :ref:`INS_HNTCH_ENABLE<INS_HNTCH_ENABLE>` = 1. After rebooting, all the relevant parameters will appear.

Key to the dynamic notch filter operation is control of its center frequency. There are five methods that can be used for doing this:

#. :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 0. Dynamic notch frequency control disabled. The center frequency is fixed and is static. Often used in Traditional Helicopters with external governors for rotor speed, either incorporated in the ESC or separate for ICE motors.
#. :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 1. (Default) Throttle position based, where the frequency at mid-throttle is determined by analysis of logs, and then variation of throttle position above this is used to track the increase in noise frequency. Note that the throttle reference only applies to VTOL motors in QuadPlanes, not forward motors. Use the :ref:`static notch filter<common-imu-notch-filtering-static-notch>` for forward motor noise in Plane (usually setup for cruise operation).
#. :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 2. RPM sensor based, where an external :ref:`RPM sensor <common-rpm>` is used to determine the motor frequency and hence primary vibration source's frequency for the notch. Often used in Traditional Helicopters using the ArduPilot Head Speed Governor feature.
#. :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 3. ESC Telemetry based, where the ESC provides motor RPM information which is used to set the center frequency.
#. :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 4. In-Flight FFT, where a running FFT is done in flight to determine the primary noise frequency and adjust the notch's center frequency to match. This probably the best mode if the autopilot is capable of running this feature. Currently, this is restricted to 2MB flash memory autopilots.

Setup Instructions
==================

First you must select the mechanism used for controlling the harmonic notch frequency. By default (:ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 1) this mechanism is :ref:`throttle-based<common-imu-notch-filtering-throttle-based-setup>` since that will work on all Copters/QuadPlanes. However, a flight log analysis will be required to correlate the hover throttle position to the noise source's frequency.

However, for Copters with an rpm sensor, :ref:`BLHeli ESC telemetry support<blheli32-esc-telemetry>`, or is capable of using In-Flight FFT control of center frequency, setup can be radically simpler by using direct RPM or center frequency sensing, which will not require log analysis to determine center frequency:

- Set :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 2 to use an RPM sensor to set the harmonic notch frequency. This is primarily used in :ref:`Helicopters<common-imu-notch-filtering-helicopter-setup>`
- Set :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 3 to use BLHeli ESC telemetry support to set the harmonic notch frequency. This requires that your ESCs are configured correctly to support BLHeli telemetry via :ref:`a serial port<blheli32-esc-telemetry>`
- If your autopilot supports it (ie. has more than 2MB of flash, see :ref:`common-limited_firmware`), set :ref:`INS_HNTCH_MODE <INS_HNTCH_MODE>` = 4 to use In-Flight FFT to set the harmonic notch frequency. See :ref:`common-imu-fft` for additional information.
- Set :ref:`INS_HNTCH_REF <INS_HNTCH_REF>` = 1 to set the harmonic notch reference value, which means no scaling
- Set :ref:`INS_HNTCH_HMNCS <INS_HNTCH_HMNCS>` to enable up to three harmonics for notches.

Then follow these additional instructions for the chosen center frequency control method:

.. toctree::
    :maxdepth: 1
    
    Throttle Based <common-throttle-based-notch>
    RPM Sensor<common-rpm-based-notch>
    ESC Telemetry<common-esc-telem-based-notch>
    In-Flight FFT <common-imu-fft>

Double-Notch
============

The software notch filters used are very "spikey" being relatively narrow but good at attenuation at their center. On larger copters the noise profile of the motors is quite dirty covering a broader range of frequencies than can be covered by a single notch filter. In order to address this situation it is possible to configure the harmonic notches as double notches that gives a wider spread of significant attenuation. To utilize this feature set :ref:`INS_HNTCH_OPTS <INS_HNTCH_OPTS>` to "1".

.. note:: Each notch has some CPU cost so if you configure both dynamic harmonics and double notches (:ref:`INS_HNTCH_OPTS <INS_HNTCH_OPTS>` set to 3) you will end up with 8 notches on your aircraft per IMU. On flight controllers with 3 IMUs, this totals 24 notches which is computationally significant and could impact operation. For example, with F4 cpus with one IMU, using :ref:`INS_GYRO_RATE<INS_GYRO_RATE>` =0 (1khz) this is safe, as is 3 IMUs running with fast sampling (:ref:`INS_GYRO_RATE<INS_GYRO_RATE>` =1 (2khz) on H7 cpus.

Note also that with a double-notch the maximum attenuation is either side of the center frequency so on smaller aircraft with a very pronounced peak their use is usually counter productive.

.. _common-imu-notch-filtering-static-notch:

Static Single Frequency Notch
=============================

In addition to the dynamic harmonic notch filter it is also possible to configure an additional, independent static harmonic notch filter. You might want to do this where you have significant frame or propeller resonance at a particular throttle value. Analysis is identical to that for the :ref:`harmonic notch<common-imu-notch-filtering-flight-and-post-flight-analysis>`, but this time do the analysis after configuring the harmonic notch so that you can see any residual vibration.

- Set :ref:`INS_HNTC2_ENABLE <INS_HNTC2_ENABLE>` = 1 to enable the static notch
- Set :ref:`INS_HNTC2_FREQ <INS_HNTC2_FREQ>` = resonant peak in Hz to set the notch center frequency
- Set :ref:`INS_HNTC2_BW <INS_HNTC2_BW>` = the notch bandwidth, a reasonable default is half of the center frequency
- Set :ref:`INS_HNTC2_ATT <INS_HNTC2_ATT>` = the notch attenuation, higher attenuation will make the notch deeper and narrower
- Set :ref:`INS_HNTC2_HMNCS <INS_HNTC2_HMNCS>` to enable up to three harmonics for the notch.

.. toctree::
    :hidden:

    Traditional Heli Notch Filter Setup<common-imu-notch-filtering-helicopter-setup>


.. _choosing-hardware:

=============================
Choosing Your Rover Hardware
=============================

Setting up your Rover requires a collection of various hardware components. This guide is designed to help you understand these essential pieces of equipment, offering both necessary requirements and additional recommendations for optimal performance.

RC Transmitter
==============

The RC (Radio Control) transmitter allows you to manually control your Rover and toggle through its various control modes. You can use any RC transmitter/receiver system, provided it has a minimum of four channels. 

.. image:: ../../../images/spektrum-dx8.jpg
    :target: ../_images/spektrum-dx8.jpg

While choosing an RC Transmitter, consider the following:

* Avoid transmitters designed specifically for cars as they come with a steering wheel and throttle trigger which are not conducive for controlling a Rover with different driving modes and additional features.
* A transmitter with at least two toggle switches is preferred, with one of these having three distinct positions for various controls.
* For budget-conscious users, the `Turnigy 9x <http://hobbyking.com/hobbyking/store/__8992__Turnigy_9X_9Ch_Transmitter_w_Module_8ch_Receiver_Mode_2_v2_Firmware_.html>`__ ($54) is a well-regarded choice.
* For those seeking higher quality and more features, consider OpenTX/FrSky systems.

:ref:`Compatible RC Transmitter and Receiver Systems <common-rc-systems>` provides a more extensive list of options.

GPS Receiver
============

A GPS module is an integral part of your Rover setup. The :ref:`UBlox GPS + Compass Module <common-installing-3dr-ublox-gps-compass-module>` is highly recommended due to its combined GPS and compass functionality. 

For alternative GPS options, refer to :ref:`GPS solutions <common-positioning-landing-page>`.

.. image:: ../../../images/GPS_TopAndSide.jpg
    :target: ../_images/GPS_TopAndSide.jpg

Batteries and Charger
=====================

Powering your Rover requires reliable batteries and an efficient charger. 

* For a smaller 1/16 or 1/18 scale Rover, a 2S (7.2v) LiPo battery under 2600 mAh, like `this one <http://hobbyking.com/hobbyking/store/__16589__Turnigy_1700mAh_2S_20C_Lipo_Pack_Suits_1_16th_Monster_Beatle_SCT_Buggy_USA_Warehouse_.html>`__, should suffice.
* For larger 1/10th scale Rover, `this is a good size choice <https://hobbyking.com/en_us/zippy-5000mah-2s1p-30c-hardcase-pack.html>`__.
* An easy-to-use LiPo charger such as `this one <https://hobbyking.com/en_us/imax-b6-ac-v2-charger-discharger-1-6-cells-genuine-au-plug.html>`__ is recommended.

Telemetry Radio
===============

The :ref:`telemetry radio <common-telemetry-landingpage>` enables your Rover to communicate with your ground station remotely using the MAVLink protocol. This adds significant capabilities to your Rover, including real-time mission interaction and live data streaming from vehicle-mounted components. Although not compulsory, it is a valuable addition to your Rover setup!

.. image:: ../../../images/Telemetry_store.jpg
    :target: ../_images/Telemetry_store.jpg

Distance Sensors
================

If you're looking to equip your Rover with obstacle avoidance capabilities, :ref:`Sonar/IR sensors <sonar-sensors>` are a great choice. These sensors help the Rover detect and avoid objects in its path, enhancing its autonomous navigation abilities.


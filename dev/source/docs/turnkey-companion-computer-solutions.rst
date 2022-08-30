.. _turnkey-companion-computer-solutions:

====================================
Turnkey Companion Computer Solutions
====================================

This article lists turnkey companion computer solutions that are advertised as working with ArduPilot. These are typically commercially produced and tuned variations of open source solutions like BeagleBoard, ODroid, Raspberry Pi etc.

.. note::

   Please let us know if you discover a new companion computer, so we can add it to this list. 

Horizon31 - PixC4-Jetson
=========================================================

.. figure:: https://horizon31.com/wp-content/uploads/2020/03/pixc4-jetson_transparent-600x452.png
   :target:  https://horizon31.com/product/pixc4-jetson/

The `PixC4-Jetson <https://horizon31.com/product/pixc4-jetson/>`__ is a professional-quality NDAA Compliant Flight Management Unit (FMUv5) which supports ArduPilot and is tightly integrated with an Nvidia Jetson (Nano, Xavier NX or TX2 NX) SBC and peripheral support system (USB, MIPI, Ethernet, M.2 slot, etc.). The PixC4-Jetson is 83x59mm and features a board-to-board design for direct integration into user platforms or can be used with specialized "breakout" boards to support various vehicle types and wiring needs.

**Included turn-key software provides the following features:**

* UDP Unicast/Multicast/Broadcast Telemetry (MAVLink)
* LTE connection management with Layer-2 peer to peer VPN
* Powerful multi-endpoint video encoding pipelines with example/open-source scripts for customization
* `ATAK <https://www.civtak.org/>`__ Integration
* Web interface for configuration and remote terminal access
* Supports multiple simultaneous communication pathways such as LOS, LTE and Satcom with advanced stale command rejection and message deduplication
* Scalable and secure cloud connectivity to Horizon31's US servers and optional access to their cloud GCS and low-latency webRTC video distribution system (https://gcs.horizon31.com)

.. figure:: https://horizon31.com/wp-content/uploads/2020/12/webgcs_2-1024x560.jpg
   :target:  https://horizon31.com/webgcs/

Horizon31 - PixC4-Pi
=========================================================

.. figure:: https://horizon31.com/wp-content/uploads/2021/08/IMG_7517-600x534.jpg
   :target:  https://horizon31.com/product/pixc4-pi/
   
The `PixC4-Pi <https://horizon31.com/product/pixc4-pi/>`__ is a professional-quality NDAA Compliant Flight Management Unit (FMUv5) which supports ArduPilot and is tightly integrated with a Raspberry Pi 4 compute module and peripheral support system (USB, MIPI, Ethernet, M.2 slot, etc.). The PixC4-Pi is about the size of a business card (84x42mm) and weighs 67g (with heatsink). The PixC4-Pi includes the same software stack as the PixC4-Jetson, with the primary difference being the PixC4-Pi does not support h.265 encoding, only h.264.

   
UAVcast-Pro - Raspberry Pi software for LTE / WiFi Drones
=========================================================

.. figure:: https://uavmatrix.com/wp-content/uploads/2018/08/ipad.png
   :target:  https://uavmatrix.com/uavcast-pro/

`UAVcast-Pro <https://uavmatrix.com/uavcast-pro/>`__ is an application which simplifies the process to communicate with your ArduPilot
flight controller over cell or WiFi network. Create the ultimate 4G / 5G / LTE Drone in couple of clicks.

Everything can be configured from a responsive web interface which can be accessed from any devices such as phone, tablet, desktop.

Read the `Documentation <https://docs.uavmatrix.com/>`__  for installation and configuration.


**UAVcast-Pro supports the following:**

* UDP & TCP Telemetry
* Connect Flight Controller to RPI by **USB => USB** or **Telem => GPIO**
* UDP & TCP Video Stream
* HD Video
* Custom Video Pipelines
* VPN with NAT Traversal (Zerotier)
* Multiple Telemetry Destinations

**Supported Raspberry Models:**

* Model Pi0w - (with Raspian stretch lite or desktop)
* Model Pi2 - (with Raspian stretch lite or desktop)
* Model Pi3 - (with Raspian stretch lite or desktop)

UAVcast-Pro supports a broad variety of modems. Open link below to see supported models.
`Supported 4G / 5G / LTE Modems. <https://www.freedesktop.org/wiki/Software/ModemManager/SupportedDevices/>`__

Flying LTE has never been easier


4Gmetry Companion Computer Kit
==============================

.. warning::

   The ArduPilot dev team advise that this product may not be as "turnkey" as indicated below. Community reports indicate that the Kit does not include the LTE module and may also not supply a powering mechanism.  

`4Gmetry <http://4gmetry.voltarobots.com/>`__ (Volta Robots) is a plug&play companion computer kit, based on Odroid XU4 single board computer.

4Gmetry gets telemetry (MavLink) from the Autopilot and streams it over 4G internet to a remote control station. 4Gmetry is fully compatible with Volta OS to remotely manage fleets of robots via a simple high level API. 4Gmetry can be used for computer vision tasks and video/image streaming over internet. 4Gmetry comes ready to connect to your VPN, for safety/security purposes; this also allows you remote console access (e.g. SSH).

.. figure:: http://4gmetry.voltarobots.com/wp-content/uploads/2015/09/4gmetry-II1-450x450.png
   :target:  http://4gmetry.voltarobots.com/services/shop/

   4GmetryII

*Image and text from Volta Website*

Alfonce Remote Gateway - Companion Computer
===========================================

.. figure:: https://www.dronotique.fr/wp-content/uploads/2018/10/offre_complete-300x215.jpg
   :target:  https://www.dronotique.fr/produit/alfonce-remote-gateway/

`Alfonce Remote Gateway <https://www.dronotique.fr/produit/alfonce-remote-gateway/>`__ is a turn key companion computer to interface cameras, ArduPilot and ground stations over cell Network, Wifi and RC transmitter.

**It integrates :**

* MAVLink proxy
* Cameras management
* Media server with MJPEG video streams
* Computer vision scripts
* Web management interface
* Wifi and 3/4G connexion
* Open VPN client

**It supports a large variety of cameras :**

* USB : PTP and UVC
* Wifi : Sony, Mapir, MJPEG and RTSP

Full list can be `found here <https://www.dronotique.fr/docs/alfonce-remote-gateway/gestion-des-appareils-photos-et-cameras/modeles-dappareils-photos-et-de-cameras-supportes/>`__ 

You can also read the full `documentation <https://www.dronotique.fr/docs/alfonce-remote-gateway/>`__ to discover full functionalities

By default it installed on a OrangePi Zero 2 + H5, but it could be integrated on a lot of supported nano computers or servers!

XBStation - Platform For Internet Drone Base On Real Time 4G/5G Connectivity
============================================================================

.. youtube:: JkNdeAKmSrg
        :width: 100%

`XBStation <https://xb-uav.com>`__ is a solution for streaming video and controling UAV via internet (wifi, 3G/4G/5G cellular network).

`XBStation <https://xb-uav.com>`__ has been designed to provide the drone applications for delivery, surveys, security, ambulance and emergency response.

**Supported Companion Computers:**

* Raspberry Pi 2
* Raspberry Pi 3B/3B+
* Raspberry Pi 4
* Odroid XU4

**Features :**

* UDP & TCP Telemetry
* HD Video
* Multiple Vehicles Control with MissionPlanner
* Sharing Telemetry Data and Video Streaming to Your Partners/Customers
* Realtime Vehicle Informations (GPS, Battery, Roll, Pitch, Yaw, ...) API
* HD Streaming Live ArduPilot Drone Footage to Your Application/Youtube/Facebook
* OpenVPN
* XB Server for reliable connections

Read the `Documentation <https://xb-uav.com/getting-started/overview>`__  for manual setup and installation.

Read the `Ready to Fly Kit <https://product.xb-uav.com/xbstation-kit>`__ pre-loaded with XBFirm softwares, you just plug and play.

.. _turnkey-companion-computer-solutions:

====================================
Turnkey Companion Computer Solutions
====================================

This article lists turnkey companion computer solutions that are advertised as working with ArduPilot. These are typically commercially produced and tuned variations of open source solutions like BeagleBoard, ODroid, Raspberry Pi etc.

.. note::

   Please let us know if you discover a new companion computer, so we can add it to this list. 

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

You can also read the full `documentation <https://www.dronotique.fr/docs/alfonce-remote-gateway/>`__ to discover full functionnalities

By defaut it installed on a OrangePi Zero 2 + H5, but it could be integrated on a lot of supported nano computers or servers!

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

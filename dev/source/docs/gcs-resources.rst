.. _gcs-resources:

=============
GCS Resources
=============

This section explains what resources the ArduPilot makes available for GCS authors to enhance their interfaces with metadata.

Parameter Definitions
---------------------

Textual descriptions of parameters, expected value ranges and other information is available from our parameter metadata.  It is available in a range of formats from (e.g) https://autotest.ardupilot.org/Parameters/ArduCopter/

Onboard Log Definitions
-----------------------

While ArduPilot's onboard logs are self-describing - in that they contain information within themselves that describes the available messages and fields, we provide more metadata that would be problematic to provide in an embedded software image.

That data is available in a variety of formats from our autotest server:
https://autotest.ardupilot.org/LogMessages/


GCS Maintainers List
--------------------

We have a low-bandwidth mailing list where we announce significant changes to ArduPilot's GCS-facing functionality, usually to do with how we handle mavlink streams.

That `forum <https://groups.google.com/forum/#!forum/ardupilot-gcs>`__ is currently on google groups. 


SRTM Data
---------

The entire SRTM database is available at https://firmware.ardupilot.org/SRTM .  This data gives approximate terrain height for most of the planet.

Firmware Manifest
-----------------

The firmware available to be downloaded from our firmware server is catalogued in a JSON file available from `here <https://http://autotest.ardupilot.org/manifest.json.gz>`__.

A sample of this::


        {
            "board_id": 121, 
            "mav-type": "ANTENNA_TRACKER", 
            "vehicletype": "AntennaTracker", 
            "mav-firmware-version-minor": "1", 
            "format": "apj", 
            "url": "https://firmware.ardupilot.org/AntennaTracker/stable-1.1.0/OMNIBUSF7V2/antennatracker.apj", 
            "mav-firmware-version-type": "STABLE-1.1.0", 
            "brand_name": "Omnibus F7 V2", 
            "mav-firmware-version-patch": "0", 
            "mav-autopilot": "ARDUPILOTMEGA", 
            "USBID": [
                "0x0483/0x5740"
            ], 
            "platform": "OMNIBUSF7V2", 
            "mav-firmware-version": "1.1.0", 
            "bootloader_str": [
                "OMNIBUSF7V2-BL"
            ], 
            "git-sha": "e22170628d5a03a18c0445c5af2d3f3688c37ed4", 
            "mav-firmware-version-major": "1", 
            "manufacturer": "Airbot", 
            "latest": 0
        }, 

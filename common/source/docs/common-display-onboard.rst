.. _common-display-onboard:

===============
Onboard Display
===============

This article explains how to connect a small onboard display to an autopilot which can be useful for displaying a small amount of vehicle information before takeoff.

.. image:: ../../../images/common-display-pixhawk.png
    :target: ../_images/common-display-pixhawk.png
    :width: 300px

This information includes:

- arming failure messages
- flight mode
- battery voltage
- GPS Lock and satellite count
- Pre-arm passed/failed
- EKF status


Where to Buy
============

SSD1306
-------

- `Amazon <https://www.amazon.com/s?k=ssd1306&ref=nb_sb_noss_1>`__
- `Ebay <https://www.ebay.com/sch/i.html?_from=R40&_trksid=m570.l1313&_nkw=ssd1306&_sacat=0>`__

SSH1106
-------

- `Amazon <https://www.amazon.com/s?k=ssh1106&ref=nb_sb_noss_1>`__
- `Ebay <https://www.ebay.com/sch/i.html?_from=R40&_trksid=m570.l1313&_nkw=ssh1106&_sacat=0&LH_TitleDesc=0&_odkw=ssd1306&_osacat=0>`__

Connecting to a Autopilot
=================================

Connect the display to the autopilot's I2C port as shown in the image above

set :ref:`NTF_DISPLAY_TYPE <NTF_DISPLAY_TYPE>` to 1 if using an SSD1306, 2 if using the SH1106 and reboot the board.

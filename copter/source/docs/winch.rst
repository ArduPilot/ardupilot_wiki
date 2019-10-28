.. _winch:

====================
Delivery Cable Winch
====================

Copter 4.0 provides means to control a delivery winch. A PWM controlled output, ``SERVOx_FUNCTION`` = 88, allows controlling a winch motor via two RC channels using ``RCx_OPTION`` = 44 and 45.

The RC channel assigned Winch Enable (``RCx_OPTION`` = 44), either outputs no PWM, effectively allowing the winch motor to have zero torque and freely release cable, or output the value determined by the Winch Control channel below, effectively holding/deploying/retracting the cable in place when Winch Enable is high (>1800us).

The other channel assigned Winch Control (``RCx_OPTION`` = 45) is usually set via a 3 position switch. When high (>1800us) outputs ``SERVOx_MAX`` PWM, retracting the cable at maximum speed, or when low (<1200us), deploys the cable at maximum speed outputting ``SERVOx_MIN`` PWM. Otherwise, `SERVOx_TRIM`` is output, if in-between.

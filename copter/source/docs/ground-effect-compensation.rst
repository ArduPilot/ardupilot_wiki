.. _ground-effect-compensation:

==========================
Ground Effect Compensation
==========================

Copter includes ground effect compensation which reduces the weighting of the barometer (in favour of the accelerometers) when the vehicle is likely taking off or landing.  This reduces the bounce sometimes seen when landing vehicles with short legs compared to the length of their propellers.

If your vehicle is not suffering from the bounce on landing it's best to leave this feature disabled because it slightly increases the risk of high vibration levels upsetting the altitude estimate.

Setup
=====

Connect to the flight contrller using your ground station (i.e. Mission Planner) and set the :ref:`GND_EFFECT_COMP <GND_EFFECT_COMP>` parameter to "1".

Video
=====

..  youtube:: ljSV-36MOOU
    :width: 100%

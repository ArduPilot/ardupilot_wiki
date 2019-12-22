.. _balance_bot-testing:

=======================
Testing the Balance Bot
=======================

If you have completed all the previous steps successfully, then you're all set to start testing! 

 .. youtube:: -EESMnSEpeM
    :width: 100%

For testing, make sure your vehicle is connected to the GCS via telemetry radio. USB would be too messy! Though we would suggest using mavproxy, Mission Planner works fine too.

#. Turn your vehicle on and connect to GCS.

#. Make sure your vehicle is standing nearly upright, then arm it from the GCS (It's better this way than from radio).

#. If all is good, your vehicle should be able to balance itself or at least be trying to.

#. If the vehicle is trying to balance, but it's not perfect, try adjusting the PID values live(via GCS). If that isn't helping, turn the vehicle off. Go to the hardware page and see if you've met all the guidelines, especially regarding the wheels and weight distribution.

#. Once the vehicle can balance itself, try moving it around. Move up the throttle slightly and see how it goes.

#. If you find it too difficult to drive, try adjusting :ref:`BAL_PITCH_MAX<BAL_PITCH_MAX>` . It's the pitch angle at 100% throttle. If you increase it, the vehicle will accelerate faster but will become more likely to topple. Reducing it will make it more stable, but sluggish.

.. tip::

   It is possible to obtain better behavior of the rover by adjusting its PID. The :ref:`tuning page <balance_bot-tuning>` has more detailed information.

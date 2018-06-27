.. _balance_bot-testing:

=======================
Testing the Balance Bot
=======================

If you have completed all the previous steps succesfully, then you're all set to start testing! 

As a final word of warning, please remember this is only at an experimental phase. Everything may not work correctly at the first attempt. You may have to revisit the previous steps patiently, to get things to work correctly. Don't be disheartened if things go wrong, it personally took us days to get our balance bot up and running. 

Even if everything works, be aware that there may be some "hiccups" which will get fixed only in later stages of this project. We have identified some of these issues. The whole purpose of this documentation and testing phase, is so that you can identify any issues we may have missed and let us know about it. Take it in the right spirit!

Currently we support driving the balance bot only in Manual mode. The thing about manual mode, is that the speed control is entirely upto the user. Any throttle you give will cause the balance bot to accelerate. If not applied VERY cautiously, this can cause it to topple. Believe me, it's incredibly hard to drive this around in Manual mode. But then that's what "Manual" would mean. It's perfectly normal to have a lot of crashes, before you start getting the hang of it (The author is still trying).

You may notice that balance bot does not hold its position after arming. It may be moving back and forth aimlessly, at zero throttle. This is because we haven't yet added support for wheel encoders or a position controller. This is completely normal for now. So don't panic!

Even our balance bot is a little bit twitchy:
 .. youtube:: -EESMnSEpeM
    :width: 100%

For testing, make sure your vehicle is connected to the GCS via telemetry radio. USB would be too messy! Though we would suggest using mavproxy, Mission Planner works fine too.

#. Turn your vehicle on and connect to GCS.

#. Make sure your vehicle is standing nearly upright, then arm it from the GCS (It's better this way than from radio).

#. If all is good, your vehicle should be able to balance itself or atleast be trying to.

#. If the vehicle is trying to balance, but it's not perfect, try adjusting the PID values live(via GCS). If that isn't helping, turn the vehicle off. Go to the hardware page and see if you've met all the guidelines, especially regarding the wheels and weight distribution.

#. Once the vehicle can balance itself, try moving it around. Move up the throttle slightly and see how it goes.

#. If you find it too difficult to drive, try adjusting BAL_PITCH_MAX. It's the pitch angle at 100% throttle. If you increase it, the vehicle will accelerate faster but will become more likely to topple. Reducing it will make it more stable, but sluggish.

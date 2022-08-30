.. _safety-multicopter:

==================
MultiCopter Safety
==================

.. warning::

   Your first priority must be the safety of people!

#. **Crashes can happen, because of pilot error or hardware or software
   malfunction.**
#. **If you are flying anywhere near other people, you are putting them
   at risk!**

   #. Be sure to maintain safe distances between yourself, and
      spectators and your copter.
   #. Circumstances will require that you will need to make your own
      determination of what is a "safe distance" from people and
      property.
   #. At a minimum, consider: at least 10ft (3m) but not further than
      30ft (10m) from you.
   #. Keep all other people, property and obstacles considerably further
      away from your copter.
   #. Ensure that no one gets between you and your copter.
   #. Spectators should always be a safe distance behind the pilot.
   #. If people intrude beyond what you have determined to be the "safe"
      area, land immediately and do not take off until they are clear.
   #. At full power, an average sized multi-copter can exceed 20 mph (32
      km/h), can ascend to hundreds of feet and easily travel more than
      a mile in distance before running out of battery.

#. **If you are flying too high or near airports you are putting manned
   aircraft and the people on them at risk!**

   #. Get to know where your nearest airports are and do not fly
      anywhere near them

#. **Always ensure the battery cable is NOT connected to the power
   distribution board or harness until you are ready to fly.**

   #. Always Turn on the transmitter and ensure the throttle stick is
      all the way down **BEFORE** connecting the battery.
   #. After landing the first thing you should do is disconnect your
      battery cable.
   #. Do not turn off the transmitter until after you have disconnected
      the battery.
   #. Always remove your props while you are testing motors, your hands,
      arms and face and those of your friends will thank you.
   #. When the battery is connected, always assume the motors are armed;
      you can check with a short throttle pulse.
   #. Don't pick up the model and the radio at the same time, you may
      bump the throttle.
   #. Do not attempt to fly longer than your battery's safe capacity, it
      is very bad for the battery and can cause a crash.

#. **ArduPilot includes a motor arming safety feature.**

   #. Immediately prior to flight after the battery has been connected,
      the RC transmitter's throttle stick needs to be held down and to
      the right for several seconds to arm the motors.
   #. After landing your first response should be to hold the throttle
      down and to the left for several seconds to "Disarm" the motors.
      Disarm condition can be tested by moving the throttle stick up,
      if the motors do not move it is probably disarmed.
   #. Even when disarmed, the throttle stick should always be kept in
      the full down position except when flying.

#. **Get used to switching back to Stabilize mode from other modes and
   reassuming full manual control.**

   #. This is the single most important recovery technique (practice
      it).
   #. Stabilize mode can have Simple mode added to it, but if you do you
      should then practice with it till you are proficient.
   #. Do not start using any other modes than Stabilize or Stabilize
      plus Simple until you are VERY comfortable flying your copter.

#. **It is very important to have excess power available.**

   #. If you have insufficient power, the automatic controls can require
      more throttle than is available and destabilization may result.
   #. Ideally your copter should be able to hover at about 50% throttle
      (mid stick).

#. **Especially while you are learning, it is recommended that you avoid
   expensive, stiff, ultra-sharp carbon fiber props.**

   #. Get cheaper, more flexible and more breakable plastic propellers.
   #. Some of the super carbon fiber ones can cut better than a Ginsu
      and while they are almost indestructible - you are not.

#. **Important primary response to a crash, inadequate landing or
   unknown autopilot state.**

   #. The first thing to do is throw a towel over your copters
      propellers (Propellers may start spinning unexpectedly).
   #. Then immediately disconnect the battery.
   #. A large towel is your most important piece of safety equipment
      followed by a fire extinguisher and a first aid kit.
   #. Generally better to use the first one than the last one.

#. **When testing or flying any of the (waypoint) navigation modes
   (using GPS):**

   #. Ensure that your GPS has "Lock" before arming and takeoff.
   #. Check that your home position on the Mission Planner is in fact
      correct.
   #. If the GPS does not accurately report your home position, reboot
      and wait for 8 or more satellites (not just 3D lock) and check
      again.

#. **Always follow the law:**

   #. Our personal use of MultiCopters (models in general) is
      continually under attack by those who fear 'drones' and invasion
      of privacy.  If you break the law, or invade someone's privacy, or
      put them in harm's way, you threaten the future of our personal
      use of models. Please, understand the law and the rights of others
      - and fly accordingly.
   #. Most countries have a prominent model aircraft organization. 
      In the USA that is the `AMA <https://www.modelaircraft.org/>`__.
      Review the AMA `safety code <https://www.modelaircraft.org/sites/default/files/105.pdf>`__. 
      Working with the FAA and other government organizations, 
      the AMA has established (and continues to update) rules for `First Person View (FPV) Operations <https://www.modelaircraft.org/sites/default/files/550.pdf>`__. 
      If you are in the USA (or not), read these documents! The AMA has a
      strong lobbying group that will help protect our rights. 
      Get involved and support your country's model aircraft organizations -
      and help protect our right to fly.

.. warning::

   Most important: Keep a safe distance between your Copter and
   People!

.. tip::

   These tips can also help protect your multicopter from
   damage.

#. **Avoid sudden or extreme transmitter control stick deflections.**

   #. Move the control sticks in small measured increments and don't
      "yank" on them.
   #. If the copter is properly calibrated and balanced it should
      require only small stick inputs to control altitude, direction and
      speed.

#. **Your copter should be more or less stable on the horizontal plane
   without any control inputs.**

   #. If you are "fighting" the copter, land and fix it - something is
      not right - Hardware adjustment or software calibration may be
      required.

#. **Be especially careful of large throttle inputs, as a copter can
   gain (or lose) altitude very rapidly.**
#. **Because MultiCopters are symmetrical it is especially easy to lose
   Visual Orientation.**

   #. For manual flight modes, maintaining a clear vision of the Copters
      Orientation (direction it is facing) is the most critical part of
      successful flight.
   #. Especially while learning it is very important to keep your copter
      appropriately close to you to aid in maintaining visual
      orientation.
   #. Generally: more than 10ft (3m) but not further than 30ft (10m)
      from you.
   #. If the copter gets further than about 100ft (30m) it starts
      getting difficult to be able to maintain **orientation** and can
      easily crash.
   #. If you lose Yaw orientation while flying in Stabilize mode, try
      only flying forward and using yaw to steer like a car.
   #. It is much better to simply descend and land rather than have an
      **orientation-induced** crash or worse still - a **fly away**.
   #. Fly-Aways often happen when the copter is commanded to tilt back
      towards the pilot but has rotated in the meantime and is so far
      away that orientation is lost.
   #. Result: the copter flies further away and crashes or is lost.

#. **Always have Stabilize mode as the (Go To) one of your mode switch
   options.**
#. **High or unexpected winds or gusts can make flight considerably more
   difficult.**

   #. High winds can prevent forward progress or spin the copter around
      causing you to become disoriented.
   #. The higher you are, the more likely high winds will be a problem.
   #. Switching to Stabilize mode and landing before you reach your
      skill limits can help you save your copter.
   #. Avoid flying at high speed or high altitude until you have gained
      considerable confidence in both manual and automatic modes.
   #. When flying around trees or buildings it is very easy to lose
      visual orientation or even to lose sight of the copter completely.
   #. Gusting winds around objects can also worsen the problem.
   #. Radio signal loss can also occur.
   #. If your copter is approaching a potentially interfering object,
      immediately switch to Stabilize mode and land or retrieve the
      copter to your location.

#. **ArduPilot specific safety modes: RTL, FailSafe and GeoFence.**

   #. RTL can provide a safe **Return to Launch** if it starts to get
      away from you.
   #. Set up a **FailSafe** on Radio Fail with an RTL or Descend
      response to save your Copter and prevent Injury.
   #. **GeoFence** establishes an automatic flying perimeter that will
      force your copter to stay in a safe proximity.
   #. Do not rely solely on the above safety modes, always be ready to
      take back control in Stabilize and set the copter down.
   #. Especially do not rely on the above safety modes to perform
      maneuvers or training that you would otherwise consider dangerous.
   #. These modes are a supplement to, not a replacement for sound
      safety practices.

#. **On your first takeoff after tuning or hardware setup:**

   #. In Stabilize mode advance the throttle very slowly until the
      copter is almost hovering.
   #. If the copter is trying to flip over turn it off and correct the
      problem.
   #. A motor could be turning the wrong direction.
   #. Or a wrong direction prop could be installed.
   #. If it tries to rotate on its axis or fly off in some direction.
   #. The transmitter or RC setup in Mission Planner may be incorrect.
   #. A motor or ESC may not be performing properly.
   #. The wrong props may be on the wrong motors.
   #. When all problems are fixed it should be fairly easy to get the
      copter to hover a foot or 2 above the ground.
   #. If a stable and stationary hover a foot or 2 above the ground
      cannot be achieved, land and fix the problem until it can.

#. **When flying FPV "First Person View" (with a video camera), Have
   your modes set to: STABILIZE, SIMPLE, and RTL.**

   #. Ensure RTL is working properly before using FPV.
   #. Use Stabilize mode to fly FPV.
   #. If you lose your FPV video, you can switch to Simple or RTL to get
      back.

#. **Make sure your battery can't fall out.**

   #. Use a Velcro Strap to hold it in place.
   #. You can also use adhesive backed velcro under the battery.

.. note::

   Get a Printable PDF Safety Sheet: `MultiCopter_Safety <https://download.ardupilot.org/downloads/wiki/pdf_guides/MultiCopter_Safety.pdf>`__ 

The `Copter Forum <https://discuss.ardupilot.org/c/arducopter>`__ permits the
developers to respond to your questions and enables you to research
similar issues, Please choose the sub-forum that is most appropriate to
the wiki page and issues you are having.

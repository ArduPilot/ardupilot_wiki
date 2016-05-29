.. _safety-multicopter:

==================
多旋翼直升机安全
==================

.. warning::

   你的首要任务是保证人的安全！

#. **由于软硬件故障，或者导航信息出现错误，炸机是随时可能发生的。**
#. **如果你在别人附近飞行，那你就是置别人与危险之中！**

   #. 确保你的直升机与你自己和周边观众保持足够的安全距离。
   #. 距离人和财产多远才算是“安全距离”，需要你根据具体情况自己做出判断。
   #. 至少，可以考虑：距离你自己不少于10ft(3米)，同时也不要大于30ft(10米)。
   #. 保持你的直升机尽可能远离其他人、财产和障碍物。
   #. 确保没有人出现在你和你的直升机之间。
   #. 观众应该始终处于驾驶员后的安全距离之外。
   #. 如果有人进入了你认为的“安全”区域以内，请立即降落，待人离开后再起飞。
   #. 在满电状态下，普通大小的多旋翼直升机速度可以超过20 mph（32千米/时），可以爬升到上百米，在电量耗尽前可以轻松飞行几公里远。

#. **如果你飞的太高或者太靠近机场，你就会置载人飞行器和他人与危险之中！**

   #. 了解离你最近的机场的位置，不要在其附近飞行。

#. **在你准备飞行之前，始终确保电源线与配电板或者配电线束之间处于断开状态。**

   #. 始终在连接电源线**之前**,记得打开遥控器并把油门拉到最低。
   #. 降落之后，你所要做的第一件事是断开电源线。
   #. 确保断开电源之后再关闭遥控器。
   #. 在做电机测试之前，始终记得去掉螺旋桨。你和你小伙伴们的手和脸都会感激你的。
   #. 当电源线连接以后，始终假定电机处于解锁（arm）状态，你可以轻推油门加以验证。
   #. 不要同时拿起航模和遥控器，你有可能会碰到油门。
   #. 不要尝试飞行时间超过你的电池安全承载能力，这对你的电池伤害极大，还有可能导致炸机。

#. **我们在APM和PX4的飞行控制器中集成了一个电机解锁安全功能，**

   #. 在电池连接之后，即将起飞之前，需要把无线遥控器的油门摇杆拉到右下角保持几秒来解锁电机。
   #. 降落之后，你的第一个动作应该是吧油门摇杆拉到左下角并保持几秒来“锁定”电机。
      \* 可以通过向上轻推油门测试锁定状态。如果电机不转，说明处于锁定状态。
   #. 除了飞行过程中，即使处于锁定状态，也要始终保持油门摇杆处于最低位置。

#. **要习惯于从其他模式切换回稳定模式，进而假定会切换到全手动控制。**

   #. 这是最重要的，也是唯一的恢复技巧（多多练习）。
   #. 稳定模式上可以叠加简单模式，但如果你想这样做你应该练习到你熟悉以后再尝试。
   #. 在你非常适应你的直升机之前，请不要尝试稳定模式和稳定+简单模式外的任何模式。

#. ** 预留冗余推力非常重要。**

   #. 如果推力不足，当自动控制的需求超过油门极值，就可能会导致不稳定。
   #. 理想状态下，当加到50%油门（摇杆中点），你的直升机应该可以悬停。

#. **建议你不要用贵的、坚硬的、极其锋利的碳纤维桨，尤其当你还在学习中。**

   #. 建议使用便宜的、更灵活更易碎的塑料桨。
   #. 一些超级碳纤维桨比武士刀还锋利，几乎是无坚不摧的 - 你不是。

#. **炸机、未完全降落或者未知的飞行控制情形下的重要应对措施。**

   #. 要做的第一件事情是扔条毛巾到你的直升机螺旋桨上（螺旋桨可能会出乎预料的乱转）。
   #. 然后立即断开电池连接。
   #. 除了灭火器和急救箱，一个大的毛巾是你非常重要的安全装备。
   #. 通常，首先使用毛巾好过另两种选择。

#. **在测试或者飞行任何导航模式时（使用GPS）：**

   #. 在解锁起飞之前，确保你的GPS已经“定位”。
   #. 检查你的起始位置和任务计划确实准确无误。
   #. 如果GPS没有精确报告你的起始位置，重启然后等待搜到8颗星以上（不仅仅是3D定位）再次检查。

#. **始终遵纪守法**

   #. 我们对于多旋翼直升机（一般的航模）的个人使用持续受到害怕‘无人机’和隐私保护人士的攻击。如果你违反法律，或者侵犯别人隐私，或者可能对他人造成伤害，你就威胁到了我们未来为航模的个人使用。所以，请了解相关法律并了解别人的相关权利 - 然后有依据的飞行。
   #. 很多国家都有知名的航模组织。在美国就是\ `AMA <http://www.modelaircraft.org/>`__.
      阅读AMA的 `安全准则 <http://www.modelaircraft.org/files/105.pdf>`__. 通过与FAA和其他政府组织的合作，AMA已经完成了（持续更新中） `无人机和第一视角飞行条例 <http://www.modelaircraft.org/documents.aspx#FPV>`__. 如果你在美国（或者不在），请阅读该文档！AMA是一个强大的游说集团，可以帮助我们保护我们的权利。加入或者支持你们国家的航模组织 - 保护我们飞行的权利.

.. warning::

   重要提示: 让你的直升机与人们保持安全距离！

.. tip::

   这些提示也可以帮助保护你的多旋翼直升机远离损毁。

#. **避免突然或者遥控器控制摇杆变位**

   #. 小刻度的移动控制摇杆，不要“猛拉”。
   #. 如果直升机已经适合的校正平衡过，那么它只需要很小的摇杆输入就可以控制高度、方向和速度了。

#. **你的直升机在没有控制输入时，应该或多或少的稳定在同一个水平线上。**

   #. 如果你正在和你的直升机“搏斗”，那么请降落并把问题解决掉 - 一定有某些地方不对 - 需要硬件调校或者是软件校准

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
      immediately switch to stabilize mode and land or retrieve the
      copter to your location.

#. **ArduPilot specific safety modes: RTL, FailSafe and GeoFence.**

   #. RTL can provide a safe **Return to Launch** if it starts to get
      away from you.
   #. Set up a\ ** FailSafe** on Radio Fail with an RTL or Descend
      response to save your Copter and prevent Injury.
   #. **GeoFence** establishes an automatic flying perimeter that will
      force your copter to stay in a safe proximity.
   #. Do not rely solely on the above safety modes, always be ready to
      take back control in stabilize and set the copter down.
   #. Especially do not rely on the above safety modes to perform
      maneuvers or training that you would otherwise consider dangerous.
   #. These modes are a supplement to, not a replacement for sound
      safety practices.

#. **On your first takeoff after tuning or hardware setup:**

   #. In stabilize mode advance the throttle very slowly until the
      copter is almost hovering.
   #. If the copter is trying to flip over turn it off and correct the
      problem.
   #. A motor could be turning the wrong direction.
   #. Or a wrong direction prop could be installed.
   #. If it tries to rotate on it's axis or fly off in some direction.
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

   Get a Printable PDF Safety
   Sheet: \ `MultiCopter_Safety <http://download.ardupilot.org/downloads/wiki/pdf_guides/MultiCopter_Safety.pdf>`__\ 

The \ `Copter Forum <http://ardupilot.com/forum/viewforum.php?f=3>`__ permits the
developers to respond to your questions and enables you to research
similar issues, Please choose the sub-forum that is most appropriate to
the wiki page and issues you are having.

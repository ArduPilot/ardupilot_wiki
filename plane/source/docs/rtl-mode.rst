.. _rtl-mode:

===========================
RTL Mode (Return To Launch)
===========================

Return to Launch mode is intended to bring a plane back to the launch location during a 
failsafe event or an operator's command. The plane will return to the launch location 
(the point where the plane armed - assuming it had GPS lock) and loiter there 
until given alternate intructions (or it runs out of fuel!). As with :ref:`AUTO <auto-mode>` mode
you can also "nudge" the aircraft manually in this mode using stick
mixing. The target altitude for RTL mode is set using the
:ref:`ALT_HOLD_RTL <ALT_HOLD_RTL>` parameter in centimeters.

Alternatively, you may :ref:`configure the plane to return to a Rally Point <common-rally-points>`, 
rather than the launch location.

.. warning::

   RTL is not designed to recover aircraft in unusual attitudes. If 
   an aircraft has become unstable in an autopilot-assisted mode 
   (FBWA, Auto, etc.), RTL will rarely have desired effects. 
   Switch to MANUAL mode to level the plane, then engage RTL.
   
   "Launch" position should always be the Plane's actual GPS takeoff 
   location; it is very important to acquire GPS lock before arming in 
   order for RTL, Loiter, Auto or any GPS dependent mode to work properly.
   
   
.. note::   
   The home position is initially established at the time a plane 
   acquires its GPS lock. It is then continuously updated as long as
   the autopilot is disarmed.
   
   Consider the use of :ref:`Rally Points <common-rally-points>` to 
   avoid returning directly to your arming point on RTL

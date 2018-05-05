.. _flip-mode:

=========
Flip Mode
=========



Vehicle will flip on its roll or pitch axis depending upon the pilot's roll and pitch stick position. Vehicle will rise for 1 second and then rapidly flip. The vehicle will not flip again until the switch is brought low and back to high. Give yourself at least 10m of altitude before trying flip for the first time!



Flip Mode State Machine
-----------------------


 Controls:
 
 *          CH7_OPT - CH12_OPT parameter must be set to "Flip" (AUXSW_FLIP) which is "2"
 *          Pilot switches to Stabilize, Acro or AltHold flight mode and puts ch7/ch8 switch to ON position
 *          Vehicle will Roll right by default but if roll or pitch stick is held slightly left, forward or back it will flip in that direction
 *          Vehicle should complete the roll within 2.5sec and will then return to the original flight mode it was in before flip was triggered
 *          Pilot may manually exit flip by switching off ch7/ch8 or by moving roll stick to >40deg left or right
 
 
 State machine approach:
 
 *          Flip_Start (while copter is leaning <45deg) : roll right at 400deg/sec, increase throttle
 *          Flip_Roll (while copter is between +45deg ~ -90) : roll right at 400deg/sec, reduce throttle
 *          Flip_Recover (while copter is between -90deg and original target angle) : use earth frame angle controller to return vehicle to original attitude
 

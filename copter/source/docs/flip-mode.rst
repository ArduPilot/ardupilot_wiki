.. _flip-mode:

=========
Flip Mode
=========

Vehicle will perform a full-rotation flip.
The pilot can control the direction of flip using gentle deflection of the roll or pitch stick.
What to expect:

- Flip will happen fast (~1 sec).
- Vehicle will increase throttle rapidly as it starts to flip.
- Vehicle does a full-rotation flip (passing through upside-down).
- Vehicle attempts to end the flip at the starting altitude.
- After the flip ends (successful or canceled), the vehicle's previous flight mode will be restored.

The vehicle will not flip again until the controls are reset (see below).

.. warning:: Give yourself at least 10m of altitude before trying flip for the first time!


Flip Mode Controls
==================
Flip Mode can only be entered from modes which permit it (Acro, AltHold, FlowHold, Stabilize).

The mode may be entered either by an RC :ref:`Auxiliary Switch <common-auxiliary-functions>` or by changing flight mode to FLIP.
A spring-loaded entry switch (RC Aux or mode-switch) is recommended, so that pilot must hold it while flipping.

- The direction of the flip defaults to ROLL LEFT.
- If the RC Pitch stick is moved slightly back or forward, it will flip on the pitch axis, back or forward, respectively.
- If the RC Pitch stick is neutral but the RC Roll stick is pushed slightly right, it will flip rolling to the right.
- In addition to mode-changing out of FLIP, you may abort the flip by:

  - Moving the pitch or roll stick to an extreme value in any direction.
  - Switching the RC Aux switch to LOW.

- If aborted these ways, the flip will immediately halt **at whatever attitude it is currently at** and return to the previous flight mode.
- To do another flip, because every flip automatically mode-switches away at its end, pilot must 'exit the first flip' (via RC Aux or mode-switch) before initiating the next.

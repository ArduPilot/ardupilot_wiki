.. _guide-center-of-gravity:

=================
Center of Gravity
=================

Getting the center of gravity right in an aircraft is critical for it
to fly well.

.. note:: A common adage in aero-modelling is that nose heavy planes
   fly badly, but tail heavy planes fly once.

The center of gravity is the balance point of your plane. You will
need to move your battery or add and remove weight from your aircraft
so that it balances at the right point.

Check the instructions
======================

Most planes will come with instructions on the right CoG (center of
gravity) of the aircraft. This is often specified as a distance back
from the leading edge of the main wing.

.. warning:: unfortunately it is fairly common for instructions
             manuals for cheaper planes to have an incorrect CoG
             listed. Try to confirm with on-line forums or someone who
             has flown the same plane.

Very roughly, you expect the CoG to be about 1/3 of the way back from
the front of the wing. If you have doubt about the correct CoG then
you should err on the side of making the plane too nose heavy instead
of too tail heavy. A nose heavy plane may have trouble taking off and
climbing, but it will usually be stable. A tail heavy plane may be
very unstable and crash shortly after takeoff.

Check Online Forums
===================

You should also check online forums for other people who have flown
the same plane. It is very common for the correct CoG to be discussed.

Correct After First Flight
==========================

One of the key things you should check after your first flight is
whether your CoG was in fact correct. Check your flight log and look
at the state of the pitch integrator. The pitch integrator is in the
PIDP.I field in your dataflash log.

If your plane showed a value of PIDP.I consistently above zero then
your plane was needing to put in up elevator to stay level. Your plane
is probably a bit nose heavy, or your elevator trim is incorrect.

If your plane showed a value of PIDP.I consistently below zero then
your plane was needing to put in down elevator to stay level. Your
plane is probably a bit tail heavy, or your elevator trim is
incorrect.

.. _traditional-helicopter-first-flight-tests:

==================
First Flight Tests
==================


First Spool Up
==============

Check the CG of helicopter. It should be exactly on the point of the main shaft. If not, adjust battery until it is.

For the first spool up, before trying to lift into hover, you should verify that the main rotor blades are tracking reasonably well. If not, excessive vibration will be created, perhaps high enough to cause attitude control errors or even cause an :ref:`ekf-inav-failsafe`.

- Arm the vehicle. If there is a pre-arm failure, resolve it before proceeding. Usually you will need to wait until the GPS obtains lock and the EKF sets its origin as indicated by the ground control station messages (and, of course, the failure to arm!)
- Before the automatic disarming timing elapses, engage the Motor Interlock switch and/or raise the collective above 1/4 stick (if using passthru RSC mode)to enable Motor Interlock. The rotor should spool up to speed. Usually, one spools up at 0 deg collective pitch setting to avoid stress while on the ground
- At this point, check that the blades are tracking. If not, adjust a blade's pitch link to the swashplate and re-try until the plane of both blades is close to identical.

.. youtube:: 3g3hJtBhSJ4

.. tip:: See :ref:`Traditional Helicopter Tips <traditional-helicopter-tips>` for more on vibration issues.

First Hover
===========

Once tracking has been verified, you can do your first test hover. In STABILIZE, repeat the above, but once the rotor has finished spooling up, increase collective slowly and lift off. Be prepared to  put in small cyclic corrections to hover the vehicle in place. If the vehicle seems too unstable in attitude, you might try setting the PID values to those used for beginning the fine tuning of the attitude control loops, discussed in the following section (:ref:`traditional-helicopter-tuning` ).

Enable Notch Filtering
======================

Assuming that first low in-place hover was successful, which it should be if you have followed the wiki, you should setup a dynamic harmonic notch filter. Tuning of the control loops, which is the next step, is greatly enhanced if the vehicle's generated vibrations are attenuated by the filter.

Follow the instructions in :ref:`Helicopter Dynamic Notch Filter Setup<common-imu-notch-filtering-helicopter-setup>`.

.. note:: There usually are two rotational vibration sources in Traditional Helicopters, the main rotor and the tail rotor. The tail rotor frequency may appear as if its a harmonic of the main rotor, but not quite at an integer multiple of the main rotor fundamental frequency. For example, if the tail rotor has a 1:3.8 gearing to the main rotor, it will appear near what would be the 4th harmonic of the main rotor, but is slightly lower (ie 3.8x the fundamental of the main rotor). In this case, the harmonic notch setup for the 4th harmonic may still be effective if the bandwidth of the notches, :ref:`INS_HNTCH_BW<INS_HNTCH_BW>`,are wide enough.

After the harmonic notch is setup, proceed to :ref:`traditional-helicopter-tuning`.
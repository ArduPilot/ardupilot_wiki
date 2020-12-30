.. _common-power-tether:

======================
Home-Made Power Tether
======================

For Maker-Faire 2017, the Cube Autopilot team needed a power tether for the ArduCopter-following-ArduRover demonstration.  Plans to use a commercially-available tether collapsed so Mr Challinger and Mr Wurzbach made a DIY tether in just 24h.

.. note::

   The equipment described here deals with high power electronics which can damage electrical equipment and even kill people if not done properly.  Please be confident in your abilities before attempting to replicate this project.

Demonstration
=============

..  youtube:: eV7bp9pxVE0
    :width: 100%

Parts used
==========
* AC to DC Power Supply Single Output 27 Volt 56 Amp (`like this one <http://www.jameco.com/z/RSP-1500-27-Mean-Well-AC-to-DC-Power-Supply-Single-Output-27-Volt-56-Amp-1-512kw_694946.html>`__)
* 600 Watt Surface Mount Transient Voltage Suppressor (`like this one <https://www.jameco.com/z/NMBJ24A-13-F-Diodes-Zetex-600-Watt-Surface-Mount-Transient-Voltage-Suppressor_1542371.html>`__)
* Capacitor Snap 10000 uF 50 Volt 20% 105c (`like this one <https://www.jameco.com/z/EC10K50LP-R-Capacitor-Snap-10000-uF-50-Volt-20-105c-Large-Can-Aluminum-Electronic_157738.html>`__)
* 25ft extension cord
* 40ft 14AWG twisted pair for power (we used silicone insulation for abrasion resistance)
* 40ft 30AWG twisted pair for remote voltage sense (same)

Construction
============

* Trim the output voltage to 24v
* Use a 6s battery capable of powering the drone on its own
* Sense wires, battery, caps and TVS should be connected on drone end of power wire
* The cathode of the TVS (the side with the white line) goes on the positive lead.

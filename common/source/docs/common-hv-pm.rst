.. _common-hv-pm:

================================
HV PM(High voltage power module)
================================

Overview：
=========

The CUAV HV_PM module is a new high voltage voltage power module independently developed by CUAV with the following features:
* Higher voltage input: 10v-60v (3s~14s battery)
* More accurate battery monitor: 
  * voltage detection accuracy: +-0.1v
  * current detection accuracy: +-0.4A
* Bec (5v) max current: 5A
* Max (detection) current: 60A
* Max output current(ESC/MOTOR PORT): 60A

Enable HV PM
============

.. image:: ../../../images/hv-pm.jpg
:target: ../_images/hv-pm.jpg

* Monitor：Anglog Voltage and Current
* Sensor：CUAV HV PM
* APM Ver：CUAV v5 or Pixhawk4(V5 series flight control selects CUAV v5)
* Click on other interface to write parameters
* Restart mp ground station and flight control

You can also enable it by setting the following parameters.

- :ref:`BATT\_MONITOR` to 4 to Set to analog voltage and current.
- :ref:`BATT\_VOLT\_PIN` to 0.
- :ref:`BATT\_CURR\_PIN` to 1.
- Restart mp ground station and flight control
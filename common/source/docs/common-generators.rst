.. _common-generators:

==========
Generators
==========

.. figure:: ../../../images/generators.png
   :target: ../_images/generators.png

ArduPilot supports generators for powering your vehicle.  Some generators can be controlled from ArduPilot, others are telemetry only.  Within ArduPilot the term generator is used to broadly group all devices that convert some form of fuel into electrical power.

The following are all currently supported within ArduPilot:

.. toctree::
    :maxdepth: 1

    RichenPower generator <common-richenpower-generator>
    Intelligent Energy 650W/800W Hydrogen Fuel Cell <common-ie650-fuelcell>
    Intelligent Energy 2.4kW Hydrogen Fuel Cell <common-ie24-fuelcell>
    Shang Fei Aviation <http://www.svffi.com/en> 

.. note:: Shang Fei generator uses a LUA script driver `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/drivers>`__


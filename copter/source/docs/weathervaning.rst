.. _weathervaning:

Weathervaning and Wind Hold
===========================

Flying a Traditional Helicopter, or some Multicopters, in significant levels of wind can present a
challenge. The issue is that any flat plate surface, such as a Traditional Helicopter's tail fin and boom, offers a lot of surface area for the wind to interact with. That can lead to a
reduction in attitude and position control and high motor and ESC
load.

To reduce the impact of wind when flying in VTOL modes the ArduPilot code supports active yaw axis weathervaning which can greatly reduce the impact of wind on
VTOL flight by keeping the vehicle pointed into the wind and reducing
the surface area exposed to the wind.

Active Weathervaning
--------------------

Active weathervaning acts to turn the aircraft into the
wind when flying in GUIDED and/or AUTO modes. You can enable
active weathervaning by setting the :ref:`WVANE_ENABLE <WVANE_ENABLE>`
parameter to a non-zero value and selecting to use it in either GUIDED and/or AUTO modes. The default is not to use active weathervaning.

+---------------------------------------------+--------------------------+
+ :ref:`WVANE_ENABLE<WVANE_ENABLE>` Value     | Weathervane Direction    +
+=============================================+==========================+
+         0                                   |  Disabled                +
+---------------------------------------------+--------------------------+
+         1                                   |  Nose Into Wind(default) +
+---------------------------------------------+--------------------------+
+         2                                   |  Nose OR Tail Into Wind  +
+---------------------------------------------+--------------------------+
+         3                                   |  Side Into Wind          +
+---------------------------------------------+--------------------------+
+         4                                   |  Tail Into Wind          +
+---------------------------------------------+--------------------------+

Then you must also enable it for GUIDED and/or AUTO mode by setting bit 7 of the :ref:`GUID_OPTIONS<GUID_OPTIONS>` and/or :ref:`AUTO_OPTIONS<AUTO_OPTIONS>` parameter, respectively. It will not be active in any other 
mode.

.. note:: enabling weathervaning will override any yaw specific control in missions, such as pointing the nose to ROI targets, or responding to MAVLink YAW positioning commands. If pilot yaw control is enabled, the pilot can change the yaw, but as soon as pilot input ceases, the vehicle will yaw back to its Weathervaing Direction set above.

The way it works is the autopilot looks at the roll and/or pitch attitude needed to
control the desired position. The basic algorithm is "yaw into the
roll/pitch". If the aircraft needs hold roll to the right in order to hold
position then it will turn in that direction on the assumption that
the right roll is needed in order to hold against the wind (assuming nose into the wind). Similarly, for Side Into the Wind, it will use the pitch angle and yaw appropriately, trying to zero the pitch required to hold position.

.. note:: by default, weathervaning does not use pitch, only roll. This is to prevent unwanted continuous yawing if the hover attitude has not been trimmed to hover, in place, with no wind. If this **has** been done, then pitch driven weathervaning can be enabled by setting :ref:`WVANE_OPTIONS<WVANE_OPTIONS>` bit 0 to "1". This will speed weathervaning if positioned with the wind blowing from behind (ie mostly being held in position with pitch, not roll). This does not affect Side Into Wind, it always uses pitch, hover trim attitude must be properly set for good operation.

How quickly the aircraft yaws is determined by the :ref:`WVANE_GAIN<WVANE_GAIN>`
parameter. It converts the lean angle into degs/sec of yaw. A good value to start with is 1 (1 degree roll = 1 deg/sec yaw). Higher values will make
the aircraft turn into the roll more quickly. If the value is too high
then you can get instability and oscillation in yaw.

To cope with a small amount of trim in the aircraft there is an
additional parameter :ref:`WVANE_ANG_MIN<WVANE_ANG_MIN>` which
controls the minimum roll/pitch level before weathervaning will be
used. This defaults to one degree. If you find your aircraft starts
yawing even in no wind then you may need to raise this value.

There are a number of additional parameters that can control when WeatherVaning is active (all are disabled by default):

- :ref:`WVANE_HGT_MIN<WVANE_HGT_MIN>`: above this height weathervaning is permitted
- :ref:`WVANE_SPD_MAX<WVANE_SPD_MAX>`: below this ground speed weathervaning is permitted
- :ref:`WVANE_VELZ_MAX<WVANE_VELZ_MAX>`: maximum climb or descent speed at which the vehicle will still attempt to weathervane
- :ref:`WVANE_TAKEOFF<WVANE_TAKEOFF>`: override weathervaning direction in auto takeoffs*
- :ref:`WVANE_LAND<WVANE_LAND>`: overide weathervaning directions in auto landings*

.. note:: to allow weathervaning only on takeoffs and landing during an AUTO mission or in GUIDED mode, set :ref:`WVANE_ENABLE<WVANE_ENABLE>` =0 but use the :ref:`WVANE_LAND<WVANE_LAND>` and/or :ref:`WVANE_TAKEOFF<WVANE_TAKEOFF>` parameters to enable the weathervaning control.

.. note:: Weathervaning can be disabled or enabled by an :ref:`RC Aux Function Switch<common-auxiliary-functions>`, option "160"


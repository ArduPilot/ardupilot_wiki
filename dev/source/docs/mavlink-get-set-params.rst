.. _mavlink-get-set-params:

==============================
Getting and Setting Parameters
==============================

ArduPilot has hundreds of parameters which allow the user to configure many aspects of how the vehicle flies/drives including attitude controller gains, the minimum safe altitude to return home at when flying in Return-To-Launch mode, etc.

Most ground stations provide an editable list of available parameters to the user to allow them to customise the vehicle.  A list of parameters can be found `here for Copter <http://ardupilot.org/copter/docs/parameters.html>`__, `Plane <http://ardupilot.org/plane/docs/parameters.html>`__, `Rover <http://ardupilot.org/rover/docs/parameters.html>`__ and `AntennaTracker <http://ardupilot.org/antennatracker/docs/parameters.html>`__ but this list changes with each firmware release so GCSs and companion computers normally retrieve the list of parameters directly from the flight controller soon after connecting.

This page explains how a ground station or companion computer can retrieve the list of parameters from the flight controller and then make changes to them.  For reference the general purpose `MAVLink parameter protocol is described here <https://mavlink.io/en/services/parameter.html>`__.

Retrieving All Parameters
-------------------------

The most efficient way to receive all parameters is to send a `PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`__ message with the following fields:

- **target_system** : the mavlink system id of the vehicle (normally "1")
- **target_components** : normally "0"

The vehicle should respond with many individual `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ messages (one for each parameter in the system).  The fields of the `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ messages are:

- **param_id** : the parameter name, a 16 character string (null terminated if less than 16 characters)
- **param_value** : the parameter value (all parameter values are sent as floats)
- **param_type** : the `MAV_PARAM_TYPE <https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE>`__.  Note that ArduPilot only supports MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32 and MAV_PARAM_TYPE_REAL32 (aka float) types.
- **param_count** and **param_index** fields should be used to determine if all parameters have been successfully received

MAVLink does not guarantee delivery of packets so if some parameter values do not arrive, use the instructions below to re-request the missing parameters.

Retrieving a Parameter
----------------------

To retrieve just a single parameter send a `PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`__ message with the following fields:

- **target_system** : the mavlink system id of the vehicle (normally "1")
- **target_components** : normally "0"
- **param_id** : the human readable 16 character parameter name if known.  If not known, this field can be anything as long as the param_index is set to a non-negative number
- **param_index** : the parameter index number or -1 if the "param_id" field should be used instead.  Note that it is more efficient for the AutoPilot to find the parameter using this fields instead of the param_id.

The vehicle should respond with a `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message as described in the section above.

Setting a Parameter
-------------------

To set the value of a single parameter send a `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`__ message with the following fields:

- **target_system** : the mavlink system id of the vehicle (normally "1")
- **target_components** : normally "0"
- **param_id** : the human readable 16 character parameter name
- **param_value** : the new value for the parameter expressed as a float
- **param_type** : the `MAV_PARAM_TYPE <https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE>`__ for the parameter.  This must match the type of the parameter within the AutoPilot and will be MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32 or MAV_PARAM_TYPE_REAL32 (the only types that ArduPilot supports).  It may be convenient to store the "param_type" field from the `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message sent by the flight controller for this parameter.

If the parameter value is successfully updated the vehicle will respond with a `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message with the "param_value" field set to the new value.

Hidden Parameters
-----------------

In order to reduce the time spent downloading parameters, ArduPilot hides parameters for some subsystems if they are disabled.

For example, users can choose to use either EKF2 or EKF3 but the EKF3 is disabled by default so all but the EK3_ENABLE parameter are hidden.  Once the EK3_ENABLE parameter is set to "1" the remaining EK3_xxx parameters become visible.

From a ground station or companion computer point of view this means that the total number of parameters may change (i.e. the `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ param_count field may increase but also note that the **param_index** for a given parameter will not change.

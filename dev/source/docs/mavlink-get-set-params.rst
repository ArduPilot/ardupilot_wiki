.. _mavlink-get-set-params:

==============================
Getting and Setting Parameters
==============================

ArduPilot has hundreds of parameters which allow the user to configure many aspects of how the vehicle flies/drives including attitude controller gains, the minimum safe altitude to return home at when flying in Return-To-Launch mode, etc.

This page explains how a ground station or companion computer can retrieve the list of parameters from the flight controller and then make changes to them.  For reference the general purpose `MAVLink parameter protocol is described here <https://mavlink.io/en/services/parameter.html>`__.

Most ground stations (aka GCS) provide an editable list of parameters to the user to allow them to customise the vehicle.  Below are links to the human and machine readable parameter names, descriptions and valid ranges but please note that the parameters are different depending upon the vehicle type and also change slightly with each firmware release so GCSs and companion computers should retrieve the list of parameters directly from the flight controller.  Many GCSs also parse the machine readable parameter list (linked below) to provide the description and valid range to the user.

+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+
| Vehicle       | Human readable                                                     | Machine Readable                                                                       |
+===============+====================================================================+========================================================================================+
|AntennaTracker |`html <https://ardupilot.org/antennatracker/docs/parameters.html>`__|`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/AntennaTracker/apm.pdef.xml>`__|
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+
|Copter         |`html <https://ardupilot.org/copter/docs/parameters.html>`__        |`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.xml>`__    |
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+
|Plane          |`html <https://ardupilot.org/plane/docs/parameters.html>`__         |`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.xml>`__     |
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+
|Rover          |`html <https://ardupilot.org/rover/docs/parameters.html>`__         |`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/Rover/apm.pdef.xml>`__         |
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+
|Submarine      | unavailable                                                        |`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/ArduSub/apm.pdef.xml>`__       |
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+

Retrieving All Parameters
-------------------------

The most efficient way to receive all parameters is to send a `PARAM_REQUEST_LIST <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST>`__ message with the following fields:

- **target_system** : the MAVLink system id of the vehicle (normally "1")
- **target_components** : normally "1"

The vehicle should respond with many individual `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ messages (one for each parameter in the system).  The fields of the `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ messages are:

- **param_id** : the parameter name, a 16 character string (null terminated if less than 16 characters)
- **param_value** : the parameter value (all parameter values are sent as floats)
- **param_type** : the `MAV_PARAM_TYPE <https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE>`__.  Note that ArduPilot only supports MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32 and MAV_PARAM_TYPE_REAL32 (aka float) types.
- **param_count** and **param_index** fields should be used to determine if all parameters have been successfully received

MAVLink does not guarantee delivery of packets so if some parameter values do not arrive, use the instructions below to re-request the missing parameters.

Retrieving a Parameter
----------------------

To retrieve just a single parameter send a `PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`__ message with the following fields:

- **target_system** : the MAVLink system id of the vehicle (normally "1")
- **target_components** : normally "1"
- **param_id** : the human readable 16 character parameter name if known.  If not known, this field can be anything as long as the param_index is set to a non-negative number
- **param_index** : the parameter index number or -1 if the "param_id" field should be used instead.  Note that it is better to use the param_id field because this index may change if "hidden" parameters are exposed (see below).

The vehicle should respond with a `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message as described in the section above.

Setting a Parameter
-------------------

To set the value of a single parameter send a `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`__ message with the following fields:

- **target_system** : the MAVLink system id of the vehicle (normally "1")
- **target_components** : normally "1"
- **param_id** : the human readable 16 character parameter name
- **param_value** : the new value for the parameter expressed as a float
- **param_type** : the `MAV_PARAM_TYPE <https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE>`__ for the parameter.  This must match the type of the parameter within the AutoPilot and will be MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32 or MAV_PARAM_TYPE_REAL32 (the only types that ArduPilot supports).  It may be convenient to store the "param_type" field from the `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message sent by the flight controller for this parameter.

If the parameter value is successfully updated the vehicle will respond with a `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message with the "param_value" field set to the new value.

If the save is unsuccessful because the parameter is a read-only parameter (like MISSION_COUNT), the vehicle will respond with **two** messages.  A `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message with the "param_value" field set to the new value and a follow-up `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ with the "param_value" field set to the original value.  This odd behaviour was implemented to deal with the bad behaviour of some GCSs that would try over and over to set the parameter until they received the number they were expecting.

Hidden Parameters
-----------------

In order to reduce the time spent downloading parameters, ArduPilot hides parameters for some subsystems if they are disabled.

For example, users can choose to use either EKF2 or EKF3 but the EKF3 is disabled by default so all but the EK3_ENABLE parameter are hidden.  Once the EK3_ENABLE parameter is set to "1" the remaining EK3_xxx parameters become visible.

From a ground station or companion computer point of view this means that the total number of parameters will change.  i.e. the `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ **param_count** field may increase but also note that the **param_index** for a given parameter may change.  This means it is best to not rely on the **param_index** field.

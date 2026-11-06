.. _mavlink-get-set-params:

==============================
Getting and Setting Parameters
==============================

ArduPilot has hundreds of parameters which allow the user to configure many aspects of how the vehicle flies/drives including attitude controller gains, the minimum safe altitude to return home at when flying in Return-To-Launch mode, etc.

This page explains how a ground station or companion computer can retrieve the list of parameters from the flight controller and then make changes to them.  For reference the general purpose `MAVLink parameter protocol is described here <https://mavlink.io/en/services/parameter.html>`__.

Most ground stations (aka GCS) provide an editable list of parameters to the user to allow them to customise the vehicle.  Below are links to the human and machine readable parameter names, descriptions and valid ranges but please note that the parameters are different depending upon the vehicle type and also change slightly with each firmware release so GCSs and companion computers should retrieve the list of parameters directly from the flight controller.  Many GCSs also parse the machine readable parameter list (linked below) to provide the description and valid range to the user.

+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+------------------------------------------------------------------------------------------+
| Vehicle       | Human readable                                                     | Machine Readable (XML)                                                                 | Machine Readable (JSON)                                                                  |
+===============+====================================================================+========================================================================================+==========================================================================================+
|AntennaTracker |`html <https://ardupilot.org/antennatracker/docs/parameters.html>`__|`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/AntennaTracker/apm.pdef.xml>`__|`apm.pdef.json <https://autotest.ardupilot.org/Parameters/AntennaTracker/apm.pdef.json>`__|
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+------------------------------------------------------------------------------------------+
|Copter         |`html <https://ardupilot.org/copter/docs/parameters.html>`__        |`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.xml>`__    |`apm.pdef.json <https://autotest.ardupilot.org/Parameters/ArduCopter/apm.pdef.json>`__    |
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+------------------------------------------------------------------------------------------+
|Plane          |`html <https://ardupilot.org/plane/docs/parameters.html>`__         |`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.xml>`__     |`apm.pdef.json <https://autotest.ardupilot.org/Parameters/ArduPlane/apm.pdef.json>`__     |
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+------------------------------------------------------------------------------------------+
|Rover          |`html <https://ardupilot.org/rover/docs/parameters.html>`__         |`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/Rover/apm.pdef.xml>`__         |`apm.pdef.json <https://autotest.ardupilot.org/Parameters/Rover/apm.pdef.json>`__         |
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+------------------------------------------------------------------------------------------+
|Submarine      |`html <https://ardupilot.org/sub/docs/parameters.html>`__           |`apm.pdef.xml <https://autotest.ardupilot.org/Parameters/ArduSub/apm.pdef.xml>`__       |`apm.pdef.json <https://autotest.ardupilot.org/Parameters/ArduSub/apm.pdef.json>`__       |
+---------------+--------------------------------------------------------------------+----------------------------------------------------------------------------------------+------------------------------------------------------------------------------------------+


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

Downloading All Parameters With MAVFTP
--------------------------------------

A much faster way to download the full parameter list is to read the ``@PARAM/param.pck`` file over :ref:`MAVFTP <mavlink-mavftp>`.  This is a virtual file which holds every parameter's name, type and value in a packed binary format.  It has been available since ArduPilot 4.1.

- ``@PARAM/param.pck?start=50&count=10`` returns only 10 parameters starting from parameter number 50
- ``@PARAM/param.pck?withdefaults=1`` also includes each parameter's default value where it differs from the current value (ArduPilot 4.3 and later)

All reads on one open file handle must use the same read size.  Without ``withdefaults`` the file is padded so that a parameter value is never split across two MAVFTP blocks.  With ``withdefaults=1`` only the default value is kept whole, so a decoder should join the blocks back into one stream before unpacking.  A change of read size after the first read fails.

The packed format is described in the `AP_Filesystem README <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Filesystem/README.md>`__.  `param_unpack.py <https://github.com/ArduPilot/ardupilot/blob/master/Tools/scripts/param_unpack.py>`__ unpacks a plain download (it does not read a file fetched with ``withdefaults=1``) and pymavlink's ``mavftp.py`` has a ``getparams`` command which downloads and decodes both.  Since ArduPilot 4.1 parameters can also be uploaded in bulk by writing a packed file to ``@PARAM/param.pck`` over MAVFTP, which MAVProxy's ``param ftpload`` command uses.

Retrieving a Parameter
----------------------

To retrieve just a single parameter send a `PARAM_REQUEST_READ <https://mavlink.io/en/messages/common.html#PARAM_REQUEST_READ>`__ message with the following fields:

- **target_system** : the MAVLink system id of the vehicle (normally "1")
- **target_components** : normally "1"
- **param_id** : the human readable 16 character parameter name if known.  If not known, this field can be anything as long as the param_index is set to a non-negative number
- **param_index** : the parameter index number or -1 if the "param_id" field should be used instead.  Note that it is better to use the param_id field because this index may change if "hidden" parameters are exposed (see below).

The vehicle should respond with a `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message as described in the section above.

From ArduPilot 4.7, if no parameter has the requested name or index the vehicle responds with a `PARAM_ERROR <https://mavlink.io/en/messages/common.html#PARAM_ERROR>`__ message with the "error" field set to MAV_PARAM_ERROR_DOES_NOT_EXIST instead.

Setting a Parameter
-------------------

To set the value of a single parameter send a `PARAM_SET <https://mavlink.io/en/messages/common.html#PARAM_SET>`__ message with the following fields:

- **target_system** : the MAVLink system id of the vehicle (normally "1")
- **target_components** : normally "1"
- **param_id** : the human readable 16 character parameter name
- **param_value** : the new value for the parameter expressed as a float
- **param_type** : the `MAV_PARAM_TYPE <https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE>`__ for the parameter.  This must match the type of the parameter within the AutoPilot and will be MAV_PARAM_TYPE_INT8, MAV_PARAM_TYPE_INT16, MAV_PARAM_TYPE_INT32 or MAV_PARAM_TYPE_REAL32 (the only types that ArduPilot supports).  It may be convenient to store the "param_type" field from the `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message sent by the flight controller for this parameter.

If the parameter value is successfully updated the vehicle will respond with a `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message with the "param_value" field set to the new value.

From ArduPilot 4.7, if the parameter cannot be set the vehicle responds with a `PARAM_ERROR <https://mavlink.io/en/messages/common.html#PARAM_ERROR>`__ message whose "error" field gives the reason:

- **MAV_PARAM_ERROR_DOES_NOT_EXIST** : no parameter has that name
- **MAV_PARAM_ERROR_VALUE_OUT_OF_RANGE** : the value sent is not a number (NaN) or is infinite
- **MAV_PARAM_ERROR_PERMISSION_DENIED** : the parameter is read-only (like MIS_TOTAL).  The vehicle also sends a "Param write denied" STATUSTEXT and a `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message with the "param_value" field set to the unchanged value

If a script has turned off parameter setting from the ground station, the firmware itself replies to a set of any existing parameter to a finite value with only a `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message with the "param_value" field set to the unchanged value, with no `PARAM_ERROR <https://mavlink.io/en/messages/common.html#PARAM_ERROR>`__ message and no STATUSTEXT.  The script may send its own replies or set the value itself, as the param-lockdown applet does.

Before 4.7 no `PARAM_ERROR <https://mavlink.io/en/messages/common.html#PARAM_ERROR>`__ message is sent.  An unknown parameter name gets no reply at all.  A value that is NaN or infinite is also ignored without a reply.  A read-only parameter gets the "Param write denied" STATUSTEXT and a single `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ message with the "param_value" field set to the unchanged value.

To decode `PARAM_ERROR <https://mavlink.io/en/messages/common.html#PARAM_ERROR>`__ a ground station or script needs a MAVLink library that includes it (for example pymavlink 2.4.50 or later).  `PARAM_ERROR <https://mavlink.io/en/messages/common.html#PARAM_ERROR>`__ is a MAVLink 2 message, so on a link still using MAVLink 1 the vehicle replies as it did before 4.7.

Hidden Parameters
-----------------

In order to reduce the time spent downloading parameters, ArduPilot hides parameters for some subsystems if they are disabled.

For example, users can choose to use QuadPlane functionality in ArduPlane, but Q_ENABLE is disabled by default so all its parameters are hidden.  Once the Q_ENABLE parameter is set to "1" the remaining Q_xxx parameters become visible.

From a ground station or companion computer point of view this means that the total number of parameters will change.  i.e. the `PARAM_VALUE <https://mavlink.io/en/messages/common.html#PARAM_VALUE>`__ **param_count** field may increase but also note that the **param_index** for a given parameter may change.  This means it is best to not rely on the **param_index** field.

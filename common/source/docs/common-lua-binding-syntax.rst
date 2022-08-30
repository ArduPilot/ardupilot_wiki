.. _common-lua-binding-syntax:

[copywiki destination="plane,copter,rover,dev"]
=====================================
Decoding the LUA Binding Descriptions
=====================================

This is not exhaustive, nor complete, but attempts to help users decode the use of bindings that have not yet been fully documented in the :ref:`common-lua-scripts` page.

Aliases
=======

Most methods have an alias defined to specify its binding group. For example, the calls bound from the AP_AHRS library, have an alias defined as "ahrs" by the description line in its group:

``singleton AP_AHRS alias ahrs``

so that using its call to retrieve current roll is accessed:
::

    roll = ahrs:get_roll()

Methods
=======

The majority of bindings are methods used to get information into and out of the running ArduPilot firmware. In the `bindings.desc <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/generator/description/bindings.desc>`_ file listing these, each are grouped by their association with the underlying ArduPilot code module's "\*.h"header file. A method usually has the form of:

**singleton <binding group name> method <method name> <function type> <argument types/ranges>**

- <method name> is the name of the function to call
- <function type> type of C++ function that is being called.
- <argument types/ranges> variable type returned or, type and range for variable used as input to C++ function being called. These are best understood by referencing the C++ function in the code being called and correlating them to the function's arguments.

Examples
--------
Perhaps the easiest way to describe the above is to show some examples, in order of increasing complexity of arguments:

In the AP_Arming group:
~~~~~~~~~~~~~~~~~~~~~~~

``singleton AP_Arming method is_armed boolean``

binds the public variable in AP_Arming.h:
::

 bool is_armed();
 returning a boolean for armed

In the AP_Baro group:
~~~~~~~~~~~~~~~~~~~~~

``singleton AP_Baro method get_temperature float``

binds to the function in AP_Baro.h:
::

  float get_temperature(void) const { return get_temperature(_primary); }
  returning a float of the baro temperature for primary barometer

In the AP_Relay group:
~~~~~~~~~~~~~~~~~~~~~~

``singleton AP_Relay method on void uint8_t 0 AP_RELAY_NUM_RELAYS``

binds to the function in AP_Relay.h:
::

 void on(uint8_t instance) { set(instance, true); }
 returning nothing, but setting Relay instance ON, where instance
 is an unsigned 8 bit integer between 0 and AP_RELAY_NUM_RELAYS (currently 6)

``singleton AP_Relay method enabled boolean uint8_t 0 AP_RELAY_NUM_RELAYS``

binds to:
::

  bool enabled(uint8_t instance) { return instance < AP_RELAY_NUM_RELAYS && _pin[instance] != -1; }
  returning a boolean if the relay instance provided has been setup

In the AP_AHRS group:
~~~~~~~~~~~~~~~~~~~~~

An example that takes advantage that a LUA variable can have a value or be NULL , having no value:

``singleton AP_AHRS method get_variances boolean float'Null float'Null float'Null``
``Vector3f'Null float'Null``

binding to:
::

    // get_variances - provides the innovations normalized using the innovation variance where a value of 0
    // indicates perfect consistency between the measurement and the EKF solution and a value of 1 is the maximum
    // inconsistency that will be accepted by the filter
    // boolean false is returned if variances are not available
    virtual bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const {
        return false;
    returning multiple values, any of which can be NULL.
    In this case the C++ function is boolean, but since the returns have 'NULL specified,
    the C++ return of a boolean is ignored and the returned values can have NULL value.
    Note that this call returns many values (arguments are variable pointers) and has no input 
    variables.

In order to see how this would be used:
:: 

      velVar, posVar,hgtVar,magVar, tasVar = ahrs:get_variances()
      if velVar then
        newVar=2*velVar 
      end

note that velVar could have a value of 0 and still the ``if`` statement test would be true in the LUA script.

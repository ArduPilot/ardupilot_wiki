.. _common-analog-pins:

=================
Analog Input Pins
=================

In addition to digital input and output pins, autopilots have analog input pins that allow measuring voltages precisely. These are commonly used for the power monitor voltge and current sense inputs, but other pins are sometimes provided for:

- RSSI (Received Signal Strength Input)
- Analog Airspeed Sensor Input
- Analog Fuel Level Sensor Input
- Analog Sonar/Rangefinder Input
- Analog Temperature Sensor Input

.. note:: all pins can accept 0-3.3v inputs. Unless the documentation (OEM or Wiki) specifically specifies 6.6V input range, only apply 3.3 maximum!

When any of these inputs are enabled by the appropriate sensor type selection, a ``_PIN`` parameter will need to be entered to designate which "pin" on the autopilot is to be used. Often, the correct pin number for the autopilot's analog inputs will be documented either on its Wiki page here, or the OEM's documentation.

If not, then examining the autopilot's ``hwdef.dat`` file `here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef>`__ can provide the information. The process is a bit difficult but manageable.

For example, the `CUAV-X7 autopilot hwdef.dat <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/CUAV-X7/hwdef.dat>`__ has a section designating its internal A-to-D converters:

::
    
    # analog in
    PA0 BATT_VOLTAGE_SENS ADC1 SCALE(1)
    PA1 BATT_CURRENT_SENS ADC1 SCALE(1)

    # ADC3.3/ADC6.6
    PC4 SPARE1_ADC1 ADC1 SCALE(1)

    # Note that this should be SCALE(2), but we don't want to break existing setups
    # See: https://github.com/ArduPilot/ardupilot/pull/22831
    PA4 SPARE2_ADC1 ADC1 SCALE(1)

    PF12 RSSI_IN ADC1 SCALE(1)

    PC5 VDD_5V_SENS ADC1 SCALE(2)
    PC1 SCALED_V3V3 ADC1 SCALE(2)

    PB1 FMU_SERVORAIL_VCC_SENS ADC1 SCALE(3)


The last three lines define inputs used internally on the board to monitor various board supply voltages. The first two lines define the voltage and current sense inputs, the next define user analog inputs and the RSSI input. In order to determine which pins numbers are used in ArduPilot parameters, one more lookup step is required:

- After determining which cpu is used in the autopilot, the pin numbers corresponding to the cpu pins can be determined from the `cpu's definition table here <https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_HAL_ChibiOS/hwdef/scripts>`__. In the above example, the cpu is STM32H743, and looking at the bottom of its `definition file <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/scripts/STM32H743xx.py>`__, you can find its cpu to ardupilot pin tables:

::

    ADC1_map = {
    # format is PIN : ADC1_CHAN
    "PF11"  :   2,
    "PA6"	:	3,
    "PC4"	:	4,
    "PB1"	:	5,
    "PF12"  :   6,
    "PA7"	:	7,
    "PC5"	:	8,
    "PB0"	:	9,
	"PC0"	:	10,
	"PC1"	:	11,
	"PC2"	:	12,
    "PC3"	:	13,
    "PA2"	:	14,
    "PA3"	:	15,
    "PA0"	:	16,
    "PA1"	:	17,
    "PA4"	:	18,
    "PA5"	:	19,
    }


and for ADC1, the power monitor voltage input, PA0, would be ArduPilot pin "16", the RSSI input, PF12, would correspond to ArduPilot pin "6", etc. and these pin values would be used in the appropriate ``_PIN`` parameter.
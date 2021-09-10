.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


in the :ref:`common-gpios` page, add the following note at the top:

.. note:: in firmware versions later than 4.1, the method for setting a PWM/SERVO/MOTOR outputs to be a GPIO function is changed. Instead of ``BRD_PWM_COUNT`` being used, the individual ``SERVOx_FUNCTION`` parameter is merely set to "-1". If set to "0", it remains a PWM output, unassigned to a function, and outputs that output's trim value when board safety is not active.

.. _common-future-wiki-changes:

==========================================
Future Wiki Changes (in Master branch now)
==========================================


in the :ref:`common-gpios` page, add the following note at the top:

.. note:: in firmware versions later than 4.1, the method for setting a PWM/SERVO/MOTOR outputs to be a GPIO function is changed. Instead of ``BRD_PWM_COUNT`` being used, the individual ``SERVOx_FUNCTION`` parameter is merely set to "-1". If set to "0", it remains a PWM output, unassigned to a function, and outputs that output's trim value when board safety is not active.

Plane
=====

:ref:`guide-tailsitter` page:
-----------------------------

- at top of  page:

Tailsitters and their parameters are enabled by setting :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` to either "1" ,for most tailsitters, or "2" for the special case of Copter Motor Only Tailsitters (those without control surfaces like elevons or ailerons/elevators).

- in Vectored and non-Vectored change:

 " CopterMotor tailsitters without them (ie. only have a lifting wing with no control surfaces) must use QASSIST (discussed below) to provide control while in fixed wing flight modes." to:**"                CopterMotor tailsitters without them (ie. only have a lifting wing with no control surfaces) must use always use their motors to provide control while in fixed wing flight modes. Setting** :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` = 2 **automatically does this."**

- under Tailsitter Configuration change:

 "The key to make a QuadPlane a tailsitter is to either set Q_FRAME_CLASS =10 or Q_TAILSIT_MOTMX non-zero. That tells the QuadPlane code to use the tailsitter VTOL backend." to:**"The key to make a QuadPlane a tailsitter is to either set** :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` **to "1" or "2" to tell the QuadPlane code to use the tailsitter VTOL backend."**

:ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>` **determines the number and layout of VTOL motors and** :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>` **determines which motors are active when in fixed wing modes, except in the special case of the Copter Motor Only Tailsitter which keeps running the motors like a Copter mode even when flying in a fixed wing mode for control surface-less Copter tailsitters (ie always running the motors to provide attitude control, even at low throttle).**

- Add table at end of this section:

+-------------------+------+----------------+-------------+--------------+-----------------------+
|Tailsitter Style   |ENABLE| CLASS          |  TYPE       |  MOTORMASK   | Motor Output Functions+
+===================+======+================+=============+==============+=======================+
|3D Single Motor    |  1   | 10(Single/Dual)|  NA         | 0            | Throttle              |
+-------------------+------+----------------+-------------+--------------+-----------------------+
|Twin Motor and Twin|  1   | 10(Single/Dual)|  NA         | 0            | Left Throttle,        |
|Motor Vectored     |      |                |             |              | Right Throttle        |
+-------------------+------+----------------+-------------+--------------+-----------------------+
|Copter Tailsitters |  1   |to match number | to match    |active motors |   Motor 1- Motor x    |
|with fixed wing    |      |of VTOL motors  | motor mixing|in fixed wing |                       |
|control surfaces   |      |                |             |modes         |                       |
+-------------------+------+----------------+-------------+--------------+-----------------------+
|Copter Tailsitters |  2   |to match number | to match    |active motors |   Motor 1- Motor x    |
|with no fixed wing |      |of VTOL motors  | motor mixing|in fixed wing |                       |
|control surfaces   |      |                |             |modes         |                       |
+-------------------+------+----------------+-------------+--------------+-----------------------+

The ENABLE column refers to the :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` parameter, while CLASS,TYPE, and MOTORMASK refer to :ref:`Q_FRAME_CLASS<Q_FRAME_CLASS>`,:ref:`Q_FRAME_TYPE<Q_FRAME_TYPE>`, and :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>`, respectively.

- under Copter Tailsitters change the note to read:

.. note:: It is possible to have a CopterMotor Tailsitter using no fixed wing control surfaces, ie basically a quadcopter with a wing. For that configuration, use :ref:`Q_TAILSIT_ENABLE<Q_TAILSIT_ENABLE>` = 2. :ref:`Q_TAILSIT_MOTMX<Q_TAILSIT_MOTMX>` is ignored in that case.

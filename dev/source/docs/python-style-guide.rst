.. _python-style-guide:

============================
ArduPilot Python Style Guide
============================

In order to improve the readability of the ArduPilot code, and help new
users pick up the code more quickly, please use the following styles. Code submitted may be rejected if these styles are not followed.

.. note::

   Some parts of the code may not conform due to historic reasons but
   new additions should!

Supported Python Versions
=========================

ArduPilot supports Python versions back to 3.9 - scripts should be written with an eye to working back that far.

Method Names
============

ArduPilot's Python has not traditionally used ``_`` as a hint that a member is intended for internal use.

They should be omitted until we adopt the convention.

Type Annotations
================

New functions should have type annotations.

If you are modifying a function's signature please consider adding type annotations.

**Right:**

::

    def set_parameter_bit(self, name: str, bit_offset: int) -> None:
        '''set bit in parameter to true, preserving values of other bits'''
        value = int(self.get_parameter(name))
        value |= 1 << bit_offset
        self.set_parameter(name, value)

Automated Linting
=================

Generally ``ruff`` is used to lint the codebase.

It is often triggered via ``pre-commit``.  You can use ``pre-commit run --all-files`` to validate your changes are correct. This check is enforced on CI.

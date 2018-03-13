.. _style-guide:

=====================
Ardupilot Style Guide
=====================

In order to improve the readability of the ArduPilot code and help new
users pick up the code quickly please use the following styles.

.. note::

   Some parts of the code may not conform due to historic reasons but
   new additions should!

*astyle* is our tool of choice for formatting the code.  Whilst we
aren't doing it automatically at the moment in the Tools/CodeStyle
directory there is an astylerc file that can be used to format the code
automatically according to the guidelines below.

::

    astyle --options=Tools/CodeStyle/astylerc <filename>

Remember any formatting changes to a file should be done as a separate
commit.

Case
====

Variables and function names are lower case with words separated by an
underscore.

**Right:**

::

    throttle_input

**Wrong:**

::

    ThrottleInput

Indentation
===========

Spaces
------

Indent using 4 spaces everywhere. Do not use tabs.

**Right:**

::

    int foo()
    {
        return 0;
    }

**Wrong:**

::

    int foo()
    {
      return 0;
    }

Case statements
---------------

The case label indentation is acceptable either 
when lined up with the switch or indented once.

**Right:**

::

    switch (condition) {
        case foo_cond:
            foo();
            break;
        case bar_cond:
            bar();
            break;
    }

**Right:**

::

    switch (condition) {
    case foo_cond:
        foo();
        break;
    case bar_cond:
        bar();
        break;
    }

**Wrong:**

::

    switch (condition) {
    case foo_cond:
        foo();
    break;
    case bar_cond:
        bar();
    break;
    }

Spacing
=======

Unary operators
---------------

Do not place spaces around unary operators.

**Right:**

::

    i++;

**Wrong:**

::

    i ++ ;

Control statements
------------------

Place spaces between control statements and their parentheses.

**Right:**

::

    if (condition) {
        foo();
    }

**Wrong:**

::

    if(condition) {
        foo();
    }

::

    if (condition){
        foo();
    }

Function calls
--------------

Do not place spaces between a function and its parentheses, or between a
parenthesis and its content.

**Right:**

::

    foo(a, 10);

**Wrong:**

::

    foo (a, 10);

::

    foo(a, 10 );

Trailing whitespaces
--------------------

Don't leave trailing whitespace on new code (a good editor can manage
this for you). Fixing whitespace on existing code should be done as a
separate commit (do not include with other changes).

Line breaks
===========

Single statements
-----------------

Each statement should get its own line except in method implementations in header files which may (or may not be) on a single lines.

**Right:**

::

    x++;
    y++;
    if (condition) {
        foo();
    }

**Wrong:**

::

    x++; y++;
    if (condition) foo();

**Right:**

::

     bool requires_GPS() const override { return false; }

Else statement
--------------

An ``else`` statement should go on the same line as a preceding close
brace.

**Right:**

::

    if (condition) {
        foo();
    } else {
        bar();
    }

**Wrong:**

::

    if (condition) {
        foo();
    }
    else {
        bar();
    }

Braces
======

Function braces
---------------

Functions definitions: place each brace on its own line. For methods
inside a header file, braces can be inline.

Control statements
------------------

Control statements (``if``, ``while``, ``do``, ``else``) should always
use braces around the statements.

**Right:**

::

    if (condition) {
        foo();
    } else {
        bar();
    }

**Wrong:**

::

    if (condition)
        foo();
    else 
        bar();

Other braces
------------

Place the open brace on the line preceding the code block; place the
close brace on its own line.

**Right:**

::

    class My_Class {
        ...
    };

    namespace AP_HAL {
        ...
    }

    for (int i = 0; i < 10; i++) {
        ...
    }

**Wrong:**

::

    class My_Class 
    {
        ...
    };

Names
=====

Private members
---------------

Private members in classes should be prefixed with an underscore:

**Right:**

::

    class My_Class {
    private:
        int _field;
    };

**Wrong:**

::

    class My_Class {
    private:
        int field;
    };

Class names
-----------

Class names should capitalise each word and separate them using
underscores.

**Right:**

::

    class AP_Compass { };

**Wrong:**

::

    class ap_compass { };

Functions and variables
-----------------------

Functions that return a single physical value or variables that represent a physical value should be suffixed by the physical unit.

**Right:**

::
    uint16 get_angle_rad() { ... };
    float distance_m;

**Wrong:**

::

    uint16 get_angle() { ... };
    float distance;

Functions or variables that represent a value relative to a frame should be suffixed with the frame first, then with the physical unit (if any).

**Right:**

::
    uint16 get_distance_ned_cm() { ... };
    uint16 get_distance_enu_m() { ... };
    float position_neu_mm;

**Wrong:**

::

    uint16 get_distance() { ... };
    float position;


Commenting
==========

Each file, function and method with public visibility should have a
comment at the top describing what it does.

Parameter Naming
================

Parameter with multiple words should have the words ordered from left to right by importance:

- the flight mode, feature or sensor should be the first word.  I.e. a parameter relevant only to the RTL flight mode should start with "RTL" like "RTL_ALT".
- qualifiers like "MIN", "MAX" or units (in the rare case they appear in the name) should be on the far right.  I.e RTL_ALT_MIN is better than RTL_MIN_ALT.

Re-use words from other parameters if possible instead of creating new words.  For eample we use "MIN" and "MAX" so these should be used instead of equivalent words like "TOP" and "BOTTOM".

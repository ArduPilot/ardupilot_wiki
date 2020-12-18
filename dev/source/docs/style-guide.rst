.. _style-guide:

=====================
ArduPilot Style Guide
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

Parameters
==========

Users gather critical information from these fields. With parameters that are well-documented in the code, the wiki and GCS can update parameters automatically. When parameters are properly documented, users can often tune their vehicles without needing pages and posts of external non-linked documentation. While information here is not a substitute for a tuning guide, it can be very effective at guiding users to change the right things.

Parameter with multiple words should have the words ordered from left to right by importance:

- the flight mode, feature or sensor should be the first word.  I.e. a parameter relevant only to the RTL flight mode should start with "RTL" like "RTL_ALT".
- qualifiers like "MIN", "MAX" or units (in the rare case they appear in the name) should be on the far right.  I.e RTL_ALT_MIN is better than RTL_MIN_ALT.

Re-use words from other parameters if possible instead of creating new words.  For example we use "MIN" and "MAX" so these should be used instead of equivalent words like "TOP" and "BOTTOM".

Parameters should be in the standard unit (meters for distances, degrees for angles) but in cases where they are not the unit may (optionally) be appended to the end.  This is definitely not a requirement but is up to the developer.

The total length of the parameter name must be 16 characters or less.


Display Name
------------

The display name is typically a 2-5 word phrase that describes what the parameter changes. Often this is the Parameter Name spelled out in full words. Do not start with nondescriptive word like "the." A good Display Name for LIM_PITCH_MAX is "Maximum Pitch Angle".


Description
-----------

The description is a long text field for a complete description of the parameter and how changing it may affect vehicle behavior. It should be kept concise while giving the most critical information to the user.

**Right:**

::

    // @Description: Gain added to pitch to keep aircraft from descending or ascending in turns. Increase in increments of 0.05 to reduce altitude loss. Decrease for altitude gain.

**Wrong:**

::

    // @Description: This is the gain term that is applied to the pitch rate offset calculated as required to keep the nose level during turns. The default value is 1 which will work for all models. Advanced users can use it to correct for height variation in turns. If height is lost initially in turns this can be increased in small increments of 0.05 to compensate. If height is gained initially in turns then it can be decreased.


Avoid in Descriptions:

- Helping words and nondescriptive language such as "This parameter changes..., you, etc." that is common to all parameters
- Referencing other parameters unless it is critical
- Describing a 0 setting as "disabled"
- Default settings

Encourage in Descriptions:

- Present tense language
- Consequences of changing the parameter (this also guides users how to tune for their vehicle)
- When the parameter is used or ignored
- When a 0 setting uses another parameter for the value


Value, Unit, Range
------------------

The values, units, ranges, and steps are all critical for adjusting the parameter as well. Include them when possible.


User
----

The user field helps to categorize and hide advanced parameters from being adjusted by new users. There are currently 2 user fields:

- Standard - Available to anyone
- Advanced - Available to advanced users

Floating Point Annotation
=========================

ArduPilot is compiled with ``-fsingle-precision-constant``.

That means it is currently allowable to leave off the float specifier from constants.  It is also permissable to have them.

**Right**

::

   1.0f
   1.0

Multiplication vs Division
==========================

Use multiplication rather than division where possible:

**Right**

::

   const float foo_m = foo_cm * 0.01;

**Wrong**

::

   const float foo_m = foo_cm / 100;

Multiplications typically take fewer cycles to compute than divisions.


ArduPilot Design Decisions
==========================

Several design decisions have been in the ArduPilot codebase to accomodate its embedded nature which may surprise some programmers.

Implicit Zeroing of Memory
--------------------------

Implicitly zeroing of memory gives us more consistent (even-if-bad) behaviour, and saves us flash space as most places in the code don't need to initialise the memory they've allocated.  The only memory you MUST zero is stack-stored variables - locals, asprintf and the like.

   - new and malloc both zero their memory
   - bss-stored data does not need to be zeroed (so no members in a singleton object need to be zeroed
   - Vectors are special and zero themelves - even on the stack
   - static variables within a function (which we generally frown upon) do not need to be zeroed


Bit fields are generally frowned upon
-------------------------------------

Using bit fields reduces RAM usage but can considerably increase flash usage, as to extract a boolean truth value from a bit field requires more machine instructions.  If the variable is frequently accessed then this can be a LOT of flash.

**Not preferred:**

::

   class Foo() {
   private:
       bool should_fly  : 1;
       bool should_grow : 1;
   };

**Preferred:**

::

   class Foo() {
   private:
       bool should_fly;
       bool should_grow;
   };


Initialise member variables in header files rather than in constructors
-----------------------------------------------------------------------

Where a member isn't dependent on a constructor parameter, we prefer to do in-class-definition initialisation.

**Not preferred:**

::

    Foo::Foo() :
      bar(37),
      baz(BAZ_DEFAULT)
    {
      ...
    }

**Preferred:**

::

   class Foo() {
   public:
   ...
   private:
     uint8_t bar = 37;
     float baz = BAZ_DEFAULT;
   };


No Standard Library
-------------------

For efficiency reasons, ArduPilot doesn't use the C standard library (``std::``).  We also prefer to use functions which are consistent across platforms to ease support (not using 64-bit maths on platforms that support it is useful, for example).

This means no ``std::vector``, no ``std::string`` and no ``std::unordered_map``, for example.

We try to avoid library calls that handle their own allocations - but if you really want to go that way, including the header to get these isn't sufficient - you will need to fiddle with the build system to link ``std`` in.

Alternatives to ``std::vector``
...............................

Most of the time fixed-length arrays are used.

``AP_ExpandingArray`` may be an option for you - but expanding in-flight might be a bad thing.

Linked lists are used on some places.


Alternatives to ``std::string``
...............................

``asprintf`` is used in some places.  Generally simply using ``char*`` is adequate.

Alternatives to ``std::unordered_map``
......................................

Create an array of structs and iterate it.  This is done when turning a mavlink id into an ap_message id, for example.

Create an ordered array of structs and bisect-search it.

Create an array of structs with a perfect hash.

No Dead Code
------------

We don't keep dead code in ArduPilot.  If code is unused, it should be removed - not just commented out.  This is a general rule and not universally adhered to.

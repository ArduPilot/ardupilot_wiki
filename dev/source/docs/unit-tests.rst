.. _ardupilot-unit-tests:

====================
ArduPilot Unit Tests
====================

ArduPilot's Unit Test suite allows for the creation of repeatable tests
which help prevent regressions in ArduPilot's internal functions.  It is based
on Google's `GTest Framework <https://https://github.com/google/googletest>`__.

Using ArduPilot's Unit Tests can:

   - make your development process more efficient by reducing time spent repeatedly running the same tests to force the same inputs to a function time and again
   - allow you to repeatedly replicate bad behaviour in an ArduPilot function, and possibly ship that test to a developer capable of fixing the issue ("test-driven-development")
   - reduce the chances of a regression in an ArduPilot function's behaviour by locking in tests for that function's behaviour

Overview
========

The Unit test suite is run on ArduPilot's autotest server on most
commits to the master branch, but can be run locally to vet software
changes.  Adding tests is straight-forward and encouraged to prove a
function's correctness after a change.


Running Unit Tests
==================

The Unit Tests require an environment in which the ArduPilot executables can be compiled.  The easiest way to obtain this is to create a SITL testing environment.  Use the SITL instructions (:ref:`SITL <using-sitl-for-ardupilot-testing>`) to obtain a valid environment.  It is suggested the ArduPilot Vagrant virtual machine configuration files be used to obtain a working environement.

Invocation
----------

.. note::

   These tests work well under Linux.  Operation under different environments is unknown.

The unit tests are compiled using the `waf` build tool:

::

    pbarker@bluebottle:~/rc/ardupilot(master)$ ./waf configure --board=linux --debug
    Setting top to                           : /home/pbarker/rc/ardupilot 
    Setting out to                           : /home/pbarker/rc/ardupilot/build 
    Autoconfiguration                        : enabled 
    .
    .
    .
    Scripting runtime checks                       : enabled 
    Checking for program 'rsync'                   : /usr/bin/rsync 
    'configure' finished successfully (1.085s)
    pbarker@bluebottle:~/rc/ardupilot(master)$ ./waf tests
    Waf: Entering directory `/home/pbarker/rc/ardupilot/build/linux'
    Waf: Leaving directory `/home/pbarker/rc/ardupilot/build/linux'
    .
    .
    .
    tests/test_polygon                                    204433   7056   1352  212841
    tests/test_segment_intersection                       193839   5832   1288  200959
    tests/test_vector2                                    210539   6472   1320  218331

    Build commands will be stored in build/linux/compile_commands.json
    'tests' finished successfully (2.249s)
    pbarker@bluebottle:~/rc/ardupilot(master)$  

You can then simply run the test from the build directory:

::

   pbarker@bluebottle:~/rc/ardupilot(master)$ ./build/linux/tests/test_vector2
    [==========] Running 7 tests from 1 test suite.
    [----------] Global test environment set-up.
    [----------] 7 tests from Vector2Test
    [ RUN      ] Vector2Test.IsEqual
    [       OK ] Vector2Test.IsEqual (0 ms)
    [ RUN      ] Vector2Test.angle
    [       OK ] Vector2Test.angle (0 ms)
    [ RUN      ] Vector2Test.length
    [       OK ] Vector2Test.length (0 ms)
    [ RUN      ] Vector2Test.normalized
    [       OK ] Vector2Test.normalized (0 ms)
    [ RUN      ] Vector2Test.reflect
    [       OK ] Vector2Test.reflect (0 ms)
    [ RUN      ] Vector2Test.closest_point
    [       OK ] Vector2Test.closest_point (0 ms)
    [ RUN      ] Vector2Test.circle_segment_intersectionx
    [       OK ] Vector2Test.circle_segment_intersectionx (0 ms)
    [----------] 7 tests from Vector2Test (0 ms total)

    [----------] Global test environment tear-down
    [==========] 7 tests from 1 test suite ran. (0 ms total)
    [  PASSED  ] 7 tests.
    pbarker@bluebottle:~/rc/ardupilot(master)$ 


Using with GDB
..............

The tests can be run under GDB to trace problems:


::

    pbarker@bluebottle:~/rc/ardupilot(master)$ gdb --quiet --args ./build/linux/tests/test_vector2
    Reading symbols from ./build/linux/tests/test_vector2...done.
    (gdb) break test_vector2.cpp:20
    Breakpoint 1 at 0x81b9: file ../../libraries/AP_Math/tests/test_vector2.cpp, line 20.
    (gdb) r
    Starting program: /home/pbarker/rc/ardupilot/build/linux/tests/test_vector2 
    [Thread debugging using libthread_db enabled]
    Using host libthread_db library "/lib/x86_64-linux-gnu/libthread_db.so.1".
    [==========] Running 7 tests from 1 test suite.
    [----------] Global test environment set-up.
    [----------] 7 tests from Vector2Test
    [ RUN      ] Vector2Test.IsEqual
    [       OK ] Vector2Test.IsEqual (0 ms)
    [ RUN      ] Vector2Test.angle

    Breakpoint 1, Vector2Test_angle_Test::TestBody (this=0x5555557c2b90)
        at ../../libraries/AP_Math/tests/test_vector2.cpp:20
    20	    EXPECT_FLOAT_EQ(M_PI/2, Vector2f(0, 1).angle());
    (gdb) 


::

   self.send_debug_trap()

Using with Valgrind
...................

The tests can be run under Valgrind's memcheck too to to trace memory problems:

::

    pbarker@bluebottle:~/rc/ardupilot(master)$ valgrind --soname-synonyms=somalloc=nouserintercepts ./build/linux/tests/test_vector2
    ==7973== Memcheck, a memory error detector
    ==7973== Copyright (C) 2002-2017, and GNU GPL'd, by Julian Seward et al.
    ==7973== Using Valgrind-3.13.0 and LibVEX; rerun with -h for copyright info
    ==7973== Command: ./build/linux/tests/test_vector2
    ==7973== 
    [==========] Running 7 tests from 1 test suite.
    [----------] Global test environment set-up.
    [----------] 7 tests from Vector2Test
    [ RUN      ] Vector2Test.IsEqual
    [       OK ] Vector2Test.IsEqual (18 ms)
    [ RUN      ] Vector2Test.angle
    [       OK ] Vector2Test.angle (11 ms)
    [ RUN      ] Vector2Test.length
    [       OK ] Vector2Test.length (2 ms)
    [ RUN      ] Vector2Test.normalized
    [       OK ] Vector2Test.normalized (8 ms)
    [ RUN      ] Vector2Test.reflect
    [       OK ] Vector2Test.reflect (8 ms)
    [ RUN      ] Vector2Test.closest_point
    [       OK ] Vector2Test.closest_point (13 ms)
    [ RUN      ] Vector2Test.circle_segment_intersectionx
    [       OK ] Vector2Test.circle_segment_intersectionx (9 ms)
    [----------] 7 tests from Vector2Test (81 ms total)

    [----------] Global test environment tear-down
    [==========] 7 tests from 1 test suite ran. (137 ms total)
    [  PASSED  ] 7 tests.
    ==7973== 
    ==7973== HEAP SUMMARY:
    ==7973==     in use at exit: 0 bytes in 0 blocks
    ==7973==   total heap usage: 234 allocs, 234 frees, 116,474 bytes allocated
    ==7973== 
    ==7973== All heap blocks were freed -- no leaks are possible
    ==7973== 
    ==7973== For counts of detected and suppressed errors, rerun with: -v
    ==7973== ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)
    pbarker@bluebottle:~/rc/ardupilot(master)$ 



Unit Tests Structure
====================

A directory in the ArduPilot source tree can contain a `wscript` file designating it as containing unit tests.

Each .cpp file in that directory is then considered a valid test, and will be included in the tests compiled when `waf` is invoked for the `tests` target.

Tests are present in the following directories at time if writing:
::

    libraries/AP_Common/tests
    libraries/AP_Math/tests
    libraries/AP_ADSB/tests
    libraries/AP_GPS/tests
    libraries/AP_HAL/tests
    libraries/AP_HAL/utility/tests
    libraries/AP_HAL_Linux/tests

Adding a Unit Test
==================

Generally copying an existing file aside and using it as the basis of your new tests is the way forward.  See the GTest framework documentation for more information.

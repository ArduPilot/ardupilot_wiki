.. _the-ardupilot-autotest-framework:

==================
Autotest Framework
==================

.. image:: ../images/autotest.jpg

ArduPilot has an automatic testing framework based on SITL. The autotest
framework is what produces the web pages at
http://autotest.ardupilot.org

You can also manually run the autotester from the command line. That is
useful when you want to add features to the autotest or want to test
some new flight code on your own machine with auto scripting.

To use the autotester you need to first :ref:`get SITL running <setting-up-sitl-on-linux>`. After that you should run:

::

    cd ardupilot
    ./Tools/autotest/autotest.py --help

that will show you the command line options for the autotester. If you
get any python errors it probably means you are missing some required
packages. Go and check on the SITL setup page and see if you are missing
anything.

Test actions
------------

The autotester supports a long list of possible test scripts. If you
run:

::

    ./Tools/autotest/autotest.py --list

you will see what test scripts you can run. You can then add those
commands on the command line to run them. For example, to build the
fixed wing code and then run a test flight do this:

::

    ./Tools/autotest/autotest.py build.ArduPlane fly.ArduPlane

the results (and log files) will be put in the ../buildlogs directory.

You can also ask it to display a map while it is flying, which can make
watching autotest a bit less boring! Run it like this:

::

    ./Tools/autotest/autotest.py build.ArduPlane fly.ArduPlane --map

you will actually see the map appear twice, once for when it loads the
default parameters, and then for the real flight. Just close the first
one.

Changing the test scripts
-------------------------

Each of the tests is in Tools/autotest. For the fixed wing tests look at
Tools/autotest/arduplane.py.

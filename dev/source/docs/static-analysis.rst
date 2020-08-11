.. _static-analysis:

===============
Static Analysis
===============

Two tools are often used by ArduPilot to perform static analysis on the codebase, clang's "scan build" option and Coverity.

----------------
Clang Scan-Build
----------------

You can do your own static analysis using autotest:

::

   ./Tools/autotest/autotest.py clang-scan-build

.. note::

   You may need to install clang for this to work.

   On Ubuntu, "sudo apt-get install clang"

   On Windows see https://clang.llvm.org/get_started.html

The results can be viewed in a web browser.  The output of the scan step includes instructions, which look something like this:

::

   scan-view tmp/scan-build-2020-08-11-114140-28761-1

On Ubuntu 18.04 one might actually need:

::

   PYTHONPATH=/usr/lib/llvm-3.8/share/scan-view scan-view-3.8 tmp/scan-build-2020-08-11-114140-28761-1

Note that the current output from this tool has several false-positives and some positives-but-should-not-be-fixed.

Things to *not* worry about:
 - EKF assigning to variables but not reading from them
    - these require too much review to be worthwhile in general
 - memory leaks around "new AP_Frsky_Telem(true);" and similar calls.
    - These create singletons which give another handle on the allocated memory.

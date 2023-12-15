==========
Parameters
==========

Parameters refer to the settings and configuration of the autopilot. The
following commands can be used to view and change the parameters:

Any specific parameter can be tab-completed. Otherwise, a list of
possible parameters is shown.

A visual parameter editor is available from the Console drop-down menu, orcan be activated via
``module load paramedit``.

param show
==========

View all current parameters on the autopilot.

param set
=========

Change a specified parameter to a particular value.

.. code:: bash

    param set FLTMODE1 9

param save
==========

Save all current parameters on the autopilot to file.

.. code:: bash

    param save filename.parm

param load
==========

Load parameters from file to the autopilot. Only the parameters in the file
will be overwritten.

.. code:: bash

    param load filename.parm

param diff
==========

View the differences between the parameters in a file and what is
currently on the autopilot.

.. code:: bash

    param diff filename.parm

Wildcards
=========

Any of the above commands can be filtered by using wildcards, if only a
particular subset of parameters needs to be manipulated.

.. code:: bash

    param show FLTMODE*
    param load RC1*

param download
==============

Downloads the parameter definitions from the Internet and into a local
cache.

.. code:: bash

    param download

param help
==========

Shows further information (usage, notes) of the specified parameter.

.. code:: bash

    param help FLTMODE1

param fetch
===========

Download all parameters, by default via MAVFTP. Otherwise it will fallback
to using ``PARAM_REQUEST_LIST``. Useful for refreshing the full list of
parameters.

If a single parameter is specified, only that parameter will downloaded.

.. code:: bash

    param fetch RC1_MIN
    param fetch

param ftp
=========

Download all parameters via MAVFTP. Useful for refreshing the full list of
parameters.

.. code:: bash

    param ftp

param revert
============

Revert the specified parameter to it's default value.

.. code:: bash

    param revert FLTMODE1
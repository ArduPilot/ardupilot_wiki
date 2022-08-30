==============
Log Management
==============

.. code:: bash

    module load log
    
Allows the user to manage the dataflash logs stored on the autopilot. Use ``log list`` to view a
list of the logs on the autopilot and ``log erase`` to delete all logs on the
autopilot.

Use ``log download [lognumber] [filename]`` to start to download a
specific log to the specified file name. ``log status`` to see the
transfer status and ``log cancel`` to cancel the download.


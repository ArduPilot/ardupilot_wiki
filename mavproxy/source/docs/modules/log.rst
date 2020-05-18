==============
Log Management
==============

.. code:: bash

    module load log
    
Allows the user to manage the dataflash logs stored on an APM. Use ``log list`` to view a
list of the logs on the APM and ``log erase`` to delete all logs on the
APM.

Use ``log download [lognumber] [filename]`` to start to download a
specific log to the specified file name. ``log status`` to see the
transfer status and ``log cancel`` to cancel the download.


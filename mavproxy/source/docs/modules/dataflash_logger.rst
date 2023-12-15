==============
DataFlash Logs
==============

.. code:: bash

    module dataflash_logger
    
This module allows dataflash logs to be streamed in realtime from the autopilot. It requires the
``LOG_BACKEND_TYPE`` parameter to include the "MAVLink" backend.

The logs will be downloaded to the vehicle logging folder.

Control
=======

.. code:: bash

    dataflash_logger start
    
Start streaming dataflash logs. Requires the dataflash logging
to be started, typically when the vehicle is armed.

.. code:: bash

    dataflash_logger stop
    
Stop streaming dataflash logs

.. code:: bash

    dataflash_logger status
    
Print the progress of the dataflash log streaming    

Settings
========

The configurable settings for this module can be controlled by:

.. code:: bash

    dataflash_logger set <setting> <value>
    
The settings are:

===============================   =======================================   ===============================
Setting                           Description                               Default
===============================   =======================================   ===============================
verbose                           Verbose printouts                         False
df_target_system                  Target System                             0
df_target_component               Target Component                          (MAVProxy default)
===============================   =======================================   ===============================





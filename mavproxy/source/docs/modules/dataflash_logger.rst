==============
DataFlash Logs
==============

.. code:: bash

    module dataflash_logger
    
This module allows dataflash logs to be downloaded from the APM.

The logs will be downloaded to the vehicle logging folder.

Control
=======

.. code:: bash

    dataflash_logger start
    
Start downloading dataflash logs

.. code:: bash

    dataflash_logger stop
    
Stop downloading dataflash logs

.. code:: bash

    dataflash_logger status
    
Print the progress of the dataflash log download    

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





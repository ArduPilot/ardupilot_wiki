.. park:

============
Park Warning
============

Provides a GCS warning if a vehicle (usually loitering) moves outside a preset distance in any 3D direction from when set.

.. code:: bash

    module load park


Usage
=====

 park <status|on|off|radius>

 - on: enable , set current position
 - off: disable (default)
 - radius: 3D distance in meters from set position to send warning to GCS (default = 2m)
 - status: status of park module
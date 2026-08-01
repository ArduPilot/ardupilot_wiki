.. _mavlink-mavftp:

======
MAVFTP
======

MAVFTP supports common FTP operations including uploading, downloading, removing and creating files on the :ref:`autopilot file system <filesystems>`

The `official mavlink.io documentation is here <https://mavlink.io/en/services/ftp.html>`__ and contains the detailed sequence of messages that should be passed between the GCS/companion computer and autopilot

Common uses for using MAVFTP include:

- :ref:`Uploading firmware to the autopilot <common-install-sdcard>`
- :ref:`Uploading Lua scripts <copter:common-lua-scripts>`
- Fast download of parameters, :ref:`onboard logs <copter:common-logs>`, mission command files and :ref:`rally points <copter:common-rally-points>` and :ref:`terrain data <copter:terrain-following>`

Reference Implementation
========================

`pymavlink <https://github.com/ArduPilot/pymavlink>`__ includes a known-working MAVFTP client, `mavftp.py <https://github.com/ArduPilot/pymavlink/blob/master/mavftp.py>`__ (with its opcode definitions in `mavftp_op.py <https://github.com/ArduPilot/pymavlink/blob/master/mavftp_op.py>`__), usable both as a Python library and as a standalone command-line tool. It is a good reference for testing a new GCS/companion computer implementation against, or for scripting bulk MAVFTP transfers directly.

.. warning::

   Implementing only the request/reply sequence described in the MAVLink specification above will work, but will be slow. pymavlink's implementation gets much better throughput by pipelining requests rather than waiting for each reply before sending the next request: reads use ``OP_BurstReadFile`` to stream multiple chunks per request rather than one chunk per ``OP_ReadFile``/reply round-trip, and writes keep a backlog of several in-flight, unacknowledged chunks (a small sliding window, similar in spirit to TCP) instead of sending one ``OP_WriteFile`` and waiting for its ack before sending the next. A GCS/companion computer implementation aiming for good performance should do the same rather than implementing a strict one-request-then-wait-for-reply loop.

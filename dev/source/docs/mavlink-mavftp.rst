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

Directory Listings With Times
=============================

In addition to the plain ``ListDirectory`` operation (opcode 3), ArduPilot 4.8 and later support ``ListDirectoryWithTime`` (opcode 16). Each entry is returned as ``<type><name>\t<size>\t<mtime>``, where ``mtime`` is the modification time in seconds since the UNIX epoch:

- directories are returned with a size of 0 and their modification time
- an unknown time is sent as 0. This includes files written to a FAT filesystem while the autopilot had no clock source, which carry the 1980 FAT epoch
- an entry whose name contains a tab character is sent as a bare ``S`` (skip) entry

Log Directory (@MAV_LOG)
========================

ArduPilot 4.8 and later provide the flight-stack-independent ``@MAV_LOG`` directory defined by the MAVFTP specification. Some ground stations, including QGroundControl, look here first for onboard logs. ``@MAV_LOG`` is an alias for the directory ArduPilot writes its logs to (normally ``APM/LOGS``, or the custom log directory if one is configured), so ``@MAV_LOG/00000001.BIN`` and ``APM/LOGS/00000001.BIN`` are the same file. Files can be listed, read, written, renamed and deleted through either path. Renaming a file between ``@MAV_LOG`` and a different virtual filesystem, such as ``@SYS``, fails with ``EXDEV``.

``@MAV_LOG`` is only present on autopilots that log to a filesystem (normally an SD card). It is not available on boards whose local filesystem is LittleFS, and it can be removed from a :ref:`custom build <copter:common-custom-firmware>` with the ``FILESYSTEM_MAVLOG`` build option.

.. note::

   ArduPilot's MAVFTP does not confine requests to a root directory, so ``@MAV_LOG/..`` reaches the rest of the filesystem, as plain paths already do.

Reference Implementation
========================

`pymavlink <https://github.com/ArduPilot/pymavlink>`__ includes a known-working MAVFTP client, `mavftp.py <https://github.com/ArduPilot/pymavlink/blob/master/mavftp.py>`__ (with its opcode definitions in `mavftp_op.py <https://github.com/ArduPilot/pymavlink/blob/master/mavftp_op.py>`__), usable both as a Python library and as a standalone command-line tool. It is a good reference for testing a new GCS/companion computer implementation against, or for scripting bulk MAVFTP transfers directly.

.. warning::

   Implementing only the request/reply sequence described in the MAVLink specification above will work, but will be slow. pymavlink's implementation gets much better throughput by pipelining requests rather than waiting for each reply before sending the next request: reads use ``OP_BurstReadFile`` to stream multiple chunks per request rather than one chunk per ``OP_ReadFile``/reply round-trip, and writes keep a backlog of several in-flight, unacknowledged chunks (a small sliding window, similar in spirit to TCP) instead of sending one ``OP_WriteFile`` and waiting for its ack before sending the next. A GCS/companion computer implementation aiming for good performance should do the same rather than implementing a strict one-request-then-wait-for-reply loop.

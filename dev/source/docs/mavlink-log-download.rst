.. _mavlink-log-download:

============
Log Download
============

This page explains how MAVLink can be used by a ground station or companion computer to list and download the vehicle's :ref:`onboard dataflash logs <copter:common-logs>`.

.. note::

   The vehicle must be disarmed before log download will be accepted.  If armed, ArduPilot replies with a "Disarm for log download" status text and ignores the request.

.. note::

   :ref:`MAVFTP <mavlink-mavftp>` is a much faster way to download logs if it is supported by both ends of the link, since it is not limited to the small chunk sizes used by the LOG_* messages below.

Listing available logs
-----------------------

Send a `LOG_REQUEST_LIST <https://mavlink.io/en/messages/common.html#LOG_REQUEST_LIST>`__ with ``start``/``end`` set to the range of log ids to list (0/0xffff for all). ArduPilot replies with one `LOG_ENTRY <https://mavlink.io/en/messages/common.html#LOG_ENTRY>`__ per log in that range, each giving the log's ``id``, ``size`` (bytes, may be approximate) and ``time_utc`` (0 if not available). If there are no logs at all, a single ``LOG_ENTRY`` with ``id`` = 0 and ``num_logs`` = 0 is sent instead.

Downloading a log
------------------

Send a `LOG_REQUEST_DATA <https://mavlink.io/en/messages/common.html#LOG_REQUEST_DATA>`__ with the desired log's ``id`` (from ``LOG_ENTRY`` above), an ``ofs`` (byte offset into the log) and a ``count`` (number of bytes wanted). ArduPilot replies with one or more `LOG_DATA <https://mavlink.io/en/messages/common.html#LOG_DATA>`__ messages, each carrying up to 90 bytes of log data starting at the requested offset.

To download a whole log, repeat ``LOG_REQUEST_DATA`` with increasing ``ofs`` until the full ``size`` reported in the log's ``LOG_ENTRY`` has been received — this is the reliable way to know a download is complete. ArduPilot does not reliably send a trailing zero-``count`` ``LOG_DATA`` to mark end-of-log (a final chunk that exactly fills the requested byte range will not be followed by one), so a client should not depend on that alone. Any missing chunks (for example after a lost message) can be re-requested by sending another ``LOG_REQUEST_DATA`` for just that offset range.

Erasing all logs
-----------------

Send a `LOG_ERASE <https://mavlink.io/en/messages/common.html#LOG_ERASE>`__ message (only ``target_system``/``target_component`` fields) to erase all onboard logs. This cannot be undone and cannot be limited to a single log.

Ending a log transfer
----------------------

ArduPilot automatically closes its read of the current log once a ``LOG_REQUEST_DATA`` request has been fully satisfied, so `LOG_REQUEST_END <https://mavlink.io/en/messages/common.html#LOG_REQUEST_END>`__ does not need to be sent after a normal, complete download. Its main use is to cancel/abort an in-progress transfer early (for example if the GCS no longer wants the rest of a log); ArduPilot also cancels the transfer itself if a subsequent ``LOG_REQUEST_DATA`` asks for an invalid log id.

**Example**

MAVProxy's built-in ``log`` module (loaded by default) provides higher
level commands that handle the ``LOG_REQUEST_LIST``/``LOG_REQUEST_DATA``
exchange above automatically:

- ``log list`` — list available logs
- ``log download <lognumber> <filename>`` — download a single log to a file
- ``log download all`` — download all logs
- ``log erase`` — erase all logs

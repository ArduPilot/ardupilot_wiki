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

To download a whole log, send ``LOG_REQUEST_DATA`` with increasing ``ofs`` until the full ``size`` reported in the log's ``LOG_ENTRY`` has been received — this is the reliable way to know a download is complete. Only one ``LOG_REQUEST_DATA`` may be outstanding on a link at a time: ArduPilot silently ignores any further ``LOG_REQUEST_DATA`` it receives while a previous one is still being serviced, so a client must wait for each request to finish (or time out) before sending the next — pipelining several requests at once does not work.

.. warning::

   Do not rely on a trailing zero-``count`` ``LOG_DATA`` to detect end-of-log. ArduPilot only sends one when the requested ``ofs`` is already at or beyond the log's ``size``; a final chunk that exactly fills the requested byte range is not followed by one. Treating ``count == 0`` as the completion signal will hang at close to 100% on any log whose size is not an exact multiple of 90 bytes.

Any chunk missed (for example after a lost message) can be re-requested afterwards with another ``LOG_REQUEST_DATA`` for just that offset range.

Erasing all logs
-----------------

Send a `LOG_ERASE <https://mavlink.io/en/messages/common.html#LOG_ERASE>`__ message (only ``target_system``/``target_component`` fields) to erase all onboard logs. This cannot be undone and cannot be limited to a single log.

Ending a log transfer
----------------------

ArduPilot automatically closes its read of the current log once a ``LOG_REQUEST_DATA`` request has been fully satisfied, so `LOG_REQUEST_END <https://mavlink.io/en/messages/common.html#LOG_REQUEST_END>`__ does not need to be sent after a normal, complete download. Its main use is to cancel/abort an in-progress transfer early (for example if the GCS no longer wants the rest of a log).

.. note::

   Sending a further ``LOG_REQUEST_DATA`` with an invalid log id does **not** cancel a transfer that is already in progress — while a transfer is active, ArduPilot ignores any additional ``LOG_REQUEST_DATA`` it receives on that link (see below). The invalid-id request is only actioned, ending the (nonexistent) transfer, when no transfer is currently running.

**Example**

MAVProxy's built-in ``log`` module (loaded by default) provides higher
level commands that handle the ``LOG_REQUEST_LIST``/``LOG_REQUEST_DATA``
exchange above automatically:

- ``log list`` — list available logs
- ``log download <lognumber> <filename>`` — download a single log to a file
- ``log download all`` — download all logs
- ``log erase`` — erase all logs

Streaming the Current Log to a Companion Computer
--------------------------------------------------

The methods above download a *completed* log after the fact. A
companion computer can instead have ArduPilot stream the *current*
log to it live, as it is being written, by setting bit 1 ("MAVLink")
of :ref:`LOG_BACKEND_TYPE <copter:LOG_BACKEND_TYPE>`. The size
of the in-memory ring buffer used for this is set by
:ref:`LOG_MAV_BUFSIZE <copter:LOG_MAV_BUFSIZE>` (KB).

With that bit set, ArduPilot sends `REMOTE_LOG_DATA_BLOCK <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_DATA_BLOCK>`__ messages (200 bytes of log data per message, identified by a ``seqno``) once a receiver signals it is ready by sending a `REMOTE_LOG_BLOCK_STATUS <https://mavlink.io/en/messages/ardupilotmega.html#REMOTE_LOG_BLOCK_STATUS>`__ with ``seqno`` = ``MAV_REMOTE_LOG_DATA_BLOCK_START`` (2147483646). The receiver acks/nacks each block it receives (or notices missing) with another ``REMOTE_LOG_BLOCK_STATUS``, ``seqno`` set to the block number and ``status`` = ``MAV_REMOTE_LOG_DATA_BLOCK_ACK`` (1) or ``_NACK`` (0); ArduPilot re-sends nacked blocks. Sending ``REMOTE_LOG_BLOCK_STATUS`` with ``seqno`` = ``MAV_REMOTE_LOG_DATA_BLOCK_STOP`` (2147483645) stops the stream.

`dflogger <https://github.com/peterbarker/dronekit-la/blob/master/dataflash_logger.cpp>`__ is one existing implementation of this protocol suitable for running on a companion computer.

MAVFTP
------

:ref:`MAVFTP <mavlink-mavftp>` can also fetch a log file directly from the autopilot's filesystem, but this is not the recommended method -- use the ``LOG_*`` messages above (for a completed log) or MAVLink log streaming (for the current log) instead.

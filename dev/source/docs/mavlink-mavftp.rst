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

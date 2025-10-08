.. _common-network-capture:

======================
Network Packet Capture
======================

ArduPilot can capture on-target network traffic for troubleshooting Ethernet and PPP links. Captures are written in standard ``pcap`` format and can be opened in tools such as Wireshark or tcpdump.

Overview
--------

When enabled, packets handled by the networking stack are written to capture files on the microSD card:

- **Ethernet:** ``eth0.cap``
- **PPP:** ``pppN.cap`` (e.g. ``ppp0.cap`` for the first PPP interface)

Capture can be enabled or disabled at runtime by toggling bit 7 in the :ref:`NET_OPTIONS <NET_OPTIONS>` parameter — no reboot required.

Build Requirement
-----------------

This feature must be included in your firmware build even if it already has the networking features since, by default, it is only included for SITL:

1. Use the `ArduPilot Custom Build Server <https://custom.ardupilot.org/>`__.
2. Under **Networking**, enable **NetworkCapture**.
3. Download and flash the resulting firmware to your flight controller.

Enabling Capture
----------------

1. Confirm your firmware was built with **NetworkCapture** enabled.
2. Set bit **7** of the :ref:`NET_OPTIONS <NET_OPTIONS>` bitmask.
3. Capture starts immediately; **no reboot is required**.
4. To stop capture, **unset** bit 7.

Files & Formats
---------------

- **Location:** Root of the flight controller’s microSD card (e.g. ``eth0.cap``, ``ppp0.cap``).
- **Format:** ``pcap`` (compatible with Wireshark/tcpdump).

Notes & Caveats
---------------

- Capturing can increase CPU and I/O load; enable only when debugging.
- Ensure the microSD card has sufficient free space and is functioning reliably.
- PPP captures are per-interface (``ppp0.cap``, ``ppp1.cap``, …).

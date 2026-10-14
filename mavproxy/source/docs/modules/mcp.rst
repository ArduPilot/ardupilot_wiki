.. _mcp:

===
MCP
===

A custom JSON-RPC client can talk to an ArduPilot vehicle through MAVProxy
with this module. It is an in-process server on two custom transports: HTTP
POST request/response, and TCP newline-delimited JSON. There is no sidecar.
A stock MCP client that only speaks stdio or HTTP+SSE will not connect.
The unmodified MCP Python SDK 2.3.0 Streamable HTTP client does connect
to ``POST /mcp`` and can list and call tools. It accepts this server's
``application/json`` response.
This is separate from the :ref:`chat` module (OpenAI Assistants + UI window).

.. code-block:: bash

    module load mavproxy_mcp

.. warning::

   Still early. Flight tools arm, change mode, take off, and send guided
   velocity setpoints when you enable them. Listeners and flight tools are
   **off by default**.

   There is no authentication. The HTTP handler does not read ``Origin``, and
   every HTTP response sends ``Access-Control-Allow-Origin: *``. The TCP
   listener has no credential check either. Bind defaults to ``127.0.0.1``.
   Leave it there except on a trusted isolated network. Loopback with
   ``allow_flight`` is still open to other processes on this machine, and to
   a browser on this machine. Prefer SITL, or a propeller-free bench, until
   you know the surface.

.. note::

   This module is **not** shipped inside MAVProxy. Install from the standalone
   `mcp-tools <https://github.com/davidbitton/mcp-tools>`__ repo and put it on
   ``PYTHONPATH``. Do not PR the module into ArduPilot/MAVProxy upstream —
   this page is only so people can find it.

Prerequisites
=============

- MAVProxy + pymavlink (same environment as your SITL lab)
- A master link (SITL or real flight controller)
- ``mavproxy_mcp.py`` on ``PYTHONPATH``

Install (Path Only)
-------------------

.. code-block:: bash

    git clone https://github.com/davidbitton/mcp-tools.git
    export PYTHONPATH="/path/to/mcp-tools/mavproxy-mcp${PYTHONPATH:+:$PYTHONPATH}"

Or add a venv ``.pth`` file that points at the ``mavproxy-mcp/`` directory.

Environment Variables (Read at Load)
------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 30 15 55

   * - Variable
     - Default
     - Meaning
   * - ``MAVPROXY_MCP_ENABLE``
     - ``0``
     - Auto-start listeners when the module loads
   * - ``MAVPROXY_MCP_HTTP``
     - ``8765``
     - HTTP port (``0`` = disable)
   * - ``MAVPROXY_MCP_TCP``
     - ``8766``
     - TCP port (``0`` = disable)
   * - ``MAVPROXY_MCP_BIND``
     - ``127.0.0.1``
     - Bind address
   * - ``MAVPROXY_MCP_ALLOW_FLIGHT``
     - ``0``
     - Expose flight tools

Usage
=====

Start MAVProxy, then:

.. code-block:: bash

    module load mavproxy_mcp
    mcp status          # listening: false
    mcp start           # 127.0.0.1:8765 HTTP, :8766 TCP
    mcp set allow_flight 1   # only if you want command tools

Loading the module does **not** open ports. ``mcp start`` does (or set
``MAVPROXY_MCP_ENABLE=1`` before load).

CLI
---

.. code-block:: bash

    mcp status | start | stop | restart | set …

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Command
     - What
   * - ``mcp status``
     - Endpoints + vehicle snapshot
   * - ``mcp start``
     - Bind listeners
   * - ``mcp stop``
     - Stop accepting new connections. Does not drop clients already connected
   * - ``mcp restart``
     - Stop then start
   * - ``mcp set <name> <value>``
     - Settings below

``mcp stop`` stops accepting new connections. It does not disconnect a client that
is already connected, and it does not close the listening sockets. A TCP
client can still call tools after the stop. An HTTP connection accepted
before the stop can still complete that request. ``mcp status`` reports
the listeners as down while that client is served. ``mcp restart`` can
then print ``TCP bind failed ... Address already in use`` until the client
disconnects. ``mcp set allow_flight 0`` blocks the next flight call
immediately, including on a connection that ``mcp stop`` left open.

Settings
--------

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Setting
     - Default
     - Meaning
   * - ``enabled``
     - ``0``
     - Load-time flag. ``mcp set enabled`` does not start or stop listeners
   * - ``http_port``
     - ``8765``
     - HTTP port (``0`` disables)
   * - ``tcp_port``
     - ``8766``
     - TCP port (``0`` disables)
   * - ``bind``
     - ``127.0.0.1``
     - Bind address
   * - ``allow_flight``
     - ``0``
     - Arm / mode / takeoff / velocity tools
   * - ``verbose``
     - ``0``
     - Extra logging

.. code-block:: bash

    mcp set allow_flight 1
    mcp set http_port 8765
    mcp restart

``mcp set enabled 1`` changes the flag only. Listeners auto-start when
``MAVPROXY_MCP_ENABLE=1`` before the module loads. After load, ``mcp start``
and ``mcp stop`` are the run-time controls, and those commands set
``enabled`` to match.

Transports
----------

HTTP: GET ``/``, ``/health``, or ``/mcp`` returns a JSON health document.
POST ``/``, ``/mcp``, or ``/rpc`` accepts JSON-RPC 2.0 (one object or a batch
array) and writes the JSON-RPC result in the response body.

TCP: newline-delimited JSON-RPC on ``bind:tcp_port``.

These transports are custom. ``initialize`` returns protocol version
``2024-11-05`` and server name ``mavproxy-mcp`` (``0.1.0``). The 2024-11-05
revision's transports are stdio and HTTP+SSE, and that revision requires
``initialize`` before any other request. This server keeps no session, so
``tools/list`` and ``tools/call`` succeed with no ``initialize`` /
``notifications/initialized`` first.

The curls below check this HTTP listener. The second call is
``tools/list`` with no handshake:

.. code-block:: bash

    curl -s http://127.0.0.1:8765/health | jq .
    curl -s -X POST http://127.0.0.1:8765/mcp \
      -H 'Content-Type: application/json' \
      -d '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | jq .

MCP Methods
-----------

``initialize`` returns ``protocolVersion`` ``2024-11-05``. That string is what
the handler sends. It does not mean these listeners implement that revision's
transports or its initialize-first lifecycle.

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Method
     - What
   * - ``initialize``
     - Returns ``protocolVersion``; other methods do not require it first
   * - ``notifications/initialized`` / ``initialized``
     - Accepted, no response, no session state
   * - ``ping``
     - Liveness
   * - ``tools/list``
     - List tools (flight set only if ``allow_flight``)
   * - ``tools/call``
     - Invoke a tool by name

``tools/call`` returns ``content`` (text JSON) and ``isError``. It also adds
``structuredContent`` (the parsed object). ``structuredContent`` is absent
from the 2024-11-05 tool-result schema. It is an optional field in the
2025-06-18 schema.

Tools
-----

Always available (read): ``vehicle_status``, ``position``, ``mcp_info``

If ``allow_flight`` is enabled, the flight tools only transmit. ``ok`` means
the message was handed to the master link. It does not mean the flight
controller accepted it or that the vehicle reached the requested state. Poll
``vehicle_status`` and ``position`` before treating mode, armed state, or
altitude as done. No tool returns ``COMMAND_ACK``. That acknowledgement is
outside this API. MAVProxy prints it on the console.

- ``set_mode`` — sends ``MAV_CMD_DO_SET_MODE`` (any mode the master link
  knows, such as ``GUIDED``, ``RTL``, or ``LOITER``).
- ``arm`` — sends ``MAV_CMD_COMPONENT_ARM_DISARM``. With ``force`` true,
  param2 is ``21196`` on the vehicle that is connected, including a real
  flight controller. That skips non-mandatory arm checks. Mandatory checks
  still run, for example OpenDroneID, RC calibration, and serial protocol.
  Copter also runs its position and altitude checks. Plane also runs its
  RC-received check.
  ``{"arm": false, "force": true}`` force-disarms, including a flying Copter
  or Plane. Keep ``force`` for SITL or a propeller-free bench.
- ``takeoff`` — on the ground, sends ``MAV_CMD_NAV_TAKEOFF``. Above 1 m
  relative altitude and below 90% of ``alt_m``, it sends one local-NED
  velocity setpoint at ``vd=-2`` m/s and returns ``method: guided_climb``.
  That single setpoint does not hold the climb until ``alt_m``. At or above
  90% of the target it sends nothing (``method: skip``). The call raises
  ``not armed`` when MAVProxy's current status is disarmed. ``arm`` does not
  wait for the heartbeat that updates that flag, so the next call can still
  see the vehicle as disarmed.
- ``velocity_ned`` — sends one GUIDED velocity setpoint in local NED (m/s).

No flight gate → permission error. That is intentional.

Sample Session (SITL)
---------------------

Rough SITL loop with ``allow_flight`` on. Each flight call only transmits.
Poll ``vehicle_status`` after ``arm`` and wait until it shows armed before
``takeoff``. ``force`` skips non-mandatory checks on a real vehicle too; use
this loop on SITL:

.. code-block:: bash

    tools/call  vehicle_status
    tools/call  set_mode      {"mode": "GUIDED"}
    tools/call  vehicle_status
    tools/call  arm           {"arm": true, "force": true}
    tools/call  vehicle_status    # wait until this shows armed
    tools/call  takeoff       {"alt_m": 10}
    tools/call  position
    tools/call  velocity_ned  {"vn": 1, "ve": 0, "vd": 0}
    tools/call  position
    tools/call  set_mode      {"mode": "RTL"}

curl form for one call:

.. code-block:: bash

    curl -s -X POST http://127.0.0.1:8765/mcp \
      -H 'Content-Type: application/json' \
      -d '{
        "jsonrpc": "2.0",
        "id": 2,
        "method": "tools/call",
        "params": {"name": "vehicle_status", "arguments": {}}
      }' | jq .

See Also
========

- `Source (mavproxy-mcp) <https://github.com/davidbitton/mcp-tools/tree/main/mavproxy-mcp>`__
- `mcp-tools repo <https://github.com/davidbitton/mcp-tools>`__
- License: GPL-3.0-only
- Related (different design): :ref:`chat`

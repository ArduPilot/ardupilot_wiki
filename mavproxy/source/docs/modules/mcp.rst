.. _mcp:

===
MCP
===

MCP clients (agents, lab tools, anything that speaks the protocol) can talk to
an ArduPilot vehicle through MAVProxy with this module. It is an in-process
JSON-RPC MCP server (HTTP + TCP) — no sidecar. Not the same as the
:ref:`chat` module (OpenAI Assistants + UI window).

.. code-block:: bash

    module load mavproxy_mcp

.. warning::

   Still early. Flight tools will arm, change mode, take off, and send guided
   velocity setpoints if you enable them. Listeners and flight tools are
   **off by default**. Bind defaults to ``127.0.0.1``. Prefer SITL until you
   know the surface.

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
     - Close listeners
   * - ``mcp restart``
     - Stop then start
   * - ``mcp set <name> <value>``
     - Settings below

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
     - Auto-start listeners on load
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

Transports
----------

HTTP: GET ``/``, ``/health``, or ``/mcp`` for health. POST ``/``, ``/mcp``, or
``/rpc`` for JSON-RPC 2.0 (single object or batch array).

TCP: newline-delimited JSON-RPC on ``bind:tcp_port``.

Smoke checks:

.. code-block:: bash

    curl -s http://127.0.0.1:8765/health | jq .
    curl -s -X POST http://127.0.0.1:8765/mcp \
      -H 'Content-Type: application/json' \
      -d '{"jsonrpc":"2.0","id":1,"method":"tools/list"}' | jq .

MCP Methods
-----------

Protocol version advertised on ``initialize``: ``2024-11-05``.
Server name: ``mavproxy-mcp`` (``0.1.0``).

.. list-table::
   :header-rows: 1
   :widths: 45 55

   * - Method
     - What
   * - ``initialize``
     - Handshake
   * - ``notifications/initialized`` / ``initialized``
     - Notification (no response)
   * - ``ping``
     - Liveness
   * - ``tools/list``
     - List tools (flight set only if ``allow_flight``)
   * - ``tools/call``
     - Invoke a tool by name

``tools/call`` returns MCP ``content`` (text JSON) plus ``structuredContent``
(parsed object).

Tools
-----

Always available (read): ``vehicle_status``, ``position``, ``mcp_info``

If ``allow_flight`` is enabled:

- ``set_mode`` — e.g. ``GUIDED``, ``RTL``, ``LOITER``
- ``arm`` — ``force`` uses SITL magic ``21196``
- ``takeoff`` — skips or GUIDED-climbs if already airborne (``NAV_TAKEOFF`` fails in the air)
- ``velocity_ned`` — GUIDED velocity in local NED (m/s)

No flight gate → permission error. That is intentional.

Sample Session (SITL)
---------------------

Rough agent loop with ``allow_flight`` on:

.. code-block:: bash

    tools/call  vehicle_status
    tools/call  set_mode      {"mode": "GUIDED"}
    tools/call  arm           {"arm": true, "force": true}
    tools/call  takeoff       {"alt_m": 10}
    tools/call  position
    tools/call  velocity_ned  {"vn": 1, "ve": 0, "vd": 0}
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

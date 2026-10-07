.. _sitl-wasm:

=====================================
SITL as WebAssembly (Browser/Node.js)
=====================================

.. note:: This feature is available in ArduPilot 4.8 and later.

SITL can be built as a `WebAssembly <https://webassembly.org/>`__ (WASM) module using the `Emscripten <https://emscripten.org/>`__ toolchain. This lets a vehicle run inside a web browser or Node.js with no native install, for example to embed a simulated vehicle in a web page or web-based GCS.

The ``wasm`` board is a SITL subtype. It uses SITL's built-in simulation models (e.g. ``--model plane``), and serial ports can be connected to JavaScript through a ring-buffer bridge instead of TCP/UDP sockets. Networking and CAN are not available in this build, so external simulators that connect to SITL over sockets (e.g. :ref:`JSON <sitl-with-JSON>`, Gazebo, RealFlight) cannot be used.

Unlike native SITL, the WASM build does not trap floating-point exceptions (overflow, divide-by-zero, invalid operation), so numerical faults that would stop a native SITL run go undetected. Use native SITL when testing for them.

An example of hosting the build in a web page is the `ArduPilot WASM SITL demo <https://github.com/gribbet/ardupilot-wasm-sitl-demo>`__.

Installing Emscripten
=====================

On Ubuntu, install the Emscripten SDK (``emsdk``) with the supplied script. Python 3.10 or newer is required.

::

    ./Tools/environment_install/install-wasm-prereqs-ubuntu.sh
    source "$HOME/emsdk/emsdk_env.sh"

By default the SDK is installed in ``~/emsdk`` and a line sourcing ``emsdk_env.sh`` is added to ``~/.profile``. These environment variables change the defaults:

-  ``EMSCRIPTEN_VERSION``: the Emscripten version to install (default ``6.0.8``)
-  ``EMSDK_ROOT``: the install location (default ``$HOME/emsdk``). If changed, source ``$EMSDK_ROOT/emsdk_env.sh`` instead of the path shown above.
-  ``SHELL_LOGIN``: the login file, relative to ``$HOME``, that the ``emsdk_env.sh`` line is added to (default ``.profile``)

Building
========

::

    ./waf configure --board wasm
    ./waf plane

The build produces a JavaScript ES module and its paired WebAssembly binary in ``build/wasm/bin/`` (e.g. ``arduplane.js`` and ``arduplane.wasm``). Other vehicles can be built in the same way, but only Plane is tested in CI.

Running
=======

The host must start ArduPilot with these serial options:

::

    --serial0 wasm --serial1 none --serial2 none

The ``wasm`` board inherits the normal SITL TCP defaults for its serial ports. ``wasm`` connects ``SERIAL0`` to the JavaScript bridge, and ``none`` stops the unused socket-backed ports from being opened. Other options, such as ``--model``, are the same as for a native SITL binary.

The build uses shared WASM memory and threads, so a browser page hosting it must be cross-origin isolated:

-  The page must be served from a secure context: HTTPS, or ``http://localhost`` during development.
-  The page must be served with these headers:

   ::

       Cross-Origin-Opener-Policy: same-origin
       Cross-Origin-Embedder-Policy: require-corp

-  With ``require-corp``, every cross-origin resource the page loads (scripts, the ``.wasm`` file, images, etc.) must be served with CORS or ``Cross-Origin-Resource-Policy`` headers, or the browser will block it.

The page can check ``crossOriginIsolated`` in JavaScript to confirm that these requirements are met.

Emscripten cannot restart a process, so a reboot request (e.g. ``PREFLIGHT_REBOOT_SHUTDOWN``) aborts the module. The host must create a new instance to "reboot" the vehicle.

.. note:: The build uses Emscripten's default in-memory filesystem, so parameters, logs and any other files are lost when the module is recreated or the page is reloaded. To keep them, the host must copy them out through the exported ``FS`` object, or mount persistent storage (e.g. IDBFS in a browser or NODEFS in Node.js). See the `Emscripten File System API <https://emscripten.org/docs/api_reference/Filesystem-API.html>`__. Files such as a defaults parameter file can be written into the filesystem from a ``preRun`` callback before ArduPilot starts.

JavaScript interface
====================

The module exports these C functions, which can be called using Emscripten's ``cwrap``:

-  ``ardupilot_serial_write(serial_num, buf, len)``: send ``len`` bytes from ``buf`` to the vehicle on that serial port. Returns the number of bytes accepted.
-  ``ardupilot_serial_read(serial_num, buf, max_len)``: read up to ``max_len`` bytes that the vehicle has sent into ``buf``. Returns the number of bytes read.
-  ``ardupilot_serial_read_available(serial_num)``: returns the number of bytes waiting to be read.
-  ``ardupilot_malloc(size)`` and ``ardupilot_free(ptr)``: allocate and free buffers in WASM memory for the calls above.

``serial_num`` is the SERIALn port number, and the port must have been started with ``--serialN wasm``. The bridge uses single-producer/single-consumer ring buffers, so the host must not make overlapping calls in the same direction (e.g. two writes at once) on a port.

A minimal example that starts Plane and exchanges MAVLink data on SERIAL0. It works in Node.js or in a cross-origin isolated browser page:

.. code-block:: javascript

    import createModule from './build/wasm/bin/arduplane.js';

    // Supply the shared memory so that it can be viewed directly from JavaScript
    const wasmMemory = new WebAssembly.Memory({initial: 2 ** 8, maximum: 2 ** 15, shared: true});

    const module = await createModule({
        wasmMemory,
        arguments: ['--model', 'plane',
                    '--serial0', 'wasm', '--serial1', 'none', '--serial2', 'none'],
        print: console.log,
        printErr: console.error,
    });

    const malloc = module.cwrap('ardupilot_malloc', 'number', ['number']);
    const read = module.cwrap('ardupilot_serial_read', 'number', ['number', 'number', 'number']);
    const write = module.cwrap('ardupilot_serial_write', 'number', ['number', 'number', 'number']);
    const size = 4096;
    const rxBuf = malloc(size);
    const txBuf = malloc(size);

    setInterval(() => {
        const len = read(0, rxBuf, size);
        if (len > 0) {
            // create a new view on each access, as memory growth replaces wasmMemory.buffer
            const bytes = new Uint8Array(wasmMemory.buffer).slice(rxBuf, rxBuf + len);
            // pass bytes to a MAVLink parser here
        }
    }, 10);

    // send bytes (up to size) from a GCS to the vehicle
    const send = (bytes) => {
        new Uint8Array(wasmMemory.buffer).set(bytes, txBuf);
        return write(0, txBuf, bytes.length);
    };

.. note:: Emscripten's own ``HEAPU8`` view is not safe to use from the host thread with this build. The vehicle runs in a worker thread, and when it grows memory, ``HEAPU8`` on the host thread can still point at the old buffer, so reads silently return nothing.

A complete example is in ``Tools/autotest/wasm_plane_smoke_test.mjs`` in the ArduPilot source.

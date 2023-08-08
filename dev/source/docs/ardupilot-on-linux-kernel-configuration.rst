.. _ardupilot-on-linux-kernel-configuration:

==========================
Linux Kernel Configuration
==========================

Real-Time Patches
=================
For some builds, ArduPilot needs to run sensors, control loop, and output
interfaces at high rates, with bounded latency to maintain stable
operation. For this reason, it is suggested to run a real-time kernel.

Builds like rover, sub, and tracker may be fine without it.

Userspace access to hardware peripherals on 64-bit boards
=========================================================
The default kernel configuration for many 64-bit boards blocks
even the root user from mmap()ing the peripheral device address space,
allowing only the kernel to access it.

For many boards, ArduPilot will need such access, so CONFIG_STRICT_DEVMEM
or CONFIG_IO_STRICT_DEVMEM will need to be disabled.

32-bit kernels are unaffected.

Preventing CPU idle states
==========================
The kernel command line option ``cpuidle.off=1`` prevents CPU cores from
entering idle states. This can be useful on CPUs where exiting idle
states is slow.

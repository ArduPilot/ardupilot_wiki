.. _failsafe:

==============
Failsafe Setup
==============

.. note:: Since the Blimp code is very new, limited failsafe behavior is currently provided and is expected to improve and evolve over time.

Blimp provides several failsafes which can be enabled or disabled:

- RC loss via :ref:`FS_THR_ENABLE<FS_THR_ENABLE>`, always disarms on any RC failsafe. "0" ignores RC failsafes
- Ground Control Station via :ref:`FS_GCS_ENABLE<FS_GCS_ENABLE>`, any non-zero value enables disarming after :ref:`FS_GCS_TIMEOUT<FS_GCS_TIMEOUT>` seconds of GCS connection failure. "0" ignores GCS connection failures. 
- EKF Failsafe, when EKF is determined to be unhealthy, :ref:`FS_EKF_ACTION<FS_EKF_ACTION>` occurs
- Vibration Failsafe via :ref:`FS_VIBE_ENABLE<FS_VIBE_ENABLE>`
- Battery Failsafe via :ref:`BATT_LOW_VOLT<BATT_LOW_VOLT>` and :ref:`BATT_CRT_VOLT<BATT_CRT_VOLT>`,always disarms currently, independent of failsafe action selected by the :ref:`BATT_FS_CRT_ACT<BATT_FS_CRT_ACT>` and :ref:`BATT_FS_LOW_ACT<BATT_FS_LOW_ACT>` parameters.

.. note:: setting :ref:`FS_THR_ENABLE<FS_THR_ENABLE>` to 1 also allows throttle values below :ref:`FS_THR_VALUE<FS_THR_VALUE>` to force a failsafe in addition to RC loss or corruption. Be sure its set below any active throttle channel value to avoid triggering a failsafe unintentionally while demanding maximum descent.


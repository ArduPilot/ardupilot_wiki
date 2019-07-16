.. _ac_throttlemid:

======================
Setting Hover Throttle
======================

Copter includes automatic learning of Hover Throttle (previously known as "mid throttle")
The :ref:`MOT_THST_HOVER <MOT_THST_HOVER>` value will slowly move towards the average motor output whenever the vehicle is holding a steady hover in non-manual flight modes (i.e. all modes except Stabilize and Acro).

If you wish to manually set the :ref:`MOT_THST_HOVER <MOT_THST_HOVER>` value, it is best to download a dataflash log and set the value to what is seen in the CTUN.ThO field.  The value should be between 0.2 and 0.8.

If for some reason you wish to disable learning, you can set the :ref:`MOT_HOVER_LEARN <MOT_HOVER_LEARN>` parameter to 0.

.. image:: ../images/throttle_mid_learning.png
    :target: ../_images/throttle_mid_learning.png

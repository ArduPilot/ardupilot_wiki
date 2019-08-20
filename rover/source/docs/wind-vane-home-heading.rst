.. _wind-vane-home-heading:

=======================
Home Heading Wind Vane
=======================

:ref:`WNDVN_TYPE <WNDVN_TYPE>` = 1, the wind vane library assumes the true wind is coming from the direction the vehicle was pointing when armed.
This method is slightly improved upon by using :ref:`WNDVN_TYPE <WNDVN_TYPE>` = 2 allowing this initial wind direction to be offset +- 45 degrees
by a RC input channel defined by :ref:`WNDVN_RC_IN_NO <WNDVN_RC_IN_NO>`. These methods do not directly sense the wind but just assume
it is coming from a constant direction so they shouldn't be used in shifty wind conditions or for longer missions. They are however a good method
to test sailing support before a proper wind vane can be fitted.

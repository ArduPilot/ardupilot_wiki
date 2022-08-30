.. _common-wiki-editing-archiving:

=====================================
Wiki Editing - Archiving and Deleting
=====================================

Topics that are no longer relevant for current products, but which may be useful for some existing users, should be archived. 

This is done by:

#. Add "Archived:" prefix to the page title:

   .. code-block:: bash

       ========================
       Archived: Original title
       ========================

#. Add a warning directive with a note below the title, explaining that the article is archived.
   If possible, provide additional information about why it has been archived, and links
   to alternative/more up-to-date information:

   .. code-block:: bash

       .. warning

           This topic is archived.

#. Move the topic under "Archived Topics" in the menu (you will need to edit the toctree
   directive in :ref:`common-archived-topics`).


Deleting wiki pages
===================

Wiki pages can be deleted by removing them from github and any menu in which they appear.

.. warning::

    Before deleting a wiki page it is important to ensure that it is not the 
    parent of other menu items (e.g. it does not contain a "toctree")

[copywiki destination="copter,plane,rover,planner,planner2,antennatracker,dev,ardupilot,mavproxy"]

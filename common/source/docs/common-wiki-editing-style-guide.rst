.. _common-editing-style-guide:

========================
Wiki Editing Style Guide
========================

This page explains some specific parts of syntax used by the wiki along with general style guidelines to promote consistency of appearance and maintainability of wiki content. The general rule is to keep things simple, using as little styling as possible

For more information check out the 
`Sphinx reStructured Text Primer <http://www.sphinx-doc.org/en/stable/rest.html>`__.

Creating a new page
===================

.. tip::

   First search the wiki to determine if there is already a wiki page
   about your topic — it is better to improve an existing topic than create
   a near-duplicate! 
   Also discuss it with the 
   `Wiki Editors on Discord <https://ardupilot.org/discord>`__ or enter an issue in the `Wiki issue list <https://github.com/ArduPilot/ardupilot_wiki/issues>`__.

Pages should be created in the **/source/docs/** folder for your target wiki, given a "descriptive" name,
and the file suffix ".rst". Typically we use all lower case for filename, and separate words using "-" or "_". 
For example: **advanced-multicopter-design.rst**.

Pages that are common to all wikis must be named with the prefix **common-** and must be created 
in **/common/source/docs/**. See :ref:`common_wiki_editing_guide_common_pages` for more information.

The title of the new page should be concise — informative enough that
the reader can determine whether the topic is relevant and to differentiate it from other similar topics. 
The first letter of every word in the heading should be capitalized, with the exception of short
connecting words like "a", "with", "and", "the".

The title should also be preceded by an anchor link named for the page. So the first information on the page would be:

.. code-block:: rst

    .. _your_file_name:

    ===============
    Your Page Title
    ===============

.. _common_wiki_editing_guide_common_pages:

Working with common pages
=========================

The wiki has a lot of information that is applicable to users of all the
different vehicle types. In order to reduce (manual) duplication we
define these topics in one place (**/common/source/docs**) and automatically copy them 
to other wikis where they are needed.

Creating and editing common pages is similar to editing other pages except:

- The filename of common pages must start with the text *common-*. For
  example, this page is **common-wiki_editing_guide.rst**.
- All common pages must be stored in **/common/source/docs**
- The copywiki shortcode can be put at the end of the source to specify the set 
  of destination wikis (use "copywiki" rather than "xcopywiki" below):

  .. code-block:: bash

      [xcopywiki destination="copter,plane"]

- If no copywiki shortcode is specified, common pages are automatically copied to the copter, 
  plane and rover wikis

- Vehicle-specific content can be added to the common topic using the
  ``site`` shortcode. Text that is not applicable to a target wiki is stripped out 
  before the common page is copied to each wiki. The example below shows text that 
  will only appear on rover and plane wikis (use site rather than xsite shown below!)

  .. code-block:: bash

      [xsite wiki="rover, plane"]Rover and plane specific text[/xsite]

-  Always :ref:`link to other common topics <common-editor-information-resource_how_to_link_to_other_topics>` using relative linking. This ensures that you will link to the correct common topic when the wiki article is copied.

Titles
------

Choose a concise and specific title. It should be informative enough that a reader can determine
if the content is likely to be relevant and yet differentiate it from other (similar) topics.

Use first-letter capitalization for all words in the title (except connecting words: "and","the", "with" etc.)

The title syntax is as shown below. Note that we use an "anchor reference" immediately before the title (and named 
using the page filename). This allows us to link to the file from other wikis and from documents even if 
they move within the file structure.

.. code-block:: rst

    .. _your_file_name2:

    ==========
    Page Title
    ==========


Abstract
--------

Start the topic (after the title) with an abstract rather than a heading or an image.

Ideally this should be a single sentence or short paragraph describing the content and scope of the topic.


Headings
--------

Headings are created by (fully) underlining the heading text with a single character. 
We use the following levels:

.. code-block:: rst

    Heading 1
    =========
    
    Heading 2
    ---------
    
    Heading 3
    +++++++++
    
    Heading 4
    ^^^^^^^^^
    
    Heading 5
    ~~~~~~~~~



Emphasis
--------

Emphasis should be used *sparingly*. A page with too much bold
or italic is hard to read, and the effect of emphasis as a tool
for identifying important information is reduced.

Use emphasis to mark up *types* of information:

- ``code`` for code and variables
- **bold** for "button to press" and filenames
- *italic* for names of dialogs and tools.

The markup for each case is listed below.

.. code-block:: rst

    ``Inline code``
    **Bold**
    *Italic*

Lists
-----

Numbered lists can be generated by starting a line with ``#.`` followed by a space. 
Unordered lists can be generated by starting a line with "*" or "-". Nested lists
are created using further indentation:

.. code-block:: rst

    #Ordered listed
    
    #. Item one
    #. Item 2
       Multiline
    #. Item 3
       
       - Nested item
       #. Nested item ordered

    #Unordered list
    
    - Item 1
    - Item 2
    
      - Nested item


Information notes and warnings
------------------------------

You can add notes, tips and warnings in the text using the "tip", "note"
and "warning" shortcodes, respectively. These render the text in an
information box:

.. code-block:: rst

    .. note::

       This is a note

.. note::

   This is a note



.. code-block:: rst

    .. tip::

       This is a tip
   
   
.. tip::

   This is a tip
   
   
.. code-block:: rst

    .. warning::

       This is a warning

.. warning::

   This is a warning

   
Code
====

Use the "code-block" directive to declare code blocks. You can specify the type of code too and it will be 
syntax marked:

.. code-block:: rst

    .. code-block:: python
    
        This is format for a code block (in python)
    
        Some code

Alternatively you can just have a double colon "::" at the end of a line, a blank line,
and then indent the code block text:

.. code-block:: rst

    This is format for a code block. ::
    
        Some code



.. _common-editor-information-resource_how_to_link_to_other_topics:

Internal links
--------------

The best way to link to a topic within the docset is to use a reference link to a named anchor. 
This link will take you to the topic even if the document moves, and you can link to it across wikis.

An anchor should ideally be placed before a heading (or title) and has the format shown below (the leading
underscore and trailing colon are important):

.. code-block:: rst

    .. _a_named_link:
    
.. tip::

    * We recommend placing an anchor at the top of every page, named using the article filename.
    * Anchors need to be unique, so use the page anchor as a prefix for heading anchors
    * We've created a bunch of useful anchors for you; for example, to link to a parameter, you
      just specify that parameter as the target.
    

You can link to the anchor from the same wiki using either of the two approaches below:

.. code-block:: rst

    :ref:`a_named_link`  #Links to "a_named_link". Displays the title that follows the anchor.
    :ref:`Link Text <a_named_link>`  #Links to "a_named_link". Displays the specified text.

You can link to the anchor from another wiki by specifying the wiki as a prefix. So for example
to link to this anchor defined other wikis you would do:

.. code-block:: rst

    :ref:`copter:a_named_link`  #Links to "a_named_link" in the copter wiki
    :ref:`Link Text <planner:a_named_link>`  #Links to "a_named_link" in the planner wiki

.. tip::

    For links within a wiki and in most common topics you can use the "bare" format. Sometimes
    you will need to explicitly specify a target wiki.


External links
--------------

To link to off-wiki topics, use the following format:

.. code-block:: rst

    `Link text <http://the-target-link-url>`__

This same format can be used for internal links, but without the benefit of being able to track when
internal links are broken by title changes etc.
 

How to put the page into the sidebar menu
-----------------------------------------

Items are added to the sidebar by specifying them in the parent article's "toctree"
directive. The filename may omit the file extension, but must include the path relative to the current directory 
(typically there is no path in our wikis).

.. code-block:: rst

    .. toctree::
        :maxdepth: 1

        Pixhawk <common-pixhawk-overview>
        Display text <filename>

Sometimes the parent article is "common" but the wiki article is specific to a particular wiki. In this case you can
wrap the toctree changes using the **site** shortcode (as below, but with "site" instead of "xsite"). You might
also ignore this case, but it will give a "missing article" warning.

.. code-block:: rst

    .. toctree::
        :maxdepth: 1

        Pixhawk <common-pixhawk-overview>
        
        [xsite wiki="rover, plane"]
        Display text <filename>
        [/xsite]


How to put links in the top menu
--------------------------------

Top menu links are hard coded in the `site theme <https://github.com/ArduPilot/sphinx_rtd_theme>`__.


Using images in your wiki pages
-------------------------------

Our general advice for images is:

- Keep images as small as possible.

  .. tip::

      Images are stored on Github, so we need to keep the overall size low. Crop images to the relevant 
      information and reduce image quality where possible.

- Images in common pages or useful across wikis should be in the root **/images** directory.
- Images specific to the wiki can be stored in its **/images** sub directory.
      
- Use captions ("figure directive") where possible
- Link to the image if it is larger than can be displayed on the page.
- Name the file using all lower case, and underscores between words.
- Name the file "descriptively" so it is easy to find, and possibly re-use. 
  A name like **planner2_flight_screen.jpg** is much more useful than **image1.jpg**.
- To change an image, simply replace the file in the source tree and commit the change.
    
Display an image in a "common" article with a caption and target as shown below. Note the paths to the files are relative
to the current directory (hence the relative link back to **images** in the project root).

.. code-block:: rst

    .. figure:: ../../../images/image_file_name.jpg
       :target: ../_images/image_file_name.jpg

       Text for your caption


Display a wiki-specific image without a caption (or target link) as shown below. 
Note that the path is absolute, and relative to the source directory for the wiki.

.. code-block:: rst

    .. image:: /images/image_file_name.jpg

[copywiki destination="copter,plane,rover,planner,planner2,antennatracker,dev,ardupilot,mavproxy"]

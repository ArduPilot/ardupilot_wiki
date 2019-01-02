.. _common-wiki-editing-big-edit:

========================
Wiki Editing - Big Edits
========================

If you need to make a *significant* edit of an existing page, edit a significant number of pages, or create a new one, then it is best to 
verify changes before submitting your Github pull request. Typically you will need to work 
on your local computer rather than directly on Git, build and test your changes, and then submit
a pull request to submit your changes.

.. note:: 

    If you've already used Git, the fork, branch, submit model should be familiar. If not, you'll need 
    to learn basic git concepts like forking, branching, committing, pushing, rebasing.

There are a number of way of using Git/Github - including a number of different GUI and command line tools. 
The typical process for working with Git on the command line is:

#. `Fork the docs repo <https://github.com/ArduPilot/ardupilot_wiki>`__ 
   (if you haven't already done so).

   "Forking" is GitHub's term for copying a repository to your own account.
   The forked repository preserves information about the original project
   so you can fetch updates from it (and contribute changes back to it). If
   you want to contribute changes back to the main project you will need to
   first create your own fork of the main ardupilot_wiki repository.

   To fork the main repository:

   -  Log into Github and go to `github.com/ArduPilot/ardupilot_wiki <https://github.com/ArduPilot/ardupilot_wiki>`__.
   -  Click the "Fork" button on the upper right and follow the directions:

	.. image:: ../../../dev/source/images/APM-Git-Github-Fork-300x64.jpg
	   :target: ../../../dev/source/images/APM-Git-Github-Fork-300x64.jpg

   When your are finished there will be a new repository within your
   account: ``//github.com/YOURID/ardupilot_wiki``

#. Clone your fork of the repository to your local machine:

   .. code-block:: bash

       git clone https://github.com/YOURID/ardupilot_wiki.git

#. Track the ArduPilot wiki repository by adding a "remote" called "upstream":

   - Use the command below to see which repositories are being tracked

     .. code-block:: bash

	   cd ardupilot_wiki
       git remote -v

     At least "origin" should appear meaning your local repo is tracking your fork on github.com.

     .. code-block:: bash

	   origin  https://github.com/YOURID/ardupilot_wiki.git (fetch)
	   origin  https://github.com/YOURID/ardupilot_wiki.git (push)

   - You must track ArduPilot's main wiki repository in order to fetch updates so use the command below to add a "remote" called "upstream"

     .. code-block:: bash

	   git remote add upstream https://github.com/ArduPilot/ardupilot_wiki.git

   - Verify that the "upstream" repository is now referenced as well.

     .. code-block:: bash

	   git remote -v

   - The "upstream" repository should now be part of the remote repository list:  

     .. code-block:: bash

	   origin  https://github.com/YOURID/ardupilot_wiki.git (fetch)
	   origin  https://github.com/YOURID/ardupilot_wiki.git (push)
	   upstream        https://github.com/ArduPilot/ardupilot_wiki.git (fetch)
	   upstream        https://github.com/ArduPilot/ardupilot_wiki.git (push)	 

#. Create a branch in your local clone for your changes

   .. code-block:: bash

       git checkout -b hgw_my_well_named_branch

#. Make any changes needed and :ref:`test them locally <common_wiki_editing_guide_building_docs>`.

#. Add and commit your changes:

   .. code-block:: bash

       git add the_name_of_file_you_changed.rst
       git commit -m "A short explanation of the change"

#. Rebase your fork to the latest version of master and push your commits to
   the fork.

   .. code-block:: bash

       git fetch upstream master
       git rebase upstream/master
       git push origin hgw_my_well_named_branch

#. Open your clone's repository on the GitHub web page and 
   `Create a pull request on GitHub <https://help.github.com/articles/using-pull-requests>`__.
   You'll be making a pull request from your fork/branch to the
   ardupilot_wiki/master repository. If using the GitHub for Windows client,
   one convenient way to navigate to the repository/branch is to click
   one one of your commits and click the "github" (view this commit on
   github.com) button:

   .. image:: ../../../images/PullRequest_OpenWikiCloneOnGitHubWebPage.png
       :target: ../_images/PullRequest_OpenWikiCloneOnGitHubWebPage.png

#. On top of the web page select the "Pull Request" tab page, and then
   select the green "New pull request" button:

   .. image:: ../../../images/PullRequest_InitiateWikiPullRequest.png
       :target: ../_images/PullRequest_InitiateWikiPullRequest.png

#. The comparison should be between ardupilot_wiki:master and the new branch
   you created for the feature. The website probably has defaulted to your
   clone's master branch so click the "compare" combo box and change it to the
   correct branch:

   .. image:: ../../../images/PullRequest_InitiateWikiPullRequest2.png
       :target: ../_images/PullRequest_InitiateWikiPullRequest2.png

#. Check the list of change at the bottom of the page only includes your
   intended changes, then press "Create pull request".

Creating a new wiki page
========================

.. tip::

   First search the wiki to determine if there is already a wiki page
   about your topic — it is better to improve an existing topic than create
   a near-duplicate! 
   Also discuss it with the 
   Wiki Editors Discussion Group (``https://groups.google.com/forum/#!forum/ardu-wiki-editors``).

To create a new wiki page you will need to follow the same process as any other 
:ref:`big edit <common_wiki_editing_guide_big_edit>`.

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

-  Always :ref:`link to other common topics <common-editor-information-resource_how_to_link_to_other_topics>`
   using relative linking. This ensures that you will link to the correct common topic when the wiki article is copied.

How to get changes approved
===========================

Once you submit a pull request with your change the wiki team will review it. If we have any questions we'll add them to the request.

[copywiki destination="copter,plane,rover,planner,planner2,antennatracker,dev,ardupilot"]

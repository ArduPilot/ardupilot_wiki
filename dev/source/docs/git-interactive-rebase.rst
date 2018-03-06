.. _git-interactive-rebase:

=======================================
Interactive Rebase: cleaning up commits
=======================================

   .. image:: ../images/git-interactive-rebase-top-image.png

This page gives a quick introduction on how to `use interactive rebase <https://help.github.com/articles/about-git-rebase/>`__
to clean up commits on your :ref:`cloned <git-clone>`.  This is often required before :ref:`submitting a pull request to the main ArduPilot repo <submitting-patches-back-to-master>`.

These instructions give the command line instructions but the same operations can be done
using Git GUI editors including `SourceTree <https://www.atlassian.com/blog/sourcetree/interactive-rebase-sourcetree>`__.

Steps in an interactive rebase
------------------------------

- open up a terminal and cd to your :ref:`clone's <git-clone>` directory, checkout the :ref:`branch <git-branch>` you wish to cleanup

   ::

       cd ardupilot
       git checkout my-new-feature-branch

- use the git rebase -i command with the range of commits to be modified.  For example to modify any of the last 10 commits:

   ::

       git rebase -i "HEAD~10"

- in the terminal that pops up (see top image on this page) find the commits you wish to modify and replace their ``pick`` with one of the following letters:

  - ``s`` for **squash** will combine the commit with the previous commit.  In the example above "AP_HAL_F4Light: fixed some support scripts" and "AP_HAL_F4LIGHT: small fix (NFC)" will be "squashed" together into a single commit
  - ``r`` for **reword** allows you to change the subject and description of the commit.  Note that you cannot simply modify the subject line in the viewer.  First you mark it with ``r`` and later git will pop up another view to allow you to modify the subject and description
  - ``e`` for **edit** allows you to modify what is in the commit (i.e. add or take away changes)
  - ``d`` will remove the commit, alternatively you can just delete the line from the viewer
  - you may also move lines up or down to re-order the commits

- save and exit the viewer.  git will run through the file from top to bottom and pause when required to allow you to make the changes you have requested.

  - if a commit was marked with ``s`` for **squash**, another viewer will pop-up to ask you to provide the commit message and subject line.

     .. image:: ../images/git-interactive-rebase-squash.png

  - if a commit was marked with ``e`` for **edit**, the viewer will return to the regular command line.
    Edit the files as you wish, commit the changes and then tye ``git rebase --continue`` to allow git to continue
    processing the other changes.

When things go wrong
--------------------

If you change the order of commits or edit their contents merge conflicts may occur.  If you can resolve these with a
merge tool (like `TortoiseMerge <https://tortoisegit.org/>`__) then you can allow the interactive rebase to continue by typing:

   ::

       git rebase --continue

alternatively you may wish to abort the interactive rebase which will return your branch to its original condition before the interactive rebase was begun

   ::

       git rebase --abort       
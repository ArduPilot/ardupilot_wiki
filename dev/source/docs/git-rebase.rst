.. _git-rebase:

======================================
Rebasing: keeping your code up to date
======================================

   .. image:: ../images/git-rebase.png
       :width: 70%

As you develop, ArduPilot's master  branch is likely to be updated with "commits" from other developers.
You should `keep your local branches and fork up to date <http://robots.thoughtbot.com/keeping-a-github-fork-updated>`__
so that you are working with the latest code and to make it easier for maintainers to eventually merge your changes to ArduPilot's master branch.

In particular it is best to keep your local repo and forked repo's master branches up-to-date with ArduPilot's master branch.

"Rebasing" allows you to update a branch from ArduPilot's master branch and then re-apply any changes you may have made to the branch.

   .. image:: ../images/git-rebase2.png

The following commands can be entered directly into a Linux/OSX terminal or if using Windows, launch
the "Git Shell" or "Git Bash" utility that should already be :ref:`installed <git-install>`.

#. Navigate to your ardupilot git repository.

   ::

       cd <ardupilot-path>

#. Starting from here we will assume that you want to rebase you master branch on ArduPilot master branch. This process is the same on all branch, just replace master by your branch name. Ensure you are looking at your master branch or the branch that you want to rebase.

   ::

       git checkout master

#. Ensure your repository is connected to the upstream repository you
   forked from.

   ::

       git remote add upstream https://github.com/ArduPilot/ardupilot.git

#. Fetch changes from the upstream repository (this simply downloads changes into a local cache, it will not overwrite or modify your changes in any way)

   ::

       git fetch upstream

#. Rebase your current branch from the upstream's master branch.

   ::

       git rebase upstream/master

#. Update your local repo's submodules

   ::

       git submodule update --recursive

#. Check that the rebase was successfull. Using cmdline (``git log --oneline --decorate --all --graph`` , type q to exit) or a GUI tool like gitk, sourcetree, etc. Your commits should appear on top of ArduPilot master lastest commit, like show on previous picture.

#. Now push the updated branch to your github repository

   ::

       git push


When things go wrong
--------------------

The rebase step (step 5) above may fail for a few reasons:

- there are uncomitted changes to files in your local repo's branch.  Stage and commit these changes and then try again.
- there are merge conflicts meaning that another developer has changed the same lines of code as your new commits have changed.  There are two choices

 - abort the rebase which will restore your branch to how it was before you began the rebase, ``git rebase --abort``.  If you are happy to throw away your new commits in your local repo's branch you can "reset" your branch so that it looks exactly like upstream master using ``git reset --hard upstream/master``.
 - user a merge tool like `TortoiseGit <https://tortoisegit.org/>`__ to resolve the mergeconflict and then continue with the rebase using ``git rebase --continue``

The submodule update (step 6) occasionally fails if new submodules have been added.  ``git submodule init`` usually resolves this.

The push step (step 7) above may fail if your :ref:`fork <git-fork>`'s branch has already been updated previously.  You can "force" the push using ``git push -f`` which will overwrite your github fork's branch to look like your local repo's branch.

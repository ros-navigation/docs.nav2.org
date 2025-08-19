.. _distribution_sync_docs:

ROS Distribution Sync Process
#############################

This is the instructions for updating a Nav2 version in a ROS distribution sync.
We do this by bulk cherry picking commits from the ``main`` branch after they've had time to soak and apply them to the branch we looking to sync with the newest features.

0. Initial Setup
----------------

First we need to create a temporary workspace to perform the sync.
Typically, this is done in the ``/tmp`` directory so that it is cleaned up after a reboot.

.. code-block:: bash

  mkdir -p /tmp/sync_ws/src
  cd /tmp/sync_ws/src
  git clone https://github.com/ros-navigation/navigation2.git -b <distro>
  cd navigation2
  git checkout -b sync_<distro>

Then, we want to get a list of all of the commits we may be interested in backporting.
This can be found either in the git logs of the ``main`` branch or using the GitHub UX at https://github.com/ros-navigation/navigation2/commits/ and select the branch you are interested in.
If using GitHub, open two tabs - one for ``main`` and one for ``<distro>``.

Finally, we need to identify the date of the last sync.
This can be found looking in the commit messages for ``<distro>``'s branch or by looking at the release page for Nav2 and finding the date of the last release for that distribution.
Scroll back in the ``main`` and ``<distro>`` git logs to find the last sync commit to begin.

Once you have a local clone, the git logs, and the date of the last sync, we can begin the process.

1. Backporting Process
----------------------

Starting with the first commit in ``main`` after the last sync date, we will look at each commit individually.
Each commit should be evaluated for the following criteria:

- Cannot change default out-of-the-box behavior or existing configured robot behavior
- Cannot change major end-user ABI/API
- Cannot change message, action, or service definitions
- Cannot include risky changes that need more time to soak, such as for complete package rewrites or experimental features

If it does not do any of the above, it is a candidate for backporting.
It can be backported via the following command in your local branch:

.. code-block:: bash

  git cherry-pick <commit_hash>


If this merges without issue, you may proceed.
If not, we either must resolve the conflicts or roll back the commit.
We may need to roll it back if the merge conflict is too complex or it relies on a cascading chain of other commits that could not be included due to their dependency on commits that were unable to be backported.

.. note::

   Some commits may already have been backported by Mergify.
   Use the git logs for the ``<distro>`` branch to check if the commit has already been backported.
   If it has been, it can be skipped.

Once all commits to the present are evaluated, we can proceed to the next step.

2. Testing
----------

Before we can merge our changes, we need to ensure that everything is working correctly.
This involves compiling the code, running any unit tests, and performing functional testing.
Navigate back to the root of the workspace and build the code:

.. code-block:: bash

  cd ../../
  colcon build
  source install/setup.bash

After building, we can run the tests:

.. code-block:: bash

  colcon test


If all tests pass, we can launch the Nav2 bringup demos and navigate the TB3 and TB4 robots in both Gazebo and the loopback simulator for 10-15 minutes to validate functional stability of the backport.

3. Open a PR
------------

Once we have validated the changes, we can open a pull request (PR) against the ``<distro>`` branch.
Increment the package.xml versions for all packages using search and replace to increment one minor version (i.e. 1.1.X).
Open a PR with the title ``<Branch> Sync: <Date>`` and include a description of the changes made.
Let CI run and merge once passes for a second independent validation of compilation and testing.

4. Bloom Release
----------------

Once the PR is merged, we can release the changes to the distribution.
Create a release of the same version as the ``package.xml`` files targeting the ``<distro>`` branch.
Then, follow `Steve's Bloom Release Guide <https://gist.github.com/SteveMacenski/1c321d1c9edca096ae4d763d8327c2ee>`_ to perform the bloom release.

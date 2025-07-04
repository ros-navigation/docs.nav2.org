.. _distribution_release_docs:

ROS Distribution Release Process
################################

This is the instructions for releasing a new Nav2 version for a ROS distribution release.

.. note::

  We require ``osrf/ros:<distro>-desktop-full`` images to exist for a distribution before this process may be completed.
  This usually means we wait until the release date to perform Nav2's release and have it included in the first distribution sync.



0. Initial Freeze
-----------------

Look at the ROS 2 timeline for release and decide on a date to freeze Nav2 new contributions.
Mark any open tickets and/or pull requests with a new tag ``<distro>-release`` that we want to get in before the branch off.
These are the actions items to focus on before the chosen release date which would block release on time.

These tickets or pull requests should represent major features, meaningful bug fixes, or ABI/API breaking changes that are necessary to be included before the branch off so that those feature(s) or API change(s) are represented in the new distribution.

1. Local Functional Testing
---------------------------

Once pre-release images are available on `ros_oci_images <https://github.com/sloretz/ros_oci_images>`_, locally pull this docker image
In a workspace root containing Nav2 and ``nav2_minimal_turtlebot_simulations``, run the following:

.. code:: bash

    sudo docker pull ghcr.io/sloretz/ros-testing:<distro>-desktop-full

    sudo docker run -it --net=host --privileged -v .:/root/jazzy_ws --volume="${XAUTHORITY}:/root/.Xauthority" --env="DISPLAY=$DISPLAY" -v="/tmp/.gazebo/:/root/.gazebo/" -v /tmp/.X11-unix:/tmp/.X11-unix:rw --shm-size=1000mb ghcr.io/sloretz/ros-testing:<distro>-desktop-full

    apt update
    apt upgrade

    source /opt/ros/<distro>/setup.bash
    rosdep init
    rosdep update

    colcon build --parallel-workers 1
    colcon test --parallel-workers 1

This will obtain the testing docker image from ``ros_oci_images``, open a container, and make sure the system can build and pass unit and system-level testing.

Next, run each major Nav2 launch file and navigate the robot around.

* ``ros2 launch nav2_bringup tb3_simulation_launch.py``
* ``ros2 launch nav2_bringup tb4_simulation_launch.py``
* ``ros2 launch nav2_bringup tb3_loopback_sim_launch.py``
* ``ros2 launch nav2_bringup tb4_loopback_sim_launch.py``

Ensure that you see all the data, costmap, footprint, path planning, control and other topics visible.
Send Navigate To Pose, Navigate Through Poses goals and cancel goals periodically.
Shutdown the Nav2 lifecycle nodes, hit Control+C, and ensure the stack shuts down cleanly.

This step may be performed early to catch and resolve issues before the release process.
Once all testing passes, move onto the next steps.

2. Setup Nav2 Docker Images
---------------------------

Next, we need to setup Nav2's Nightly and Release docker image jobs in `nav2_docker <https://github.com/ros-navigation/nav2_docker>`_.

Update ``.github/workflows/build_images.yaml`` to contain the new distribution in the ``ros_distro`` matrix.
Remove any EOL distributions at this time.
Within the ``strategy.matrix.version`` map, please a new entry for the distribution being added.
The ``main_verison`` is ``1`` unless a new major version is set in Nav2's ``package.xml`` files.
The ``distro_version`` should be bumped by one from the last distribution representing the release in the format ``1.4.X``.
Remove any EOL distributions at this time.

Open the ``README.md`` and update the distributions to include this.

At this time, the build should fail because there is no ``distro`` branch on Nav2, which is our next step.


2. Branch Off Distribution
--------------------------

Now, we will setup the new branch and its CI system.

First, update ``.github/workflows/build_main_against_distros.yml`` by adding the new ``ros_distro`` to the matrix.
While the ``nav2_docker`` nightly image does not yet exist, it will by the end of this step.
Remove any EOL distributions at this time.

Next, bump the ``main`` branch's distribution number to the same set in Step 2 above (i.e. ``1.4.0``).
Add the distribution branch to ``.github/workflows/update_ci_image.yaml`` so that future pushes will result in CI image updates.
Remove any EOL distributions at this time.

Finally, create the new distribution branch from ``main`` and push to the server.
Go into the GitHub Actions tab on ``nav2_docker`` and retrigger its build job.
The nightly and release jobs should now exist for the new distribution and return successfully (validate this).

3. Setup Branch CI
------------------

The final change to the branch is to setup CI so PRs targeting it can be successfully built.
In the new distribution branch, update the files for CI (`Humble Example <https://github.com/ros-navigation/navigation2/commit/4eb4ee01967a3b881c05d962ffd856c668b2e4c0>`_).

* Update ``.circleci/config.yml`` to use the new distribution image (replace ``ghcr.io/ros-navigation/navigation2:main``).
* Update ``.devcontainer/devcontainer.json`` to ``cacheFrom`` the new distribution image (replace ``ghcr.io/ros-navigation/navigation2:main``).
* Update ``Dockerfile`` to use the new distribution's image rather than ``rolling``.
* Update ``tools/distro.Dockerfile`` to use the new distribution's image rather than ``rolling`` in 3x places.

Then, retrigger the Update CI Image workflow in Nav2's GitHub Actions tab, it should now also be successful.
Open a dummy PR against the new distribution branch and ensure that it builds successfully.


4. Update Auxiliary Projects
----------------------------

Nav2 has a number of auxiliary projects that also need to be updated for a new distribution.
These include:

* ``nav2_minimal_turtlebot_simulation``
* ``navigation2_tutorials``

For each:

* Update the package.xml for a new distribution version
* Create a new branch from ``main`` for the distribution
* Update CI on the new branch to use this new distribution image
* Review and update the readme as needed

5. Run Bloom Release
--------------------

Once the new branches, versions, and CI are setup and ready, we can run the bloom release process.
Run the following command to create a new release for the distribution for each Nav2 repository (Nav2, Minimal Turtlebot Simulation, SLAM Toolbox, NPVL, STVL etc).

.. code:: bash

  bloom-release navigation2 --rosdistro distro --track distro --new-track --edit

Be patient, this will take a while to run.

6. Nav2 Docker Build
--------------------

To allow the ``nav2_docker`` build of the released version in Step 5, we need to enable the first build to pass the latest tag check in the workflow.
You should see that the nightly of this distribution works, but the release version is failing with ``Error: No matching package versions found.``.
To resolve, comment out the ``exit 1`` in the ``latest_version`` validity check.
Once the job turns over, revert this commit to reintroduce the error.

7. Announcements
----------------

Finally, we can announce the updates!
Create a new migration guide page on the Nav2 website for contributors to populate with notable changes in the next distribution cycle.
Update the Roadmap page with a new table of projects and features to be added over the next distribution.

Make announcements on Slack, ROS Discourse, and LinkedIn to announce the new distribution and its major new features.
Gifs, videos, and images are always welcome to be included in the announcements!

Identify tickets to address or PRs to merge for the next distribution.
Make a call for contributions on the roadmap items, these strategic items, or other ``good-first-issues``.

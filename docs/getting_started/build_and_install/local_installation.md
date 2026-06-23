# Local Installation

## Dependencies

To use Nav2, you'll first need to build or install ROS 2 and related development tools, including: `colcon`, `rosdep` and `vcstool`. There are a few ways to install and build them:

=== "ROS 2 Released Distribution Binaries"

    For more information on building or installing ROS 2 distros, see the official documentation:

    - [ROS 2 Installation](https://docs.ros.org/en/jazzy/Installation.html)
    - [Install development tools and ROS tools](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Development-Setup.html#install-development-tools-and-ros-tools)

    Once your environment is ready, source the ROS 2 setup file:

    ```bash
    source /opt/ros/<distro>/setup.bash
    ```

=== "ROS 2 from Source"

    For more information on building ROS 2 from source, see the official documentation:

    - [ROS 2 Building from source](https://docs.ros.org/en/jazzy/Installation.html#building-from-source)

    Once your environment is ready, source the ROS 2 setup file:

    ```bash
    source <ros_ws>/install/setup.bash
    ```

## Nav2 Project

!!! warning

    The branch naming schema for Nav2 is organized by ROS distro, while the default branch for Rolling is `main`.

Similarly, you can install the Nav2 project in different ways:

=== "Nav2 Released Distribution Binaries"

    !!! warning

        Nav2 does not currently release binaries on rolling, so it must be [build from source](#__tabbed_2_2).

    Nav2 and its dependencies are released as binaries.
    You may install it via the following to get the latest stable released version:

    === "Jazzy and newer"

        ```bash
        sudo apt update
        sudo apt install \
            ros-$ROS_DISTRO-navigation2 \
            ros-$ROS_DISTRO-nav2-bringup \
            ros-$ROS_DISTRO-nav2-minimal-tb\*
        ```

        !!! tip "Hint"

            For more examples on building Nav2 from released distribution binaries, checkout [distro.Dockerfile](https://github.com/ros-navigation/navigation2/blob/jazzy/tools/distro.Dockerfile).

    === "Iron and older"

        ```bash
        sudo apt update
        sudo apt install \
            ros-$ROS_DISTRO-navigation2 \
            ros-$ROS_DISTRO-nav2-bringup \
            ros-$ROS_DISTRO-turtlebot3\*
        ```

        !!! tip "Hint"

            For more examples on building Nav2 from released distribution binaries, checkout [distro.Dockerfile](https://github.com/ros-navigation/navigation2/blob/iron/tools/distro.Dockerfile).


=== "Nav2 from Source"

    Once your environment is setup, clone the Nav2 repo, install all dependencies, and build the workspace:

    ```bash
    NAV_DISTRO=${ROS_DISTRO/rolling/main}
    mkdir -p ~/nav2_ws/src && cd ~/nav2_ws
    git clone https://github.com/ros-navigation/navigation2.git --branch $NAV_DISTRO ./src/navigation2
    git clone https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation.git --branch $NAV_DISTRO ./src/nav2_minimal_turtlebot_simulation
    rosdep install -r -y \
    --from-paths ./src \
    --ignore-src
    colcon build \
    --symlink-install
    ```

    !!! note "Note for Rolling"

        Since rolling development source tracks the latest upstream ROS 2 branches, builds may occasionally fail due to ABI/API breaking changes being actively developed in ROS 2 core packages. If you encounter build failures, consider using the [Released Distribution Binaries](#__tabbed_2_1) approach instead or submit the patch to Nav2.

    You can then source Nav2 setup file to get ready for demonstrations!

    ```bash
    source ~/nav2_ws/install/setup.bash
    ```

    It is safe to ignore the rosdep error of from the missing `slam_toolbox` key.

    !!! tip "Hint"

        If you are using `rmw_zenoh_cpp` and want to run tests without a Zenoh router, you can enable isolated tests by building with:

        ```bash
        colcon build --symlink-install \
        --cmake-args -DUSE_ISOLATED_TESTS=ON
        ```

    !!! tip "Hint"

        For more examples on building Nav2 from rolling development source, checkout [distro.Dockerfile](https://github.com/ros-navigation/navigation2/blob/jazzy/tools/distro.Dockerfile).

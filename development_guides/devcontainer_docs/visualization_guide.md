# Visualization Guide

While using, debugging, or developing the project, you'll likely need to visualize internal and external states of a robotic system, such as transform trees, sensor data, simulated ground truths, etc. Although its easiest to consider dev containers as headless environments, there are still yet a myriad of methods to display visualizations using local, remote, or web-based graphical user interfaces.

![Nav2 running in Dev Container with Foxglove and GzWeb running as PWAs](/images/devcontainers/nav2_foxglove_gzweb_pwa.png)

For example, the screenshot above depicts a simulated demo, launched from the `nav2_simple_commander` package, running from inside a dev container, opened via VS Code, with Foxglove Studio and GzWeb respectively visualizing Nav2 and Gazebo from split web browser windows.

:::{hint}
This demo can be launched from a headless dev container by sourcing the overlay workspace, and disabling the gzclient and rviz GUIs:
``` bash
source $OVERLAY_WS/install/setup.bash
source /usr/share/gazebo/setup.sh
GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(find /opt/ros/$ROS_DISTRO/share \
  -mindepth 1 -maxdepth 2 -type d -name "models" | paste -s -d: -)
ros2 launch nav2_simple_commander security_demo_launch.py \
  use_rviz:=False headless:=False
```
:::

:::{seealso}
Checkout the Nav2 Simple Commander [Examples and Demos](/commander_api/index.rst) for more documentation on the depicted simulation.
:::

There are a few ways to visualize the project within dev containers, each with their own pros and cons. Ordered by increasing complexity:

- [**Web Apps**](#web-apps)
  - Simple and remotely accessible from any web browser, while providing a responsive interface
- [**Native Apps**](#native-apps)
  - Performative over local networks, while providing a native interface and GPU acceleration
- [**VNC Desktop**](#vnc-desktop)
  - Flexible and remotely accessible from any client, while providing robust integration
- [**Display Socket**](#display-socket)
  - Performative and flexible for local development, while providing robust integration

The decision matrix below provides a rough comparison of the various options listed. Check each section for further details.

| Approach vs Utility | Web Apps | Native Apps | VNC Desktop | Display Socket |
|---------------------|----------|-------------|-------------|------------|
| Setup Simplicity    | 🟢        | 🟠           | 🟠           | 🔴          |
| Tooling Flexibility | 🟠        | 🟠           | 🟢           | 🟢          |
| Robust Integration  | 🟢        | 🔴           | 🟢           | 🟢          |
| Responsive UI/UX    | 🟢        | 🟢           | 🔴           | 🟠          |
| GPU Acceleration    | 🟢        | 🟢           | 🔴           | 🟠          |
| Remote Connectivity | 🟢        | 🟠           | 🟢           | 🔴          |
| Bandwidth Efficient | 🟠        | 🔴           | 🟠           | 🔴          |

> Legend: 🟢 Good | 🟠 Modest | 🔴 Poor

(web-apps)=
## Web Apps

By far one of the easiest ways to quickly visualize the project is by using web client interfaces via Foxglove Studio and GzWeb. These apps are accessible from any web browser, regardless of the host operating system or whether the dev container is running locally or remotely. This is useful for:

- Remote Development
  - Enables
  - Applicable
  - Necessary
- Education
  - Enables
  - Applicable
  - Necessary

:::{seealso}
More details on Foxglove Studio and GzWeb can be found here:

- [Foxglove Studio](https://foxglove.dev/studio)
  - Robotics visualization and debugging with customizable layouts
- [Foxglove Bridge](https://github.com/foxglove/ros-foxglove-bridge)
  - Foxglove WebSocket bridge for ROS 1 and ROS 2
- [GzWeb](https://gazebosim.org/gzweb)
  - Web client for Gazebo classic simulation
:::

visualizer stage

bridge

statefull
gracefull reconect


  - Pro: Applicable for developing either locally on a host or remotely, e.g. SSH or Codespaces
  - Con: Limited to modest message rates and sizes based on network bandwidth and bridge performance

basdwith (large by more finite) vs resource usage

### VS Code Tasks

### PWA progressive web app

:::{tip}
You can also install Foxglove Studio and GzWeb as PWAs, allowing you to resize them like native apps separate from the browser. Just remember that PWAs are fixed to the URL they were installed from, so you'll need to reinstall if the URLs or port numbers ever change.
:::

:::{seealso}
More details on Progressive Web Apps installation can be found here:

- [Use Progressive Web Apps](https://support.google.com/chrome/answer/9658361)
  - with Google Chrome (i.e. on Computer or Android)
- [PWA Installation](https://web.dev/learn/pwa/installation)
  - with any browser (i.e. Firefox, or Safari on iOS)
:::

(native-apps)=
## Native Apps

  - Pro: Leverage natively installed desktop applications for rendering, e.g. Rviz, Gzclient, etc.
  - Con: Fragile when missing or changing interface and resources, e.g. message types or 3D assets

host networking for ROS ports
 gzserver
native foxglove

resources and interfaces

(vnc-desktop)=
## VNC Desktop

  - Pro: Applicable when developing remotely over moderate yet finite network bandwidth
  - Con: Not performative or responsive, e.g.

basdwith (large by more finite) vs resource usage

https://github.com/devcontainers/features/tree/main/src/desktop-lite

:::{seealso}
This approach is similarly used by other robotic web based services. More examples can be found here:

- [The Construct](https://www.theconstructsim.com)
  - An educational and training platform for robotics development and simulation
- [AWS RoboMaker](https://aws.amazon.com/robomaker)
  - A Continuos Integration and Deployment service for robotic applications and simulations
:::

(display-socket)=
## Display Socket

  - 3D hardware acceleration
  - Con: Difficult to configure, connect to, and enable GPU hardware acceleration


### Local Host

### Remote Host

https://github.com/microsoft/vscode-remote-release/issues/8031

https://docs.docker.com/config/containers/resource_constraints/#gpu

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

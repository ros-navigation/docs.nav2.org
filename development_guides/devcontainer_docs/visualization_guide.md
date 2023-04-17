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
Checkout the Nav2 Simple Commander [Examples and Demos](/commander_api/index.rst#examples-and-demos) for more documentation on the depicted simulation.
:::

- Web Visualization
  - Simple and accessible
- Native Desktop
  - Performative but limited
- X11 Forwarding
  - Performative but complex
- VNC Desktop
  - Simple but not performative

| Approach vs Utility | Web Apps | Native Apps | X11 Forward | VNC Client |
|---------------------|----------|-------------|-------------|------------|
| Tooling Flexibility | 🟠       | 🟠          | 🟢          | 🟢         |
| Robust Integration  | 🟢       | 🔴          | 🟢          | 🟢         |
| Responsive UI/UX    | 🟢       | 🟢          | 🟠          | 🔴         |
| HW Acceleration     | 🟢       | 🟢          | 🟠          | 🔴         |
| Remote Connectivity | 🟢       | 🟠          | 🔴          | 🟢         |
| Bandwidth Efficient | 🟠       | 🔴          | 🔴          | 🟠         |

> Good: 🟢 | Modest: 🟠 | Poor: 🔴

Summary list and comparison of various visualization options for development containers.

gazebo
rviz
rqt
foxglove

## Web Visualization


basdwith (large by more finite) vs resource usage

## VS Code Tasks

### PWA progressive web app

https://support.google.com/chrome/answer/9658361?hl=en&co=GENIE.Platform%3DDesktop

https://support.google.com/chrome/answer/9658361?hl=en&co=GENIE.Platform%3DAndroid

## Native Desktop

host networking for ROS ports
 gzserver
native foxglove

resources and interfaces

## X11 Forwarding

### Local Host

### Remote Host

https://github.com/microsoft/vscode-remote-release/issues/8031

https://docs.docker.com/config/containers/resource_constraints/#gpu

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html


## VNC Desktop

basdwith (large by more finite) vs resource usage

https://github.com/devcontainers/features/tree/main/src/desktop-lite

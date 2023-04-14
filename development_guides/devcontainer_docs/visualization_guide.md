# Visualization Guide


``` bash
source $OVERLAY_WS/install/setup.bash
source /usr/share/gazebo/setup.sh
GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(find /opt/ros/$ROS_DISTRO/share \
  -mindepth 1 -maxdepth 2 -type d -name "models" | paste -s -d: -)
ros2 launch nav2_simple_commander security_demo_launch.py use_rviz:=False headless:=False

ros2 launch ./security_demo_launch.py use_rviz:=False headless:=False
```

- Web Visualization
  - Simple and accessible
- Native Desktop
  - Performative but limited
- X11 Forwarding
  - Performative but complex
- VNC Desktop
  - Simple but not performative

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

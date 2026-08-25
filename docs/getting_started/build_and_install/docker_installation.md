# Docker Installation

!!! tip

    For a *repeatable*, *reproducible* and *streamlined* development experience, check out [Dev Containers][dev-containers]!

## Docker Container Images

Building Nav2 using Docker container images provides a repeatable and reproducible environment to automate and self document the entire setup process. Instead of manually invoking the development tools as described in the [Local Installation][local-installation] guide, you can leverage the project's Dockerfiles to build and install Nav2 for various distributions.

!!! info "See also"

    For more information on installing Docker or leaning about Dockerfiles, see the official documentation:

    - [Docker Engine](https://docs.docker.com/engine/install)
    - [Dockerfile reference](https://docs.docker.com/engine/reference/builder)

Once your system is setup, you can build the Nav2 Dockerfile from the root of the repo:

```bash
export ROS_DISTRO=rolling # replace 'rolling' with your distribution version

NAV_DISTRO=${ROS_DISTRO/rolling/main}
git clone https://github.com/ros-navigation/navigation2.git --branch $NAV_DISTRO
docker build --tag navigation2:$ROS_DISTRO \
  --build-arg FROM_IMAGE=ros:$ROS_DISTRO \
  --build-arg OVERLAY_MIXINS="release ccache lld" \
  --cache-from ghcr.io/ros-navigation/navigation2:$NAV_DISTRO \
  ./navigation2
```

The [docker build](https://docs.docker.com/engine/reference/commandline/build/) command above creates a tagged image using the *Dockerfile* from the context specified using the path to the repo, where build-time variables are set using additional arguments, e.g. passing a set of [colcon mixins](https://github.com/colcon/colcon-mixin-repository) to configure the workspace build. Check the `ARG` directives in the *Dockerfile* to discover all build-time variables available. The command also specifies an [external cache source](https://docs.docker.com/engine/reference/commandline/build/#cache-from) to pull the latest cached image from Nav2's [Container Registry](https://github.com/ros-navigation/navigation2/pkgs/container/navigation2) to speed up the build process.

!!! tip

    The images cached from above are used for Nav2 CI, but can also be used with Nav2 [Dev Containers][dev-containers]!

## Using Pre-built nav2_docker Images

For contributors who want to get started quickly, Nav2 provides pre-built Docker images through the [nav2_docker](https://github.com/ros-navigation/nav2_docker) repository. These images are available for all active ROS 2 distributions and come in two variants:

- **Nightly images**: Built from the latest Nav2 branch for each distribution
- **Release images**: Built from the latest officially released Nav2 version

Supported distributions include Humble, Jazzy, Kilted, and Rolling.

To use a pre-built nav2_docker image for development:

```bash
# Pull a release image (example: Jazzy 1.3.1)
docker pull ghcr.io/ros-navigation/nav2_docker:jazzy-1.3.1

# Or pull a nightly image for the latest features
docker pull ghcr.io/ros-navigation/nav2_docker:rolling-nightly
```

You can then use these images directly for development or as base images for your own custom deployments by specifying them in your Dockerfile.

# Dev Containers

Alternatively you can use dev containers to build the project if you prefer a streamlined setup experience. This means you can use the same tools and dependencies as the rest of the team, including our Continuous Integration (CI) workflows, without worrying about installing dependencies on your host machine. Additionally, using Dev Containers makes it simple to switch between local or remote development environments, such as GitHub Codespaces. More info on Dev Containers can be found here:

- [Development Containers](https://containers.dev/)
  - An open specification for enriching containers with development specific content and settings
- [Developing inside a Container](https://code.visualstudio.com/docs/remote/containers)
  - Learn how to use Visual Studio Code to develop inside a Docker container
- [GitHub Codespaces overview](https://docs.github.com/en/codespaces/overview)
  - A development environment hosted in the cloud

## What, why, how?

Lets briefly explain what dev containers are, why you should use them, and how they work. Here, we'll assume the use of VS Code, but the same concepts apply to other supporting tools and services, including alternative CLIs, IDEs, etc. such as:

- [Dev Container CLI](https://github.com/devcontainers/cli)
  - A reference implementation for the open specification
- [JetBrains Space | Develop in Dev Environment](https://www.jetbrains.com/help/space/develop-in-a-dev-environment.html)
  - Using Dev Containers with JetBrain based products
- [Supporting tools]()
  - List of tools and services supporting the development container specification

### What is a Dev Container?

A dev container is a Docker container that has all the tools and dependencies you need to develop the project. It runs in a self-contained environment and is isolated from other containers and your host machine. This lets you reliably develop for the project anywhere, notably for linux distributions targeted by ROS, regardless of your host machine's operating system.

### Why use a Dev Container?

A dev container provides a common and consistent development environment. It ensures that everyone on the team is using the same tools and dependencies. It also makes it easy to switch between projects because each project can use a different container. This is especially useful if you work on multiple projects that use different versions of the same tools and dependencies, such as different versions of ROS.

### How do Dev Containers work?

When you open the project in VS Code, VS Code checks for the dev container configuration nested within the `.devcontainer` folder under the project's root directory. If it finds one, it can prompt you to reopen the project in a container. If you choose to do so, it launches the container, connects to it, and mounts your project folder inside the container. You can then use VS Code in the container just as you would locally. While setting up the container, VS Code can also attempt to passthrough useful aspects of your local environment, such as git user configurations, X11 sockets, and more.

This is quite similar to earlier tools used to customize and run docker containers for development:

- [rocker | ROS + Docker](https://github.com/osrf/rocker)
  - A tool to run docker containers with overlays and convenient options for things like GUIs etc.
  - Developed by [Open Robotics](https://www.openrobotics.org/)
- [ADE Development Environment](https://ade-cli.readthedocs.io/en/latest/)
  - A modular Docker-based tool to ensure developers have a common, consistent development environment
  - Developed by [Apex.AI](https://www.apex.ai/)

## Prerequisites

To use dev containers, you'll need the following:

- [Docker Engine](https://docs.docker.com/engine/install/) installed and running on the host machine
- [Visual Studio Code](https://code.visualstudio.com/) installed on any remote machine
- [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension installed in VS Code

Alternatively, you could use GitHub Codespaces directly from the project repo:

- [Creating a codespace for a repository](https://docs.github.com/en/codespaces/developing-in-codespaces/creating-a-codespace-for-a-repository?tool=webui)
  - How to create a codespace for repository via GitHub CLI, VS Code, or Web browser

## Creating Dev Containers

Creating a dev container is as simple as opening the project in VS Code by either: following the prompts to reopen the project in a container, or explicitly opening the command palette and selecting `Remote-Containers: Reopen in Container`. This will create a new container, install any extensions specified in the project's default `.devcontainer/devcontainer.json` config file, and mount the project's root directory within the container. Once the container is created, VS Code will connect to it and you can start developing.

While you wait, feel free to stretch your legs, grab a coffee, or continue to read the following subsections to learn more about how dev containers are constructed. Note: opening the output log in VS Code allows you to watch how this sausage is made.

### Building the image

When first creating Dev Containers, any supporting tool or service used will download and build the docker images needed to run the container. This includes pulling any parent images the project's Dockerfile builds `FROM`, as well as any tags or layers declared via `cacheFrom`, as specified in the chosen `devcontainer.json` config file. This can take a while, but only needs to be done once, or at least not again until such layers are updated and pushed to the image registry.

Specifically, for this project, the default `devcontainer.json` file targets the `dever` stage within the project's root Dockerfile, the stage that also includes handy tools for developing the project, such as bash auto completion. This stage is in turn built `FROM` the `builder` stage, the stage that only includes the dependencies needed for building the project, as reused by the project's CI.

To speed up the initial build, images layers from this `builder` stage are cached by pulling the same image tag used by the project's CI, hosted from the image registry. This ensures your local dev container replicates our CI environment as close as possible, while benefiting from any cached work preemptively performed by the CI. Yet, this still allows you to customize the project's Dockerfile and rebuild the container, without needing to update CI images to reflect your local modifications.

Once the base image from the target stage is built, the supporting tool or service may then add additional layers to the image, such as installing additional [features](https://containers.dev/features) or customizations. For VS Code, this also includes some fancy file caching for any extensions to install later. Once this custom image is built, it is then used to start the dev container.

### Starting the container

When first creating Dev Containers, any supporting tool or service will invoke a sequence of commands specified in the chosen `devcontainer.json` config file. This can take a while, but only needs to be done once, or at least not again until the container is rebuilt, triggered by either updating the Dockerfile, base image, or `.devcontainer/` config.

Specifically, for this project, the default `devcontainer.json` config executes the `onCreateCommand` to initially colcon cache, clean, and build the overlay workspace for the project. This ensures the workspace is precompiled and ready to use, while also ensuring any changes to the project's source code are reflected in the container. This is especially useful for:

- IntelliSense
  - Enables VS Code extensions to parse auto generated code
  - Applicable for ROS package defining messages and services files
  - Necessary for code modeling, navigation, and syntax highlighting
- Caching
  - Enables Codespace Prebuilds to cache the workspace artifacts
  - Applicable for reducing startup time when spawning new Codespaces
  - Necessary for limiting costs from CPU and storage usage

While the colcon workspace is being built, VS Code will simultaneously install any specified extensions and settings. Next the `updateContentCommand` is executed, which reruns whenever the container is started or restarted. Specifically, for this project, this command re-cleans and re-builds the same colcon workspace as before, but only for invalidated packages detected by colcon cache using the lockfiles initialized during the `onCreateCommand`. This caching behavior also replicates the project's CI workflow. This is especially useful for:

- Branching
  - Enables caching of workspace artifacts when switching between branches
  - Applicable for reviewing pull requests without rebuilding entire container
  - Necessary for reducing startup time when spawning new Codespaces

Finally, the `postCreateCommand` is executed, which also reruns whenever the container is started or restarted. Specifically, for this project, this command makes a last few tweaks to the user's environment to improve the development experience.

To speed up subsequent startups, volumes are mounted to store the ccache directory, while the environment is set to enable ccache via colcon mixins.

## Using Dev Containers

##

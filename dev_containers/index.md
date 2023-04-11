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
- [Supporting tools](https://containers.dev/supporting)
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

Specifically, for this project, the default `devcontainer.json` file targets the `dever` stage within the project's root Dockerfile, the stage that also includes handy tools for developing the project, such as bash auto completion. This stage is in turn built `FROM` the `builder` stage, the stage that only includes the dependencies needed for building the project, as reused by the project's CI. For example, the `dever` stage modifies `/etc/bash.bashrc` to automatically source `install/setup.bash` from the underlay workspace, ensuring all VS Code extensions are loaded with the correct environment, while avoiding any race conditions during installation and startup.

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

To speed up subsequent startups, volumes are mounted to the container store a persistent ccache directory, while the environment is set to enable [ccache](https://ccache.dev/) via [colcon mixins](https://github.com/colcon/colcon-mixin-repository). Additionally, the container is granted [privileged](https://docs.docker.com/engine/reference/commandline/run/#privileged) capabilities and connected using the [host](https://docs.docker.com/network/host/) network mode. This is especially useful for:

- Hybrid development
  - Enables connecting ROS nodes external to the container
  - Applicable for debugging or visualizing distributed systems
  - Necessary for DDS discovery and shared memory transport
- Device connectivity
  - Enables hardware forwarding from host machine to container
  - Applicable for ROS package using sensors and actuators
  - Necessary for some GPU drivers and USB devices

Note that these `runArgs` in the `devcontainer.json` config can be further customized, either expanded or or narrowed in scope, to better suit your desired development environment. The current configuration is merely the project default in order to be the most flexible and useful for the widest range of development use cases.

## Using Dev Containers

Once the dev container has been created and setup completed, VS Code will open a new workspace directly from the project's root directory, which itself is mounted within the source directory in the overlay colcon workspace. From here you can build, test, and debug the project as you normally would, with the added benefit of having the project's dependencies, intellisense, linters, and other extensions pre-configured and ready to use.

So to build or test the project, simply open a terminal, cd to the root of the colcon workspace, and run the usual colcon commands. You may also want to incorporate the same colcon verbs used by the setup commands from the `devcontainer.json` config file to further automate your local development workflow.

### Terminals

If you prefer using alternate terminal emulators, rather than the built-in VS Code terminal, you can also open a separate shell session directly using devcontainer CLI, or simply via docker exec. Note that new shell sessions started via `exec` will not automatically inherit the same environment setup by the `postCreateCommand` from the `devcontainer.json` config file. So you may need to manually source any necessary scripts, such as `install/setup.bash` from the underlay workspace.

- [Dev Container CLI](https://code.visualstudio.com/docs/devcontainers/devcontainer-cli)
  - `devcontainer exec --workspace-folder <path-to-workspace> bash`
- [docker exec
](https://docs.docker.com/engine/reference/commandline/exec/)
  - `docker exec -it <container-id> bash`

### Lifecycle

While using the dev container, try and keep in mind the lifecycle of the container itself. Specifically, containers are ephemeral, meaning they are normally destroyed and recreated whenever the dev environment is rebuilt or updated. Subsequently, a best practice is to avoid storing any persistent data within the container, and instead utilize the project's source directory, or a separate mounted volume. When altering the development environment inside the container, try to remember to codify your changes into the Dockerfile, or the `devcontainer.json` config file, so that they can be easily reproduced and shared with others. This is particularly important when the host machine is inherently ephemeral as well, as the case may be when using cloud based environments such as Codespaces, so be sure to commit and push local changes to a remote repository:

- [The codespace lifecycle](https://docs.github.com/en/codespaces/getting-started/the-codespace-lifecycle)
  - Maintain your data throughout the entire codespace lifecycle

### Rebuilding

From time to time, you may need to rebuild the dev container, either because the base image, or `.devcontainer/` config was updated, or simply out of wanting a new fresh development environment. To do so, simply open the Command Palette (Ctrl+Shift+P) and select the `Remote-Containers: Rebuild Container` command. For example, you may need to rebuild the dev container when:

- Pulling newer images from a container registry
  - specifically, image tags built `FROM` in the Dockerfile
  - or tags listed under `cacheFrom` in `devcontainer.json`
  - periodically done manually to ensure local environment reflects CI
- Updating the dev container configuration
  - specifically when modifying dependent stages in the `Dockerfile`
  - or when modifying `./devcontainer` files and commands
  - where build cache reuse correlates with severity of changes made

If necessary, you can also rebuild the container from scratch, e.i. without caching from docker, by selecting the `Remote-Containers: Rebuild Container Without Cache` command instead. Rebuilding without caching may be necessary when:

- Needing to update the base image
  - specifically if dev container configurations remain unmodified
  - to forcefully rerun a `RUN` directive in the Dockerfile
  - such as unchanged `apt upgrade` or `rosdep update` commands

Specifically, for this project, volumes remain unaffected by this rebuilding process: i.e. those used to mount the ccache directory. The management of these volumes is left for the developers discretion, and can be done via the [Docker CLI](https://docs.docker.com/engine/reference/commandline/cli/), or the [VS Code Docker extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker). However, other projects may of course handle this differently, so be sure to check the `./devcontainer` configuration to inspect how various container resources may be managed.

## Security

A word of caution when using dev containers: they are powerful tools, but can be a security concern, as the capability of arbitrary code execution facilitated by IDE extensions to enable such automation and convenience remains inherently dual use. Before launching a dev container, ensure you trust the workspaces and authors. For example, when reviewing a pull request, verify patches remain benign and do not introduce any malicious code. Although such vigilance is merited whenever compiling and running patched code, using containers with either elevated privileges or filesystem access renders this diligence even more prudent. More info on trusting workspaces and extensions in general can be found here:

- [Workspace Trust](https://code.visualstudio.com/docs/editor/workspace-trust)
  - VS Code user guid on trusting and configure workspaces

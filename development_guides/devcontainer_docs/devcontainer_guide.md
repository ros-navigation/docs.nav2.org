(devcontainer-guide)=
# Dev Container Guide

In this guide, we'll walk through the process of creating and using dev containers for the project. While included subsections will provide greater detail on the various aspects of the process, complete comprehension of the entire guide is not required to get started, but is recommended for those interested in how dev containers work, or how to customize and optimize them for their own personal workflows.

## Creating Dev Containers

Before creating a dev container, you'll want to choose the exact configuration to use. By default the `.devcontainer/devcontainer.json` configuration is selected, however you can also choose any other `devcontainer.json` file in the `.devcontainer/` directory, where such configurations can be nested to provide greater customization: either by targeting different stages within different Dockerfiles, overriding any merged metadata or default properties, or inclusion of additional extensions and alternate commands.

:::{seealso}
The specification, reference, and schema for the `devcontainer.json` config file format can be found here:
- [Specification](https://containers.dev/implementors/spec)
  - Development Container Specification
- [Reference](https://containers.dev/implementors/json_reference)
  - Metadata and properties reference
- [Schema](https://containers.dev/implementors/json_schema)
  - JSON schema for `devcontainer.json`
:::

### Building the image

When first creating Dev Containers, any supporting tool or service used will download and build the docker images needed to run the container. This includes pulling any parent images the project's Dockerfile builds `FROM`, as well as any tags or layers declared via `cacheFrom`, as specified in the chosen `devcontainer.json` config file. This can take a while, but only needs to be done once, or at least not again until such layers are updated and pushed to the image registry.

Specifically, for this project, the default `devcontainer.json` file targets the `dever` stage within the project's root Dockerfile, the stage that also includes handy tools for developing the project, such as bash auto completion. This stage is in turn built `FROM` the `builder` stage, the stage that only includes the dependencies needed for building the project, as reused by the project's CI. For example, the `dever` stage modifies `/etc/bash.bashrc` to automatically source `install/setup.bash` from the underlay workspace, ensuring all VS Code extensions are loaded with the correct environment, while avoiding any race conditions during installation and startup.

To speed up the initial build, images layers from this `builder` stage are cached by pulling the same image tag used by the project's CI, hosted from the image registry. This ensures your local dev container replicates our CI environment as close as possible, while benefiting from any cached work preemptively performed by the CI. Yet, this still allows you to customize the project's Dockerfile and rebuild the container, without needing to update CI images to reflect your local modifications.

:::{seealso}
More details on the project's CI and related docker image registry can be found here:

- [Chronicles of Caching and Containerising CI for Nav2](https://vimeo.com/649647161/5b0c278e6c)
  - Video presentation from ROS World 2021 - Ruffin White
:::

Once the base image from the target stage is built, the supporting tool or service may then add additional layers to the image, such as installing additional [features](https://containers.dev/features) or customizations. For VS Code, this also includes some fancy file caching for any extensions to install later. Once this custom image is built, it is then used to start the dev container.

### Starting the container

When first creating Dev Containers, any supporting tool or service will invoke a sequence of commands specified in the chosen `devcontainer.json` config file. This can take a while, but only needs to be done once, or at least not again until the container is rebuilt, triggered by either updating the Dockerfile, base image, or `.devcontainer/` config.

Specifically, for this project, the default `devcontainer.json` config executes the `onCreateCommand` to initially colcon cache, clean, and build the overlay workspace for the project. This ensures the workspace is precompiled and ready to use, while also ensuring any changes to the project's source code are reflected in the container. This is useful for:

- IntelliSense
  - Enables VS Code extensions to parse auto generated code
  - Applicable for ROS package defining messages and services files
  - Necessary for code modeling, navigation, and syntax highlighting
- Caching
  - Enables Codespace Prebuilds to cache the workspace artifacts
  - Applicable for reducing startup time when spawning new Codespaces
  - Necessary for limiting costs from CPU and storage usage

While the colcon workspace is being built, VS Code will simultaneously install any specified extensions and settings. Next the `updateContentCommand` is executed, which reruns whenever the container is started or restarted. Specifically, for this project, this command re-cleans and re-builds the same colcon workspace as before, but only for invalidated packages detected by colcon cache using the lockfiles initialized during the `onCreateCommand`. This caching behavior also replicates the project's CI workflow. This is useful for:

- Branching
  - Enables caching of workspace artifacts when switching between branches
  - Applicable for reviewing pull requests without rebuilding entire container
  - Necessary for reducing startup time when spawning new Codespaces

:::{hint}
More documentation about these additional colcon verb extensions can be found here:

- [colcon-cache](https://github.com/ruffsl/colcon-cache)
  - A colcon extension to cache the processing of packages
- [colcon-clean](https://github.com/colcon/colcon-clean)
  - A colcon extension to clean package workspaces
:::

Finally, the `postCreateCommand` is executed, which also reruns whenever the container is started or restarted. Specifically, for this project, this command makes a last few tweaks to the user's environment to improve the development experience.

To speed up subsequent startups, volumes that are mounted to the container store a persistent ccache and colcon workspace, while the environment is set to enable [ccache](https://ccache.dev/) via [colcon mixins](https://github.com/colcon/colcon-mixin-repository). These volumes are labeled using the [`devcontainerId`](https://containers.dev/implementors/json_reference/#variables-in-devcontainerjson) variable, which uniquely identify the dev container on a Docker host, allowing us to refer to a common identifier that is unique to the dev container, while remaining stable across rebuilds. This is useful for:

- Caching
  - Enables colcon workspaces and ccache to persist between container rebuilds
  - Applicable for avoiding re-compilation when modifying dev container config files
  - Necessary for quickly customizing image or features without rebuilding from scratch

:::{tip}
While these volumes are uniquely named, you could rename them locally to further organize or segment works-in-progress. E.g. appending branch names to the volume name to quickly switch between pull requests and cached colcon workspaces.
:::

Additionally, the container can be granted [privileged](https://docs.docker.com/engine/reference/commandline/run/#privileged) and non-default [Linux capabilities](https://docs.docker.com/engine/reference/run/#runtime-privilege-and-linux-capabilities), connected using the [host](https://docs.docker.com/network/host/) network mode and [IPC](https://docs.docker.com/engine/reference/run/#ipc-settings---ipc) and [PID](https://docs.docker.com/engine/reference/run/#pid-settings---pid) spaces, with a relaxed [security configuration](https://docs.docker.com/engine/reference/run/#security-configuration) and seccomp confinement for native debugging and external connectivity. This is useful for:

- Hybrid development
  - Enables connecting ROS nodes external to the container
  - Applicable for debugging or visualizing distributed systems
  - Necessary for DDS discovery and shared memory transport
- Device connectivity
  - Enables hardware forwarding from host machine to container
  - Applicable for ROS package using sensors and actuators
  - Necessary for some GPU drivers and USB devices

:::{attention}
Such `runArgs` in the `devcontainer.json` config can be enabled or customized, either expanded or or narrowed in scope, to better suit your desired development environment. The default configuration merely comments out these parameters, to limit unintended side effects or cross talk between containers, but can be uncommented to accommodate the widest range of development use cases.
:::

:::{seealso}
More details on using DDS, debuggers, or devices with Docker containers can be found here:

- [How to Communicate Across Docker Containers Using the Host Driver](https://community.rti.com/kb/how-use-rti-connext-dds-communicate-across-docker-containers-using-host-driver)
  - Using the `host` network driver to access all network interfaces of the host machine from the Docker container
- [Communicate between two Docker containers using DDS and shared memory](https://community.rti.com/kb/communicate-between-two-docker-containers-using-rti-connext-dds-and-shared-memory)
  - Enabling containers to communicate with one another and with the host machine using interprocess communication (IPC)
- [Debugging programs running inside Docker containers, in production](https://nvartolomei.com/debugging-programs-running-inside-docker-containers--in-production/)
  - Using tools like strace, perf, gdb when debugging programs running inside containers
:::

## Using Dev Containers

Once the dev container has been created and setup completed, VS Code will open a new workspace directly from the project's root directory, which itself is mounted within the source directory in the overlay colcon workspace. From here you can build, test, and debug the project as you normally would, with the added benefit of having the project's dependencies, intellisense, linters, and other extensions pre-configured and ready to use. Simply open a new terminal (Crtl+Shift+`), cd to the root of the colcon workspace, and run the usual colcon commands.

:::{tip}
You can incorporate the same scripts used by the `devcontainer.json` config file to further automate your local development workflow.
:::

### Terminals

If you prefer using an alternate terminal emulator, rather than the built-in VS Code terminal, you can open a separate shell session by simply using the Dev Container CLI or directly using the Docker CLI via the `exec` subcommands.

- [Dev Container CLI](https://code.visualstudio.com/docs/devcontainers/devcontainer-cli)
  - `devcontainer exec --workspace-folder $NAV2_WS/src/navigation2 bash`
- [docker exec
](https://docs.docker.com/engine/reference/commandline/exec/)
  - `docker exec -it <container-id> bash`

:::{attention}
Shell sessions spawned directly via `docker exec` do not set the same environment that `devcontainer exec` does using `userEnvProbe`. Additional environment variables include `REMOTE_CONTAINERS_IPC`, `REMOTE_CONTAINERS_SOCKETS` and are used by vscode, ssh and X11.
:::

:::{hint}
:class: dropdown
The environment provided by `userEnvProbe` can be sourced manually. E.g. for the default `loginInteractiveShell` probe:

``` bash
find /tmp -type f -path "*/devcontainers-*/env-loginInteractiveShell.json" -exec \
  jq -r 'to_entries | .[] | "\(.key)=\(.value | @sh)"' {} \; > .env
source .env
```
:::

### Lifecycle

While using the dev container, try and keep in mind the lifecycle of the container itself. Specifically, containers are ephemeral, meaning they are normally destroyed and recreated whenever the dev environment is rebuilt or updated. Subsequently, a best practice is to avoid storing any persistent data within the container, and instead utilize the project's source directory, or a separate mounted volume. When altering the development environment inside the container, try to remember to codify your changes into the Dockerfile, or the `devcontainer.json` config file, so that they can be easily reproduced and shared with others.

:::{important}
This is particularly important when the host machine is inherently ephemeral as well, as the case may be when using cloud based environments such as Codespaces, so be sure to commit and push local changes to a remote repository:

- [The codespace lifecycle](https://docs.github.com/en/codespaces/getting-started/the-codespace-lifecycle)
  - Maintain your data throughout the entire codespace lifecycle
:::

### Rebuilding

From time to time, you may need to rebuild the dev container, either because the base image, or `.devcontainer/` config was updated, or simply out of wanting a new fresh development environment. To do so, simply open the Command Palette (Ctrl+Shift+P) and select the `Remote-Containers: Rebuild Container` command.

:::{caution}
Rebuilding the container will destroy any changes made to the container itself, such as installing additional packages, or modifying the environment. However, the project's source directory, and any mounted volumes, will remain unaffected.
:::

For example, you may need to rebuild the dev container when:

- Pulling newer images from a container registry
  - specifically, image tags built `FROM` in the Dockerfile
  - or tags listed under `cacheFrom` in `devcontainer.json`
  - periodically done manually to ensure local environment reflects CI
- Updating the dev container configuration
  - specifically when modifying dependent stages in the `Dockerfile`
  - or when modifying `./devcontainer` files and commands
  - where build cache reuse correlates with severity of changes made

When necessary, you can also rebuild the container from scratch, e.i. without caching from docker, by selecting the `Remote-Containers: Rebuild Container Without Cache` command. This instead omits the `--cache-from` flag from the `docker buildx` command, while also adding the `--no-cache` and `--pull` flags to prevent caching from any existing image layers, using only the latest images from a container registry.

:::{caution}
Rebuilding the container without cache may likely pull newer images from a container registry or install newer packages, as is common when developing for ROS 2 Rolling. You may then want to clean your overlay volume to avoid ABI incompatibilities or stale artifacts.
:::

Rebuilding without caching may be necessary when:

- Needing to update the base image
  - specifically if dev container configurations remain unmodified
  - to forcefully rerun a `RUN` directive in the Dockerfile
  - such as unchanged `apt upgrade` or `rosdep update` commands

Specifically, for this project, volumes remain unaffected by this rebuilding process: i.e. those used to mount the ccache directory or colcon workspace. While volume management is left to the user's discretion, other projects may of course handle this differently, so be sure to check the `./devcontainer` configuration to inspect how various container resources may be managed.

:::{tip}
Docker volume management can be done via the Docker CLI, or the VS Code Docker extension:

* [Docker Volume CLI](https://docs.docker.com/engine/reference/commandline/volume)
  * Manage volumes using subcommands to create, inspect, list, remove, or prune volumes
* [VS Code Docker extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker)
  * Makes it easy to create, manage, and debug containerized applications
:::

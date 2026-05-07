# docs.nav2.org
https://docs.nav2.org/

This folder holds the source and configuration files used to generate the
[Navigation2 documentation](https://docs.nav2.org) web site.


### Native Installation

Dependencies for Build:

``` bash
sudo apt install python3-pip
pip3 install -r requirements.txt
```

### Installation in a Virtual Environment

Install `pip` and `venv` if not already installed:

``` bash
sudo apt install python3-pip python3-venv
```

Create a virtual environment and install the dependencies:

``` bash
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
```

### Using Docker

Build the Docker image (from this directory):

```bash
docker build -f Dockerfile --build-arg user=$(id -un) --build-arg uid=$(id -u) -t nav2_docs .
```

To build the documentation:

```bash
docker run --rm -v $(pwd):/docs nav2_docs make html
```

To run autobuild (watches for changes and rebuilds automatically):

```bash
docker run --init --rm -it -v $(pwd):/docs -p 8000:8000 nav2_docs make autobuild
```

Then browse to http://127.0.0.1:8000. Use Ctrl+C to stop (the `--init` flag enables proper signal handling).

### Using VS Code Dev Container

If you're using Visual Studio Code, you can use the dev container for an easy setup:

1. Install the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
2. Open this folder in VS Code
3. When prompted, click "Reopen in Container" (or use Command Palette: "Dev Containers: Reopen in Container")
4. Once the container is built and running, use the integrated terminal to build the docs

The dev container automatically sets up all dependencies. You can then use the build tasks or run commands directly:
- Build once: `make html` or use the "Build" task (Ctrl+Shift+B)
- Auto-rebuild: `make autobuild` or use the "Autobuild" task

### Build the Docs

Build the docs locally with `make html` and you'll find the built docs entry point in `_build/html/index.html`.

To automate the build process, you can use a [sphinx-autobuild](https://github.com/sphinx-doc/sphinx-autobuild) package. \
Run this command from the virtual environment to build the documentation and start a server:

```bash
sphinx-autobuild . ./_build/html
```
For more options for the command, see the documentation linked above.

Now you can access the page using the local address: http://127.0.0.1:8000. \
After saving any changes, the documentation will be automatically rebuilt and displayed.

Any images, diagrams, or videos are subject to their own copyrights, trademarks, and licenses.

Want a local PDF version? Follow the [instructions here](https://gist.github.com/alfredodeza/7fb5c667addb1c6963b9).

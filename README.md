# mk.docs.nav2.org

[![CircleCI](https://img.shields.io/badge/-circleci-1b273a?style=for-the-badge&logo=circleci&logoColor=white)](https://app.circleci.com/pipelines/github/ros-navigation/mk.docs.nav2.org)

<!-- Placeholder for the link / logo -->

## Installation

### Virtual Environment

Install `pip` and `venv` if not already installed:
``` shell
sudo apt install python3-pip python3-venv
```

Create a virtual environment and activate it:
```shell
python3 -m venv venv &&
source venv/bin/activate
```

Install all required dependencies:
```shell
pip3 install -r requirements.txt
```

## Build

### Single distribution

Use the following command from the required distribution branch to build the documentation:
```shell
mkdocs build
```

The build result can be found in the `site` directory, and the entry point in `site/index.html` file.
[Other options][mkdocs-build-url] for the build command.

Instead of the usual build process, MkDocs provides a live preview server that can be started with:
```shell
mkdocs serve
```

It allows to preview new changes during documentation update. The server will automatically rebuild the entire documentation after each file saving and display result at http://127.0.0.1:8000/. See [more options][mkdocs-serve-url] for the command.

> [!WARNING]
> The command `mkdocs serve` will display documentation for only one distribution without the ability to switch between them. \
> To preview multiple distributions, follow the instructions below.


### Multiple distributions

The current documentation relies on the [mike][mike-url] utility to support multiple versions.
To build the documentation, execute the following command from the corresponding `<distribution>` branch for each version that needs to be displayed:
```shell
mike deploy <distribution>
```

After execution, a new directory with the selected `<distribution>` name will be created on the local `gh-pages` branch. This directory will also contain the build result with the corresponding entry point `<distribution>/index.html`.
In addition, it creates the same `site` directory as before using `mkdocs build` in the current branch.

You can set the `--title` option to change the version name displayed on the website. See [more options][mike-build-url] for this command.

Example for two branches:
- Execute from `rolling` branch:
  ```shell
  mike deploy rolling --title=Rolling
  ```

- Execute from `jazzy` branch:
  ```shell
  mike deploy jazzy --title=Jazzy
  ```

Before viewing, you need to set the default version:
```shell 
mike set-default <distribution>
``` 

Similar to `mkdocs serve`, mike provides a server that can be started with:
```shell
mike serve
```

The documentation will be available at the same address http://127.0.0.1:8000/, and each version at the corresponding address `http://127.0.0.1:8000/<distribution>/`. \
See [more options][mike-serve-url] for this command.

> [!NOTE]
> This server does not provide a live preview after changes are made locally. \
> To display the new changes, use the `mike deploy` command for the corresponding branch, as was shown before. 

This command is useful for local testing and viewing differences between versions without the need to use a real web server.
If you need to make changes for only one version, it will be more convenient to use `mkdocs serve` for this purpose, due to its live preview ability.

After completing all changes, use this command to exit the virtual environment (venv):
```shell
deactivate
```

## License

This project is licensed under the terms of the [Apache-2.0](./LICENSE) license. \
Any images, diagrams, or videos are subject to their own copyrights, trademarks, and licenses.


[mkdocs-build-url]: https://www.mkdocs.org/user-guide/cli/#mkdocs-build
[mkdocs-serve-url]: https://www.mkdocs.org/user-guide/cli/#mkdocs-serve
[mike-url]: https://github.com/jimporter/mike
[mike-build-url]: https://github.com/jimporter/mike#building-your-docs
[mike-serve-url]: https://github.com/jimporter/mike#viewing-your-docs

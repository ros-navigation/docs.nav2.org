# Description

The `macros` directory is intended for the [mkdocs-macros-plugin](https://github.com/fralau/mkdocs-macros-plugin), which enables reusable macros and variables throughout the documentation. See the [Mkdocs-Macros Documentation](https://mkdocs-macros-plugin.readthedocs.io) for all available features.

The Nav2 documentation uses macros to automatically generate documentation for BT Node ports and their XML examples.
Macro definitions are located in `macros/main.py`, while auxiliary variables are provided in `macros/variables.yml` for easier configuration. Variables can be used both in the documentation itself (see `nav2_bt_hpp_dir_path` in the example below) and within the macros.

Currently available macros:

- `render_bt_node_ports(bt_node_id: str)`

    Generates MkDocs-formatted documentation for a Behavior Tree node ports. The `bt_node_id` argument must correspond to the defined BT node ID in the Nav2 project.

- `render_bt_node_example(file_path: Path, class_name: str | None = None)`

    Generates MkDocs-formatted Behavior Tree node XML example. The `file_path` parameter points to the `.hpp` file containing the XML example (defined in Doxygen comments). The optional `class_name` parameter specifies which class example to render when a file contains multiple node classes (e.g. `clear_costmap_service.hpp`).

Example of usage:

```
# Spin { #spin }

{{ render_bt_node_ports(page.title) }}

## Example

{% set bt_hpp_file_path = nav2_bt_hpp_dir_path + "/action/spin_action.hpp" %}

{{ render_bt_node_example(bt_hpp_file_path) }}
```

Documentation generation process:

1. During build, the plugin reads `macros/variables.yml` to obtain paths to necessary files and directories: the cache directory, the Behavior Tree XML file, BT node header file directories, and GitHub repositories to clone for source code access.

2. Specified files from the GitHub repositories are cloned into the cache directory (`macros/cache`) if they are not present or outdated.

3. Pages using the macros invoke the appropriate macro functions from `macros/main.py`, which read files from the cache directory and generate the formatted documentation.

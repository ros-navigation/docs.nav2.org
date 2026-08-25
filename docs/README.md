# Documentation maintenance

The `/docs` directory contains main documentation files for the Nav2 project. Below is information useful for maintaining the documentation.

## Page synchronization

To minimize maintenance efforts, some information is automatically synchronized across different distribution branches. The following files and directories are cloned from the `rolling` branch and symlinked to provide a single source for shared documentation:

- `index.md`
- `assets`
- `community`
- `robots_using`
- `about_and_contact`

To update these synchronized items, edit only the `rolling` branch. Changes from there will be propagated to all distribution versions.

## Page autogeneration

The documentation uses the [macros](https://github.com/fralau/mkdocs-macros-plugin) plugin to automatically generate documentation for Behavior Tree (BT) Node ports and their XML examples.

To add or update XML examples, modify them in the Doxygen comments of the hpp files located in `navigation2/nav2_behavior_tree/include/nav2_behavior_tree/plugins/` using the following format:
```
/**
 * ...
 * Usage in XML:
 * @code
 * ...
 * @endcode
 */
```

To update the nodes and ports information, modify the `navigation2/nav2_behavior_tree/nav2_tree_nodes.xml` file in the corresponding branch of the Nav2 repository. Note that the information in this file must match what is specified in the hpp files. Only the port descriptions may differ.

Once BT Node ports or XML examples are updated, the macros plugin automatically fetches, parses, and inserts them into the documentation during the build process. This ensures the documentation stays synchronized with the source code.

See [macros/README.md](../macros/README.md) for further information.

## Cross-page references

The documentation uses the [autorefs](https://github.com/mkdocstrings/autorefs) plugin to make it easier and faster to refer to other internal pages.

To create a referenceable heading, add an anchor ID (permalink) to an h1 heading like this:
```
# Robots Using { #robots-using }
```

Once you've created an anchor, you can reference it anywhere in the documentation using the following syntax:
```
For a list of robots using Nav2, see [Robots Using][robots-using].
```

The current documentation explicitly specifies permalinks only:
- For h1 headings, to make them easily searchable and referenceable across the project
- For headings with duplicate names, for example:
    ```
    # DWB Controller { #dwb-controller-index }
    # DWB Controller { #dwb-controller }
    ```

For all other headings, mkdocs automatically generates permalinks, which can then also be used as references. This enables maintainable cross-references that work regardless of file renaming and relocation.
Refer to the plugin [documentation](https://mkdocstrings.github.io/autorefs/) for further information.

## Assets organization

Any graphic material (images, videos, and GIFs) and other supplementary material are located in separate `assets` directories as close as possible to the pages where they are used.

For example:
```shell
/docs
├── index.md
├── assets
│   ...
└── robots_using
    ├── assets
    └── index.md
```

This approach reduces path lengths in documentation and simplifies structural modifications without requiring path updates.

# Docs Distribution Release Process { #docs-distribution-release-process }

This page outlines the main steps to add a new documentation version following a Nav2 distribution release.

## 1. Branch Off Docs Distribution

Create new distribution branch from `rolling`:

```shell
git checkout -b <distro> rolling
```

## 2. Update Configuration files

### 2.1 CircleCI

Add new distribution branch to `filters` in `.circleci/config.yml`:

```yaml
workflows:
  build_docs:
    jobs:
      - docs_build:
          filters:
            branches:
              ignore:
                - rolling
                - <distro>
  publish_docs:
    jobs:
      - docs_publish:
          filters:
            branches:
              only:
                - rolling
                - <distro>
```

### 2.2 Github Actions

Add new branch in configuration files located in `.github/workflows`:

- Update conditions for `pre-commit`:

    ```yaml
    on:
      pull_request:
      push:
        branches:
          - rolling
          - <distro>
    ```

### 2.3 MkDocs Material

Update link for `edit_uri` in `mkdocs.yml` configuration file:

```yaml
edit_uri: https://github.com/ros-navigation/docs.nav2.org/blob/<distro>/docs/
```

Update `branch` variable in `macros/variables.yml`:

```yaml
github_repositories:
  navigation2:
    owner: "ros-navigation"
    branch: "<distro>"
```

## 3. Update Documentation

- Update all github links to point to new distribution branch where it applies.
- Update all links referring to ROS 2 Documentation.

    !!! warning Important

        The ROS 2 Rolling documentation has a different structure than released distributions. Update links with the correct path, not just the branch name.

## 4. Build and Publish Documentation

Once all the changes are made, use the command below to check the build:

```shell
sudo apt install python3-pip python3-venv
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
mkdocs build
```

Refer to README.md for additional commands, such as previewing multiple versions locally before publishing.

Publish the documentation to the new distribution branch:

```shell
git push origin <distro>
```

---
edit_uri: https://github.com/ros-navigation/mkdocs.nav2.org/tree/rolling/docs/
---

# Docs Distribution Release Process { #docs-distribution-release-process }

This page outlines the main steps to add a new documentation version following a Nav2 distribution release.

## 1. Branch Off Docs Distribution

Create new distribution branch from `rolling` and switch to it:

```shell
git checkout -b <distro> rolling
```

All following actions and commands execute in the new branch only. Replace `<distro>` with the actual distribution name (e.g. `lyrical`).

## 2. Update Configuration files

### 2.1 CircleCI

Replace the `rolling` branch with a new one for workflow `filters` in `.circleci/config.yml`:

```yaml
workflows:
  build_docs:
    jobs:
      - docs_build:
          filters:
            branches:
              ignore:
                - <distro>
  publish_docs:
    jobs:
      - docs_publish:
          filters:
            branches:
              only:
                - <distro>
```

### 2.2 Github Actions

Update branch name in all configuration files located in `.github/workflows`:

- Update condition for `pre-commit`:

    ```yaml
    on:
      pull_request:
      push:
        branches:
          - <distro>
    ```

### 2.3 MkDocs Material

Update link for `edit_uri` key in `mkdocs.yml` configuration file:

```yaml
edit_uri: https://github.com/ros-navigation/docs.nav2.org/blob/<distro>/docs/
```

Update `ros2_distro` variable in `mkdocs.yml`:

```yaml
extra:
  ros2_distro: "<distro>"
```

Update `branch` variable in `macros/variables.yml`:

```yaml
github_repositories:
  navigation2:
    owner: "ros-navigation"
    branch: "<distro>"
```

## 3. Update Documentation

### 3.1 Update links

- Update all GitHub links to point to new distribution branch where it applies.
- Update all links referring to ROS 2 Documentation.

    !!! warning "Important"

        The ROS 2 documentation for the Rolling version has a different structure than other released distributions. Each link must be checked to ensure the correct path to the ROS 2 documentation page.

    Here is an example showing the difference on one of the pages:

    **Rolling**: [https://docs.ros.org/en/rolling/ROS-Framework/interfaces/actions/Working-with-actions/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html](https://docs.ros.org/en/rolling/ROS-Framework/interfaces/actions/Working-with-actions/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

    **Lyrical**: [https://docs.ros.org/en/lyrical/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html](https://docs.ros.org/en/lyrical/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

### 3.2 Review tutorials

Review [tutorials][tutorials] for compatibility with the new distribution, including API and behavior changes.

## 4. Build and Publish Documentation

Once all the changes are made, use the command below to check the build:

```shell
sudo apt install python3-pip python3-venv
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
mkdocs build
```

Refer to [README.md](https://github.com/ros-navigation/docs.nav2.org/blob/master/README.md) for additional commands, such as previewing multiple versions locally before publishing.

Publish the documentation to the new distribution branch:

```shell
git push origin <distro>
```

## 5. Mark Branch as Protected

Go to the Repo Settings -> Branches. Create a branch protection rule for the new branch that matches the last.

- Request a PR before merging -> Require approvals & override for infra-admins.
- Restrict who can push branches that match this rule.

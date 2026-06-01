from __future__ import annotations  # Required for Python 3.8 support in CI (ubuntu:focal)

import subprocess
import logging
import re
import sys
from shutil import rmtree
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, TypedDict

from jinja2 import Template


logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s    -  [%(asctime)s][%(filename)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


_TYPE_DIRECT_MAPPINGS = {
    'unsigned short': 'uint16',
}


_TYPE_REGEX_TRANSFORMS = [
    (re.compile(r'^std::'), ''),
    (re.compile(r'_<std::allocator<void>\s*>\s*'), ''),
    (re.compile(r',\s*std::allocator<(?:[^<>]|<[^>]*>)*>\s*'), ''),
]


_DOXYGEN_REGEX_PATTERNS = {
    'COMMENT_STYLE': re.compile(r'^[ \t]*\*[ \t]?'),
    'COMMENT_END': re.compile(r'^\s*\*/'),
    'XML_USAGE_EXAMPLE': re.compile(r'Usage in XML:'),
    'CODE_BLOCK_START': re.compile(r'@code\b'),
    'CODE_BLOCK_END': re.compile(r'@endcode\b'),
}


# MkDocs-formatted template for a single port
_PORT_TEMPLATE = Template("""
### **`{{ port_name }}`**

| Type                        | Default                 |
|-----------------------------|-------------------------|
| `{{ port_data.data_type }}` | {{ port_data.default }} |

Description
:   {{ port_data.description }}
""")


_XML_CODE_BLOCK_TEMPLATE = Template("""\
```xml
{{ xml_code }}
```
""")


TREE_NODES_MODEL_TAG = 'TreeNodesModel'


class PortData(TypedDict):
    data_type: str
    default: str
    description: str

type NodePorts = dict[str, PortData]  # {port_name: PortData}


def _convert_type(type_name: str) -> str:
    """
    Convert C++ types to simplified names for documentation.

    For example:
    `std::string` -> `string`,
    `std::vector<int, std::allocator<int> >` -> `vector<int>`,
    `nav2_msgs::msg::Route_<std::allocator<void> >` -> `nav2_msgs::msg::Route`.
    """
    if type_name in _TYPE_DIRECT_MAPPINGS:
        return _TYPE_DIRECT_MAPPINGS[type_name]

    result = type_name
    for pattern, replacement in _TYPE_REGEX_TRANSFORMS:
        result = pattern.sub(replacement, result)
    return result


def _extract_ports(node: ET.Element) -> tuple[NodePorts, NodePorts, NodePorts]:
    """
    Extract input, output and bidirectional ports from given XML BT node.

    Returns tuple (input_ports, output_ports, bidirectional_ports).
    """
    input_ports: NodePorts = {}
    output_ports: NodePorts = {}
    bidirectional_ports: NodePorts = {}

    for child in node:

        port_direction = child.tag
        if port_direction not in ('input_port', 'output_port', 'bidirectional_port'):
            continue

        port_name = child.attrib.get('name')
        if not port_name:
            raise ValueError(f'Each port must have a "name" attribute.')

        raw_port_type = child.attrib.get('type')
        if not raw_port_type:
            raise ValueError(f'Port {port_name}: missing "type" attribute.')
        port_type = _convert_type(raw_port_type)

        port_default_value = child.attrib.get('default', 'N/A')
        if port_default_value in ('double', 'float'):
            # Simplify numeric default values by stripping trailing zeros
            port_default_value = str(float(port_default_value))

        port_description = (child.text or '').strip()
        if not port_description:
            raise ValueError(f'Port {port_name}: missing description.')

        port_data: PortData = {
            'data_type': port_type,
            'default': port_default_value,
            'description': port_description,
        }

        match port_direction:
            case 'input_port':
                input_ports[port_name] = port_data
            case 'output_port':
                output_ports[port_name] = port_data
            case 'bidirectional_port':
                bidirectional_ports[port_name] = port_data

    return (input_ports, output_ports, bidirectional_ports)


def _get_git_branch_name(local_repo_path: Path) -> str | None:
    """Get current Git branch name in the working directory."""
    try:
        branch_name = subprocess.run(
            ['git', 'branch', '--show-current'],
            cwd=local_repo_path,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        return branch_name
    except subprocess.CalledProcessError as exc:
        logger.error(f'Failed to get Git branch name: {exc}')
        return None


def _is_git_workdir_synced(local_repo_path: Path) -> bool:
    """
    Check if the local working directory is synced with the GitHub remote repository.

    Returns True if up to date, False if update is needed.
    """
    try:
        local_commit_hash = subprocess.run(
            ['git', 'rev-parse', 'HEAD'],
            cwd=local_repo_path,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()

        branch_name = _get_git_branch_name(local_repo_path)
        remote_commit_hash = subprocess.run(
            ['git', 'ls-remote', 'origin', f'refs/heads/{branch_name}'],
            cwd=local_repo_path,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip().split()[0]

        return local_commit_hash == remote_commit_hash

    except subprocess.CalledProcessError as exc:
        logger.error(f'Failed to check Git working directory status: {exc}')
        return False


def _clone_sparse_github_data(
    repo_name: str,
    owner: str,
    branch: str,
    data_to_clone: list[str],
    clone_dir: Path
) -> None:
    """Clone GitHub repository sparsely and checkout the specified files/directories."""
    if not data_to_clone:
        raise ValueError('No directories or files specified for sparse checkout.')

    if not clone_dir.exists():
        raise ValueError(f'Clone directory does not exist: {clone_dir}')

    repo_workdir = clone_dir / repo_name
    if repo_workdir.exists():
        rmtree(repo_workdir)

    github_url = f'https://github.com/{owner}/{repo_name}.git'
    logger.info(f'Cloning from {github_url} (branch: {branch})')
    try:
        subprocess.run([
            'git', 'clone',
            '--depth=1',
            '--filter=blob:none',
            '--sparse',
            '--branch', branch,
            github_url,
            repo_workdir,
        ], check=True, text=True)

        logger.info(f'Performing sparse checkout in {repo_workdir} directory...')
        subprocess.run([
            'git',
            'sparse-checkout',
            'set',
            '--no-cone',
            *data_to_clone,
        ], cwd=repo_workdir, check=True, text=True)
    except subprocess.CalledProcessError:
        rmtree(repo_workdir, ignore_errors=True)
        raise

    logger.info(
        f'Cloned the following data from {github_url} (branch: {branch}) to {repo_workdir}:'
    )
    for path in data_to_clone:
        logger.info(f'\t - {path}')


def _clone_github_repository(
    repo_name: str,
    owner: str,
    branch: str,
    clone_dir: Path
) -> None:
    """Clone GitHub repository."""
    if not clone_dir.exists():
        raise ValueError(f'Clone directory does not exist: {clone_dir}')

    repo_workdir = clone_dir / repo_name
    if repo_workdir.exists():
        rmtree(repo_workdir)

    github_url = f'https://github.com/{owner}/{repo_name}.git'
    logger.info(f'Cloning repository {github_url} (branch: {branch})')
    try:
        subprocess.run([
            'git', 'clone',
            '--branch', branch,
            github_url,
            repo_workdir,
        ], check=True, text=True)
    except subprocess.CalledProcessError:
        rmtree(repo_workdir, ignore_errors=True)
        raise

    logger.info(
        f'Cloned the repository {github_url} (branch: {branch}) to {repo_workdir}.'
    )


def _extract_bt_nodes_model(content: ET.ElementTree[Any]) -> ET.Element:
    """Load the Behavior Tree nodes model from the given XML element tree."""
    root = content.getroot()
    if root is None or len(root) == 0:
        raise ValueError(
            'Invalid XML structure: '
            f'Expected <{TREE_NODES_MODEL_TAG}> element as the first child of the root.'
        )
    bt_nodes_model = root[0]

    if bt_nodes_model.tag != TREE_NODES_MODEL_TAG:
        raise ValueError(
            'Invalid XML structure: '
            f'Expected <{TREE_NODES_MODEL_TAG}> element as the first child of the root.'
        )
    return bt_nodes_model


def _get_lines_section(
    lines: list[str],
    start: re.Pattern,
    end: re.Pattern,
    stop_at: re.Pattern
) -> list[str]:
    """
    Extract a section of lines.

    Search between start and end patterns, with a condition to stop searching.
    """
    start_idx = None
    end_idx = None

    # Note: Intentionally updates start_idx/end_idx on every match to capture
    # the MOST RECENT occurrence before stop_at.

    for index, line in enumerate(lines):
        if start.search(line):
            start_idx = index

        if end.search(line):
            end_idx = index

        if stop_at.search(line):
            break

    if start_idx is None:
        raise ValueError(f'Pattern "{start.pattern}" not found.')

    if end_idx is None:
        raise ValueError(f'Pattern "{end.pattern}" not found.')

    if end_idx < start_idx:
        raise ValueError(
            f'End pattern "{end.pattern}" found before start pattern "{start.pattern}".'
        )

    return lines[start_idx:end_idx]


def _extract_doxygen_code_block(
    lines: list[str],
    code_start_pattern: re.Pattern,
    code_end_pattern: re.Pattern,
    comment_block_pattern: re.Pattern
) -> list[str]:
    """Extract code block from Doxygen with trailing comments removing."""
    code_lines: list[str] = []
    in_code_block = False
    code_start_found = False
    code_end_found = False

    for line in lines:
        if code_start_pattern.search(line):
            in_code_block = True
            code_start_found = True
            continue
        elif code_end_pattern.search(line):
            if not code_start_found:
                raise ValueError(
                    f'Code end pattern "{code_end_pattern.pattern}" '
                    f'found before code start pattern "{code_start_pattern.pattern}".'
                )
            in_code_block = False
            code_end_found = True
            continue

        if in_code_block:
            line = re.sub(comment_block_pattern, '', line)
            code_lines.append(line)

    if not code_start_found:
        raise ValueError(f'Code start pattern "{code_start_pattern.pattern}" not found.')

    if not code_end_found:
        raise ValueError(f'Code end pattern "{code_end_pattern.pattern}" not found.')

    if not code_lines:
        raise ValueError(
            f'No code block found between "{code_start_pattern.pattern}" '
            f'and "{code_end_pattern.pattern}" patterns. '
            'Review Doxygen comment formatting in the hpp file.'
        )

    code_lines[-1] = code_lines[-1].rstrip('\n')
    return code_lines


def define_env(env):
    """This is the hook for the variables, macros and filters."""
    cache_dir = Path(env.variables['cache_dir'])
    nav2_tree_nodes_file_path = Path(env.variables['nav2_tree_nodes_file_path'])
    github_repos = env.variables['github_repositories']
    cached_bt_nodes_file_path = cache_dir / nav2_tree_nodes_file_path

    for repo_name, repo_info in github_repos.items():

        local_repo_path = cache_dir / Path(repo_name)
        if local_repo_path.exists() \
                and repo_info['branch'] == _get_git_branch_name(local_repo_path) \
                and _is_git_workdir_synced(local_repo_path):
            logger.info(
                f'Cached Git repository "{local_repo_path}" '
                f'is synced with remote GitHub repository. '
                f'Skipping clone.'
            )
            continue
        try:
            _clone_sparse_github_data(
                repo_name=repo_name,
                owner=repo_info['owner'],
                branch=repo_info['branch'],
                data_to_clone=repo_info['data_to_clone'],
                clone_dir=cache_dir
            )
        except subprocess.CalledProcessError as exc:
            stderr = getattr(exc, 'stderr', None)
            logger.error(f'Failed to clone GitHub data: {stderr or exc}')
            logger.info('Attempting complete clone as fallback...')
            try:
                _clone_github_repository(
                    repo_name=repo_name,
                    owner=repo_info['owner'],
                    branch=repo_info['branch'],
                    clone_dir=cache_dir
                )
            except (subprocess.CalledProcessError, ValueError, OSError) as exc:
                stderr = getattr(exc, 'stderr', None)
                logger.error(f'Failed to clone GitHub repository: {stderr or exc}')
                sys.exit(1)
        except (ValueError, OSError) as exc:
            logger.error(f'Failed to clone GitHub data: {exc}')
            sys.exit(1)

    try:
        bt_nodes_content = ET.parse(cached_bt_nodes_file_path)
    except (OSError, ET.ParseError) as exc:
        logger.error(
            f'Failed to load BT nodes data from {cached_bt_nodes_file_path}: {exc}\n'
            'Review paths in macros/variables.yml configuration.'
        )
        sys.exit(1)

    try:
        bt_nodes_model = _extract_bt_nodes_model(bt_nodes_content)
    except (ValueError, IndexError) as exc:
        logger.error(f'Failed to extract BT nodes model from XML: {exc}')
        sys.exit(1)

    @env.macro
    def render_bt_node_ports(bt_node_id: str) -> str:
        """
        Render MkDocs-formatted documentation for a Behavior Tree node's ports.

        Returns a string with sections for input, output and bidirectional ports.
        """
        node = bt_nodes_model.find(f'.//*[@ID="{bt_node_id}"]')
        if node is None:
            logger.error(f'BT node ID not found: {bt_node_id}.')
            sys.exit(1)

        try:
            input_ports, output_ports, bidirectional_ports = _extract_ports(node)
        except ValueError as exc:
            logger.error(f'Failed to extract ports for BT node {bt_node_id}: {exc}')
            sys.exit(1)

        rendered_ports = []
        if input_ports:
            rendered_ports.append('## Input Ports\n')
            for port_name, port_data in input_ports.items():
                rendered_port = _PORT_TEMPLATE.render(
                    port_name=port_name, 
                    port_data=port_data
                )
                rendered_ports.append(rendered_port)

        if output_ports:
            rendered_ports.append('## Output Ports\n')
            for port_name, port_data in output_ports.items():
                rendered_port = _PORT_TEMPLATE.render(
                    port_name=port_name, 
                    port_data=port_data
                )
                rendered_ports.append(rendered_port)

        if bidirectional_ports:
            rendered_ports.append('## Bidirectional Ports\n')
            for port_name, port_data in bidirectional_ports.items():
                rendered_port = _PORT_TEMPLATE.render(
                    port_name=port_name, 
                    port_data=port_data
                )
                rendered_ports.append(rendered_port)

        return '\n\n'.join(rendered_ports)

    @env.macro
    def render_bt_node_example(file_path: Path, class_name: str | None = None) -> str:
        """
        Render MkDocs-formatted documentation for a Behavior Tree node's example usage in XML.

        Returns a string with the XML code example from a Doxygen comment in a HPP file.
        """
        cached_bt_hpp_file_path = cache_dir / file_path

        try:
            with open(cached_bt_hpp_file_path, encoding='utf-8') as file:
                file_lines = file.readlines()
        except OSError as exc:
            logger.error(f'Failed to read lines from file {cached_bt_hpp_file_path}: {exc}')
            sys.exit(1)

        class_str = r'^\s*class\s+'
        if class_name:
            class_str += rf'{re.escape(class_name)}\b'
        class_pattern = re.compile(class_str)

        try:
            doxygen_section = _get_lines_section(
                lines=file_lines,
                start=_DOXYGEN_REGEX_PATTERNS['XML_USAGE_EXAMPLE'],
                end=_DOXYGEN_REGEX_PATTERNS['COMMENT_END'],
                stop_at=class_pattern
            )
        except ValueError as exc:
            logger.error(f'Failed to extract lines section from file {cached_bt_hpp_file_path}: {exc}')
            sys.exit(1)

        try:
            code_section = _extract_doxygen_code_block(
                lines=doxygen_section,
                code_start_pattern=_DOXYGEN_REGEX_PATTERNS['CODE_BLOCK_START'],
                code_end_pattern=_DOXYGEN_REGEX_PATTERNS['CODE_BLOCK_END'],
                comment_block_pattern=_DOXYGEN_REGEX_PATTERNS['COMMENT_STYLE']
            )
        except ValueError as exc:
            logger.error(f'Failed to extract code block from file {cached_bt_hpp_file_path}: {exc}')
            sys.exit(1)

        code_example = ''.join(code_section)
        code_example_md = _XML_CODE_BLOCK_TEMPLATE.render(xml_code=code_example)

        return code_example_md

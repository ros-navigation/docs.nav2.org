from __future__ import annotations # Required for Python 3.8 support in CI (ubuntu:focal)
import subprocess, logging, re, sys, os, shutil
import xml.etree.ElementTree as ET
from typing import Pattern 
from pathlib import Path
from jinja2 import Template


logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s - [%(asctime)s][%(filename)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


_TYPE_DIRECT_MAPPINGS = {
    "unsigned short": "uint16",
}


_TYPE_REGEX_TRANSFORMS = [
    (re.compile(r'^std::'), ''),
    (re.compile(r'_<std::allocator<void>\s*>\s*'), ''),
    (re.compile(r',\s*std::allocator<(?:[^<>]|<[^>]*>)*>\s*'), ''),
]


_DOXYGEN_REGEX_PATTERNS = {
    "COMMENT_STYLE": re.compile(r"^[ \t]*\*[ \t]?"),
    "COMMENT_END": re.compile(r"^\s*\*/"),
    "XML_USAGE_EXAMPLE": re.compile(r"Usage in XML:"),
    "CODE_BLOCK_START": re.compile(r"@code\b"),
    "CODE_BLOCK_END": re.compile(r"@endcode\b"),
}


_PORT_SECTION_TEMPLATE = Template("""\
## {{ heading }}

{% for port in ports -%}
### **`{{ port.name }}`**

| Type              | Default            |
|-------------------|--------------------|
| `{{ port.type }}` | {{ port.default }} |

{% if port.description %}
Description
:   {{ port.description }}
{% endif %}
{% endfor -%}
""")


_XML_CODE_BLOCK_TEMPLATE = Template("""\
```xml
{{ xml_code }}
```
""")


def _convert_type(type_name: str) -> str:
    """
    Convert C++ types to simplified names for documentation, e.g.
    `std::string` -> `string`,
    `std::vector<int, std::allocator<int> >` -> `vector<int>`,
    `nav2_msgs::msg::Route_<std::allocator<void> >` -> `nav2_msgs::msg::Route`,
    """

    if type_name in _TYPE_DIRECT_MAPPINGS:
        return _TYPE_DIRECT_MAPPINGS[type_name]

    result = type_name
    for pattern, replacement in _TYPE_REGEX_TRANSFORMS:
        result = pattern.sub(replacement, result)
    return result


def _format_default(default_str: str) -> str:
    """
    Simplify numeric default values by stripping trailing zeros, e.g.
    `0.150000` -> `0.15`, `1.000000` -> `1.0`
    """
    try:
        return str(float(default_str))
    except (ValueError, TypeError):
        return default_str


def _parse_ports(
        node: ET.Element
    ) -> tuple[list[dict[str, str]], list[dict[str, str]], list[dict[str, str]]]:
    """
    Parse input_port, output_port and bidirectional_port children from XML BT node.
    Returns (input_ports, output_ports, bidirectional_ports).
    """

    input_ports: list[dict[str, str]] = []
    output_ports: list[dict[str, str]] = []
    bidirectional_ports: list[dict[str, str]] = []

    for child in node:
        
        port_direction = child.tag
        if port_direction not in ('input_port', 'output_port', 'bidirectional_port'):
            continue

        port_name = child.attrib.get('name', '')
        
        raw_port_type = child.attrib.get('type', '')
        port_type = _convert_type(raw_port_type)

        raw_port_default_value = child.attrib.get('default', 'N/A')
        port_default_value = (
            _format_default(raw_port_default_value)
            if port_type in ('double', 'float')
            else raw_port_default_value
        )

        port_description = (child.text or '').strip()

        port = {
            'name': port_name,
            'type': port_type,
            'default': port_default_value,
            'description': port_description,
        }

        if port_direction == 'input_port':
            input_ports.append(port)
        elif port_direction == 'output_port':
            output_ports.append(port)
        elif port_direction == 'bidirectional_port':
            bidirectional_ports.append(port)

    return input_ports, output_ports, bidirectional_ports


def _remove_dir(dir_path: Path):
    """Remove directory if it exists"""
    if os.path.exists(dir_path):
        try:
            shutil.rmtree(dir_path)
            logger.info(f"Removed existing directory: {dir_path}")
        except OSError as exc:
            logger.error(f"Failed to remove existing directory {dir_path}: {exc}")
            raise


def _get_git_branch_name(local_repo_path: Path) -> str | None:
    """
    Get current Git branch name in the working directory.
    """

    try:
        branch_name = subprocess.run(
            ["git", "branch", "--show-current"],
            cwd=local_repo_path,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        return branch_name
    except subprocess.CalledProcessError as exc:
        logger.error(f"Failed to get Git branch name: {exc}")
        return None


def _is_git_workdir_synced(local_repo_path: Path) -> bool:
    """
    Check if the local working directory is synced 
    with the GitHub remote repository.
    Returns True if up to date, False if update is needed.
    """

    try:
        local_commit_hash = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=local_repo_path,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()

        branch_name = _get_git_branch_name(local_repo_path)
        remote_commit_hash = subprocess.run(
            ["git", "ls-remote", "origin", f"refs/heads/{branch_name}"],
            cwd=local_repo_path,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip().split()[0]

        return local_commit_hash == remote_commit_hash

    except subprocess.CalledProcessError as exc:
        logger.error(f"Failed to check Git working directory status: {exc}")
        return False


def _clone_sparse_github_data(
        repo_name: str, 
        owner: str, 
        branch: str, 
        data_to_clone: list[str], 
        clone_dir: Path
    ) -> None:
    """
    Clone GitHub repository sparsely and 
    checkout the specified files/directories.
    """

    if not data_to_clone:
        logger.error("No directories or files specified for sparse checkout.")
        raise ValueError("data_to_clone cannot be empty.")
    
    if not clone_dir.exists():
        logger.error(f"Clone directory does not exist: {clone_dir}")
        raise ValueError(f"clone_dir must exist: {clone_dir}")
    
    repo_work_dir = clone_dir / repo_name
    _remove_dir(repo_work_dir)

    github_url = f"https://github.com/{owner}/{repo_name}.git"
    logger.info(f"Cloning from {github_url} (branch: {branch})")
    try:
        subprocess.run([
            "git", "clone",
            "--depth=1",
            "--filter=blob:none",
            "--sparse",
            "--branch", branch,
            github_url,
            str(repo_work_dir),
        ], check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as exc:
        raise
    
    logger.info(f"Performing sparse checkout in {repo_work_dir}")
    try:
        subprocess.run([
            "git", 
            "sparse-checkout", 
            "set",
            "--no-cone",
            *data_to_clone,
        ], cwd=repo_work_dir, check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError as exc:
        _remove_dir(repo_work_dir)
        raise

    logger.info(
        f"Cloned the following data from {github_url} (branch: {branch}) to {repo_work_dir}:\n\t" +
        "\n\t".join(data_to_clone)
    )


def _clone_github_repository(
        repo_name: str, 
        owner: str, 
        branch: str, 
        clone_dir: Path
    ) -> None:
    """Clone GitHub repository."""

    if not clone_dir.exists():
        raise ValueError(f"Clone directory does not exist: {clone_dir}")

    repo_work_dir = clone_dir / repo_name
    _remove_dir(repo_work_dir)

    github_url = f"https://github.com/{owner}/{repo_name}.git"
    logger.info(f"Cloning repository {github_url} (branch: {branch})")
    try:
        subprocess.run([
            "git", "clone",
            "--branch", branch,
            github_url,
            str(repo_work_dir),
        ], check=True, capture_output=True, text=True)
    except subprocess.CalledProcessError:
        _remove_dir(repo_work_dir)
        raise

    logger.info(
        f"Cloned the repository {github_url} (branch: {branch}) to {repo_work_dir}."
    )


def _load_bt_nodes_model(file_path: Path ) -> ET.Element:
    """
    Load the behavior-tree nodes model from a cached file.
    """

    if not file_path.exists():
        raise ValueError("File path does not exist.")

    try:
        root = ET.parse(file_path).getroot()
        return root[0]
    except (ET.ParseError, IndexError) as exc:
        logger.error(f"Failed to parse file {file_path} from cache: {exc}")
        raise


def _get_all_lines(file_path: Path) -> list[str]:
    """Read all lines from a given file and return them as list[str]."""
    try:
        with open(file_path, encoding="utf-8") as file:
            return file.readlines()
    except OSError:
        logger.error(f"Cannot read file {file_path}")
        raise


def _get_lines_section(
        lines: list[str], 
        start: Pattern, 
        end: Pattern, 
        stop_at: Pattern
    ) -> list[str]:
    """
    Extract a section of lines between start and end patterns, 
    with a condition to stop searching.
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
        raise ValueError(f"Pattern '{start.pattern}' not found.")
    
    if end_idx is None:
        raise ValueError(f"Pattern '{end.pattern}' not found.")

    if end_idx < start_idx:
        raise ValueError(
            f"End pattern '{end.pattern}' found before start pattern '{start.pattern}'."
        )

    return lines[start_idx:end_idx]
    

def _extract_doxygen_code_block(
        lines: list[str], 
        code_start_pattern: Pattern, 
        code_end_pattern: Pattern, 
        comment_block_pattern: Pattern
    ) -> list[str]:
    """
    Extract code block from Doxygen with trailing comments removing.
    """

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
                    f"Code end pattern '{code_end_pattern.pattern}' "
                    f"found before code start pattern '{code_start_pattern.pattern}'."
                )
            in_code_block = False
            code_end_found = True
            continue
        
        if in_code_block:
            line = re.sub(comment_block_pattern, '', line)
            code_lines.append(line)

    if not code_start_found:
        raise ValueError(f"Code start pattern '{code_start_pattern.pattern}' not found.")
    
    if not code_end_found:
        raise ValueError(f"Code end pattern '{code_end_pattern.pattern}' not found.")

    if not code_lines:
        raise ValueError(
            f"No code block found between '{code_start_pattern.pattern}' "
            f"and '{code_end_pattern.pattern}' patterns. "
            "Review Doxygen comment formatting in the hpp file."
        )
    
    code_lines[-1] = code_lines[-1].rstrip('\n')
    return code_lines


def define_env(env):
    """
    This is the hook for the variables, macros and filters.
    """

    cache_dir = Path(env.variables["cache_dir"])
    github_repos = env.variables["github_repositories"]
    nav2_tree_nodes_file_path = Path(env.variables["nav2_tree_nodes_file_path"])

    for repo_name, repo_info in github_repos.items():

        local_repo_path = cache_dir / Path(repo_name)
        if local_repo_path.exists() \
        and repo_info["branch"] == _get_git_branch_name(local_repo_path) \
        and _is_git_workdir_synced(local_repo_path):
            logger.info(
                f"Cached Git repository '{local_repo_path}' "
                "is synced with remote GitHub repository. "
                "Skipping clone."
            )
            continue
        try:
            _clone_sparse_github_data(
                repo_name=repo_name, 
                owner=repo_info["owner"], 
                branch=repo_info["branch"], 
                data_to_clone=repo_info["data_to_clone"], 
                clone_dir=cache_dir
            )
        except subprocess.CalledProcessError as exc:
            stderr = getattr(exc, 'stderr', None)
            logger.error(f"Failed to clone GitHub data: {stderr or exc}")
            logger.info("Attempting complete clone as fallback...")
            try:
                _clone_github_repository(
                    repo_name=repo_name, 
                    owner=repo_info["owner"], 
                    branch=repo_info["branch"], 
                    clone_dir=cache_dir
                )
            except (subprocess.CalledProcessError, ValueError, OSError) as exc:
                stderr = getattr(exc, 'stderr', None)
                logger.error(f"Failed to clone GitHub repository: {stderr or exc}")        
                sys.exit(1)
        except (ValueError, OSError) as exc:
            logger.error(f"Failed to clone GitHub data: {exc}")
            sys.exit(1)

    try:
        bt_nodes_model = _load_bt_nodes_model(nav2_tree_nodes_file_path)
    except (ValueError, ET.ParseError, IndexError) as exc:
        logger.exception(
            f"BT node model not found at: {nav2_tree_nodes_file_path}\n"
            "Review paths in macros/variables.yml configuration."
        )
        sys.exit(1)

    @env.macro
    def render_bt_node_ports(bt_node_id: str) -> str:
        """
        Render MkDocs-formatted documentation for a Behavior Tree node's input/output ports.
        """

        node = bt_nodes_model.find(f'.//*[@ID="{bt_node_id}"]')
        if node is None:
            logger.error(f"BT node ID not found: {bt_node_id}.")
            sys.exit(1)

        input_ports, output_ports, bidirectional_ports = _parse_ports(node)

        sections = []
        if input_ports:
            sections.append(
                _PORT_SECTION_TEMPLATE.render(heading='Input Ports', ports=input_ports)
            )
        if output_ports:
            sections.append(
                _PORT_SECTION_TEMPLATE.render(heading='Output Ports', ports=output_ports)
            )
        if bidirectional_ports:
            sections.append(
                _PORT_SECTION_TEMPLATE.render(heading='Bidirectional Ports', ports=bidirectional_ports)
            )

        return '\n\n'.join(sections)
    

    @env.macro
    def render_bt_node_example(file_path: Path, class_name: str | None = None) -> str:
        """
        Render MD-formatted XML code example for a Behavior Tree node 
        from a Doxygen comment in a HPP file.
        """
        
        try:
            file_lines = _get_all_lines(file_path)
        except OSError as exc:
            logger.error(f"Failed to read lines from file: {exc}")
            sys.exit(1)

        class_str = r"^\s*class\s+"
        if class_name:
            class_str += rf"{re.escape(class_name)}\b"
        class_pattern = re.compile(class_str)

        try:
            doxygen_section = _get_lines_section(
                lines=file_lines, 
                start=_DOXYGEN_REGEX_PATTERNS["XML_USAGE_EXAMPLE"], 
                end=_DOXYGEN_REGEX_PATTERNS["COMMENT_END"], 
                stop_at=class_pattern
            )
        except ValueError as exc:
            logger.error(f"Failed to extract lines section from file {file_path}: {exc}")
            sys.exit(1)

        try:
            code_section = _extract_doxygen_code_block(
                lines=doxygen_section, 
                code_start_pattern=_DOXYGEN_REGEX_PATTERNS["CODE_BLOCK_START"], 
                code_end_pattern=_DOXYGEN_REGEX_PATTERNS["CODE_BLOCK_END"], 
                comment_block_pattern=_DOXYGEN_REGEX_PATTERNS["COMMENT_STYLE"]
            )
        except ValueError as exc:
            logger.error(f"Failed to extract code block from file {file_path}: {exc}")
            sys.exit(1)

        code_example = ''.join(code_section)
        code_example_md = _XML_CODE_BLOCK_TEMPLATE.render(xml_code=code_example)

        return code_example_md        

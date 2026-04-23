from __future__ import annotations # Required for Python 3.8 support in CI (ubuntu:focal)
import subprocess, logging, re, sys, os, shutil
import xml.etree.ElementTree as ET
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


def _parse_ports(node: ET.Element) -> tuple:
    """
    Parse input_port and output_port children from an XML BT node.
    Returns (input_ports, output_ports).
    """

    input_ports = []
    output_ports = []

    for child in node:
        
        port_direction = child.tag
        if port_direction not in ('input_port', 'output_port'):
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

    return input_ports, output_ports


def _remove_dir(dir_path: Path):
    """Remove directory if it exists"""
    if os.path.exists(dir_path):
        try:
            shutil.rmtree(dir_path)
            logger.info(f"Removed existing directory: {dir_path}")
        except OSError as exc:
            logger.error(f"Failed to remove existing directory {dir_path}: {exc}")
            raise


def _clone_sparse_github_data(repo: str, branch: str, data_to_clone: list[str], clone_dir: Path):
    """
    Clone GitHub repository sparsely and checkout the specified files/directories.
    """

    if not data_to_clone:
        logger.error("No directories or files specified for sparse checkout.")
        raise ValueError("data_to_clone cannot be empty.")
    
    if not clone_dir.exists():
        logger.error(f"Clone directory does not exist: {clone_dir}")
        raise ValueError(f"clone_dir must exist: {clone_dir}")
    
    repo_work_dir = clone_dir / Path(repo).name
    _remove_dir(repo_work_dir)

    github_url = f"https://github.com/{repo}.git"
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


def _load_bt_nodes_model(file_path: Path ) -> ET.Element | None:
    """
    Load the behavior-tree nodes model from a cached file.
    """

    if not file_path.exists():
        return None

    try:
        root = ET.parse(file_path).getroot()
        return root[0]
    except (ET.ParseError, IndexError) as exc:
        logger.error(f"Failed to parse file {file_path} from cache: {exc}")
        return None

def define_env(env):
    """
    This is the hook for the variables, macros and filters.
    """

    cache_dir = Path(env.variables["cache_dir"])
    nav2_repo = env.variables["nav2_repo"]
    nav2_branch = env.variables["nav2_branch"]
    nav2_data_to_clone = env.variables["nav2_data_to_clone"]
    nav2_tree_nodes_file_path = Path(env.variables["nav2_tree_nodes_file_path"])

    try:
        _clone_sparse_github_data(nav2_repo, nav2_branch, nav2_data_to_clone, cache_dir)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        logger.error(f"Failed to clone GitHub data: {getattr(exc, 'stderr', exc)}")
        sys.exit(1)

    bt_nodes_model = _load_bt_nodes_model(nav2_tree_nodes_file_path)
    
    if bt_nodes_model is None:
        logger.error(
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

        input_ports, output_ports = _parse_ports(node)

        sections = []
        if input_ports:
            sections.append(
                _PORT_SECTION_TEMPLATE.render(heading='Input Ports', ports=input_ports)
            )
        if output_ports:
            sections.append(
                _PORT_SECTION_TEMPLATE.render(heading='Output Ports', ports=output_ports)
            )

        return '\n\n'.join(sections)

from __future__ import annotations # Required for Python 3.8 support in CI (ubuntu:focal)
import requests
import xml.etree.ElementTree as ET
import logging
import re
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


def _fetch_github_file(repo: str, ref: str, file_path: str) -> str:
    """
    Fetch raw file content using the GitHub API.
    """
    
    api_url = f"https://api.github.com/repos/{repo}/contents/{file_path}"

    headers = {
        "Accept": "application/vnd.github.v3.raw",
    }

    response = requests.get(api_url, headers=headers, params={"ref": ref}, timeout=15)
    response.raise_for_status()
    logger.info(f'File {Path(file_path).name} is fetched from the GitHub repository {repo}')
    return response.text


def _cache_file(cache_file_path: Path, data):
    cache_file_path.parent.mkdir(parents=True, exist_ok=True)
    cache_file_path.write_text(data, encoding='utf-8')
    logger.info(f'File {cache_file_path.name} is cached to {cache_file_path.parent} directory')


def _load_bt_nodes_model(
    repo: str,
    ref: str,
    bt_nodes_repo_file_path: str,
    bt_nodes_cache_file_path: Path
) -> ET.Element | None:
    """
    Load BT node model from cache, or fetch from GitHub if missing.
    """

    if bt_nodes_cache_file_path.exists():
        try:
            root = ET.parse(bt_nodes_cache_file_path).getroot()
            return root[0]
        except (ET.ParseError, IndexError) as exc:
            logger.error(f'Failed to parse the file ({bt_nodes_cache_file_path}) from cache: {exc}')
            logger.warning('Falling back to GitHub fetch...')
    
    try:
        xml_content = _fetch_github_file(repo, ref, bt_nodes_repo_file_path)
        _cache_file(bt_nodes_cache_file_path, xml_content)
    except Exception as exc:
        logger.error(f'Failed to fetch the file ({bt_nodes_repo_file_path}) from GitHub: {exc}')
        return None
    
    try:
        root = ET.fromstring(xml_content)
        return root[0]
    except (ET.ParseError, IndexError) as exc:
        logger.error(f'Failed to parse the file ({bt_nodes_repo_file_path}) from GitHub: {exc}')
        return None

def define_env(env):
    """
    This is the hook for the variables, macros and filters.
    """

    repo = env.variables["nav2_repo"]
    ref = env.variables["nav2_ref"]
    bt_nodes_repo_file_path = env.variables["nav2_bt_nodes_file_path"]
    cache_dir = Path(env.variables["cache_dir"])

    bt_nodes_cache_file_path = cache_dir / Path(bt_nodes_repo_file_path).name
    bt_nodes_model = _load_bt_nodes_model(repo, ref, bt_nodes_repo_file_path, bt_nodes_cache_file_path)

    @env.macro
    def render_bt_node_ports(bt_node_id: str) -> str:
        """
        Render MkDocs-formatted documentation for a Behavior Tree node's input/output ports.
        """
        
        if bt_nodes_model is None:
            return (
                '!!! warning "BT node reference data unavailable"\n'
                f'    The behavior tree node model file ({bt_nodes_cache_file_path}) could not be loaded.\n'
            )
        
        node = bt_nodes_model.find(f'.//*[@ID="{bt_node_id}"]')
        if node is None:
            logger.warning(f'BT node not found - No node with ID `{bt_node_id}` found in the BT node model')
            return (
                '!!! warning "BT node not found"\n'
                f'    No node with ID `{bt_node_id}` found in the BT node model.\n'
            )

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

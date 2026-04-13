from __future__ import annotations # Required for Python 3.8 support in CI (ubuntu:focal)
import requests
import xml.etree.ElementTree as ET
import logging
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s - [%(asctime)s][%(filename)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)


def _strip_template_suffix(type_str: str) -> str:
    """Strip allocator template suffix from msg types, e.g.
    'nav2_msgs::msg::Route_<std::allocator<void> >' -> 'nav2_msgs::msg::Route'
    """
    
    if 'msg::' in type_str:
        idx = type_str.find('_<')
        if idx != -1:
            return type_str[:idx]
    return type_str


def _format_default(default_str: str) -> str:
    """Simplify numeric default values by stripping trailing zeros, e.g.
    '0.150000' -> '0.15', '1.000000' -> '1.0'
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
        if child.tag not in ('input_port', 'output_port'):
            continue

        raw_type = child.attrib.get('type', '')
        type = _strip_template_suffix(raw_type)

        raw_default = child.attrib.get('default', 'N/A')
        default = _format_default(raw_default) if type in ('double', 'float') else raw_default

        port = {
            'name': child.attrib.get('name', ''),
            'parameter_type': type,
            'default': default,
            'description': (child.text or '').strip(),
        }

        if child.tag == 'input_port':
            input_ports.append(port)
        elif child.tag == 'output_port':
            output_ports.append(port)

    return input_ports, output_ports


def _render_port_section(heading: str, ports: list) -> str:
    lines = [f'## {heading}\n']
    for p in ports:
        lines.append(f'### **`{p["name"]}`**\n')
        lines.append('| Type | Default |')
        lines.append('|------|---------|')
        lines.append(f'| `{p["parameter_type"]}` | {p["default"]} |\n')
        if p['description']:
            lines.append('Description')
            lines.append(f':   {p["description"]}\n')
    return '\n'.join(lines)


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


def _load_bt_nodes_model(repo: str, ref: str, bt_nodes_repo_file_path: str, bt_nodes_cache_file_path: Path) -> ET.Element | None:
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
            sections.append(_render_port_section('Input Ports', input_ports))
        if output_ports:
            sections.append(_render_port_section('Output Ports', output_ports))

        return '\n\n'.join(sections)

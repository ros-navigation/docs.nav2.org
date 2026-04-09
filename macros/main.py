import requests
import xml.etree.ElementTree as ET


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


# def _get_github_data(repo: str, file_path: str, ref: str) -> str:
#     """
#     Fetch raw file content using the GitHub API.
#     """
    
#     api_url = f"https://api.github.com/repos/{repo}/contents/{file_path}"

#     headers = {
#         "Accept": "application/vnd.github.v3.raw",
#     }

#     response = requests.get(api_url, headers=headers, params={"ref": ref}, timeout=15)
#     response.raise_for_status()
#     return response.text


def define_env(env):
    """
    This is the hook for the variables, macros and filters.
    """

    @env.macro
    def render_bt_node_ports(bt_node_id: str) -> str:

        try:
            tree = ET.parse('./macros/tmp/nav2_tree_nodes.xml')
        except Exception as exc:
            return (
                '!!! warning "Could not fetch BT parameters"\n'
                f'    Failed to load: {exc}\n'
            )
        
        root = tree.getroot()
        bt_nodes = root[0]

        node = bt_nodes.find(f'.//*[@ID="{bt_node_id}"]')
        if node is None:
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

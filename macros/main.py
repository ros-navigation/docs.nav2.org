import requests

def _get_github_data(repo: str, file_path: str, ref: str) -> str:
    """
    Fetch raw file content using the GitHub API.
    """
    
    api_url = f"https://api.github.com/repos/{repo}/contents/{file_path}"

    headers = {
        "Accept": "application/vnd.github.v3.raw",
    }

    response = requests.get(api_url, headers=headers, params={"ref": ref}, timeout=15)
    response.raise_for_status()
    return response.text


def define_env(env):
    """
    This is the hook for the variables, macros and filters.
    """

    @env.macro
    def generate_bt_parameters(file_path: str, ref: str = "") -> str:

        repo = env.variables["nav2_repo"]
        resolved_ref = ref or env.variables["nav2_ref"]

        try:
            source = _get_github_data(repo, file_path, resolved_ref)
        except Exception as exc:
            return (
                '!!! warning "Could not fetch BT parameters"\n'
                f'    Failed to load {file_path!r} from {resolved_ref!r}: {exc}\n'
            )

        return f'```\n{source}\n```'

#!/usr/bin/env python3
"""Generate redirect stubs for pre-MkDocs URLs into a gh-pages checkout.

The legacy Sphinx URLs live at the site root, outside every mike version
directory, so nothing in the MkDocs build can produce them -- they have to be
written straight into the gh-pages branch. See redirects/legacy_map.yml.

Usage:
    python3 scripts/gen_legacy_redirects.py --out /path/to/gh-pages-checkout

The script is strictly additive: it only writes paths named in the map plus
404.html, and refuses to touch the version directories or mike's own files.
"""
import argparse
import json
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP = os.path.join(REPO_ROOT, "redirects", "legacy_map.yml")

SITE_URL = "https://docs.nav2.org"

# Never overwrite these: mike owns them, or they configure GitHub Pages itself.
PROTECTED_FILES = {"index.html", "versions.json", "CNAME", ".nojekyll"}

# Version directories, plus distros that were never published under mike but may
# still appear in hand-built URLs. Used to protect paths and to strip prefixes
# in the 404 handler.
KNOWN_VERSIONS = [
    "rolling", "lyrical", "jazzy", "kilted", "iron", "humble", "galactic", "foxy",
]

STUB = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Redirecting</title>
<link rel="canonical" href="{site}{target}">
<meta http-equiv="refresh" content="0; url={target}">
<script>
  window.location.replace("{target}" + window.location.search + window.location.hash);
</script>
</head>
<body>
  This page has moved to <a href="{target}">{target}</a>.
</body>
</html>
"""


def load_map(path):
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict) or not data:
        raise SystemExit("error: %s did not parse as a non-empty mapping" % path)
    return data


def target_url(new_path, version):
    """Absolute, root-relative URL for a map value."""
    return "/%s/%s" % (version, new_path or "")


def check_writable(out_dir, rel_path):
    """Refuse to write anywhere that belongs to mike or to a published version."""
    top = rel_path.split("/")[0]
    if rel_path in PROTECTED_FILES:
        raise SystemExit("refusing to overwrite mike-owned file: %s" % rel_path)
    if top in KNOWN_VERSIONS:
        raise SystemExit("refusing to write inside version directory: %s" % rel_path)
    full = os.path.realpath(os.path.join(out_dir, rel_path))
    if not full.startswith(os.path.realpath(out_dir) + os.sep):
        raise SystemExit("refusing to write outside output directory: %s" % rel_path)
    return full


def write_stubs(mapping, out_dir, version):
    written = 0
    for legacy, new_path in sorted(mapping.items()):
        full = check_writable(out_dir, legacy)
        os.makedirs(os.path.dirname(full), exist_ok=True)
        target = target_url(new_path, version)
        with open(full, "w", encoding="utf-8") as fh:
            fh.write(STUB.format(site=SITE_URL, target=target))
        written += 1
    return written


def write_404(mapping, out_dir, version):
    """Catch-all for legacy URLs this map does not name.

    GitHub Pages serves the *root* 404.html for any unmatched path sitewide; the
    per-version 404.html files MkDocs generates are never reached.
    """
    routes = {k: target_url(v, version) for k, v in mapping.items()}
    payload = json.dumps(routes, indent=0, sort_keys=True, separators=(",", ":"))
    versions = json.dumps(KNOWN_VERSIONS)

    html = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Page moved &middot; Nav2 documentation</title>
<style>
  :root {{ color-scheme: light dark; }}
  body {{
    font-family: Inter, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    max-width: 40rem; margin: 4rem auto; padding: 0 1.5rem; line-height: 1.6;
  }}
  code {{ background: rgba(128,128,128,.18); padding: .1em .35em; border-radius: .2em; }}
  a {{ color: #2a7ae2; }}
</style>
</head>
<body>
<h1>This page moved</h1>
<p id="msg">The Nav2 documentation was reorganized and is now versioned. Looking for
the new location&hellip;</p>
<p id="fallback" hidden>
  We could not find a new home for <code id="path"></code>.
  Try the <a href="/{version}/">documentation home</a> or
  <a href="/{version}/?q=" id="search">search</a>.
</p>
<script>
(function () {{
  var routes = {payload};
  var versions = {versions};
  var version = "{version}";

  function slug(p) {{
    p = p.replace(/\\/+$/, "");
    p = p.replace(/\\/index\\.html$/, "").replace(/\\.html$/, "");
    var base = p.split("/").pop() || "";
    return base.toLowerCase().replace(/[-_]/g, "");
  }}

  // Slug index over the current map's targets, for near-miss lookups.
  var bySlug = {{}};
  for (var key in routes) {{
    var s = slug(routes[key]);
    if (s && !(s in bySlug)) {{ bySlug[s] = routes[key]; }}
  }}

  function lookup(path) {{
    if (routes[path]) {{ return routes[path]; }}
    if (routes[path + "index.html"]) {{ return routes[path + "index.html"]; }}
    var s = slug(path);
    return s ? bySlug[s] : null;
  }}

  var path = window.location.pathname.replace(/^\\//, "");

  // Try as-is, then with a leading version segment stripped -- catches
  // hand-built /rolling/<old-sphinx-path> guesses.
  var found = lookup(path);
  if (!found) {{
    var parts = path.split("/");
    if (versions.indexOf(parts[0]) !== -1) {{
      found = lookup(parts.slice(1).join("/"));
    }}
  }}

  if (found) {{
    window.location.replace(found + window.location.search + window.location.hash);
    return;
  }}

  // Nothing matched: offer the page name as a search term, words intact.
  var words = path.replace(/\\/+$/, "")
                  .replace(/\\/index\\.html$/, "")
                  .replace(/\\.html$/, "")
                  .split("/").pop()
                  .replace(/[-_]+/g, " ")
                  .trim();

  document.getElementById("msg").hidden = true;
  document.getElementById("path").textContent = window.location.pathname;
  document.getElementById("search").href =
    "/" + version + "/?q=" + encodeURIComponent(words);
  document.getElementById("fallback").hidden = false;
}})();
</script>
</body>
</html>
""".format(payload=payload, versions=versions, version=version)

    with open(os.path.join(out_dir, "404.html"), "w", encoding="utf-8") as fh:
        fh.write(html)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True,
                        help="gh-pages checkout to write the stubs into")
    parser.add_argument("--map", default=DEFAULT_MAP,
                        help="redirect map (default: redirects/legacy_map.yml)")
    parser.add_argument("--version", default="rolling",
                        help="version directory legacy URLs land in (default: rolling)")
    args = parser.parse_args()

    if not os.path.isdir(args.out):
        raise SystemExit("error: --out is not a directory: %s" % args.out)

    mapping = load_map(args.map)
    count = write_stubs(mapping, args.out, args.version)
    write_404(mapping, args.out, args.version)

    print("wrote %d redirect stubs + 404.html into %s (version: %s)"
          % (count, args.out, args.version))
    return 0


if __name__ == "__main__":
    sys.exit(main())

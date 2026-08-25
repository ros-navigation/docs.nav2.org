#!/usr/bin/env python3
"""Generate the redirect page for pre-MkDocs URLs into a gh-pages checkout.

Writes a single file, 404.html, at the root of the gh-pages branch. GitHub Pages
serves the site-root 404.html for any unmatched path across the whole site, so
one page carrying redirects/legacy_map.yml as an embedded route table can
resolve every legacy Sphinx URL -- no per-URL files.

(The 404.html files MkDocs writes inside each version directory are never
reached; GitHub Pages only consults the one at the site root.)

Usage:
    python3 scripts/gen_legacy_redirects.py --out /path/to/gh-pages-checkout

The only path written is <out>/404.html, which nothing in a mike deployment
owns, so this cannot disturb a published version.
"""
import argparse
import json
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP = os.path.join(REPO_ROOT, "redirects", "legacy_map.yml")

# Version directories, plus distros that were never published under mike but may
# still appear in hand-built URLs. The 404 handler strips these as a prefix.
KNOWN_VERSIONS = [
    "rolling", "lyrical", "jazzy", "kilted", "iron", "humble", "galactic", "foxy",
]


def load_map(path):
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict) or not data:
        raise SystemExit("error: %s did not parse as a non-empty mapping" % path)
    return data


def target_url(new_path, version):
    """Absolute, root-relative URL for a map value."""
    return "/%s/%s" % (version, new_path or "")


def write_404(mapping, out_dir, version):
    """Write the single redirect page.

    GitHub Pages serves the *root* 404.html for any unmatched path sitewide, so
    this one page handles every legacy URL -- and every other miss on the site.
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
  Try the <a href="/{version}/" id="home">documentation home</a> or
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

  // A miss inside a published version should offer that version, not another
  // distro -- readers of /jazzy/ pages must not be quietly sent to rolling.
  var parts = path.split("/");
  var inVersion = versions.indexOf(parts[0]) !== -1 ? parts[0] : version;

  // Try as-is, then with a leading version segment stripped -- catches
  // hand-built /rolling/<old-sphinx-path> guesses.
  var found = lookup(path);
  if (!found) {{
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
  document.getElementById("home").href = "/" + inVersion + "/";
  document.getElementById("search").href =
    "/" + inVersion + "/?q=" + encodeURIComponent(words);
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
                        help="gh-pages checkout to write 404.html into")
    parser.add_argument("--map", default=DEFAULT_MAP,
                        help="redirect map (default: redirects/legacy_map.yml)")
    parser.add_argument("--version", default="rolling",
                        help="version directory legacy URLs land in (default: rolling)")
    args = parser.parse_args()

    if not os.path.isdir(args.out):
        raise SystemExit("error: --out is not a directory: %s" % args.out)

    mapping = load_map(args.map)
    write_404(mapping, args.out, args.version)

    print("wrote 404.html (%d routes) into %s (version: %s)"
          % (len(mapping), args.out, args.version))
    return 0


if __name__ == "__main__":
    sys.exit(main())

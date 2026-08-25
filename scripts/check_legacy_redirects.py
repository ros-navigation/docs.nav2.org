#!/usr/bin/env python3
"""Verify redirects/legacy_map.yml against a built site.

Checks that every map target resolves to a real page in a `mkdocs build` output,
and -- when a copy of the old Sphinx build is available -- that every legacy page
has an entry and no entry names a page that never existed.

Usage:
    mkdocs build
    python3 scripts/check_legacy_redirects.py --site site
    python3 scripts/check_legacy_redirects.py --site site --old-build /path/to/sphinx-build
"""
import argparse
import os
import sys

import yaml

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_MAP = os.path.join(REPO_ROOT, "redirects", "legacy_map.yml")

# Legacy pages that must NOT be in the map: mike owns the gh-pages root
# index.html and already redirects / to the default version.
MIKE_OWNED = {"index.html"}


def load_map(path):
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def target_exists(site_dir, new_path):
    """A target resolves if MkDocs emitted an index.html for it."""
    return os.path.isfile(os.path.join(site_dir, new_path, "index.html"))


def old_pages(old_build):
    pages = set()
    for dirpath, dirnames, filenames in os.walk(old_build):
        dirnames[:] = [d for d in dirnames
                       if d not in (".git", "_sources", "_static", "_images")]
        for fn in filenames:
            if fn.endswith(".html"):
                pages.add(os.path.relpath(os.path.join(dirpath, fn), old_build))
    return pages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--site", default="site", help="mkdocs build output directory")
    parser.add_argument("--map", default=DEFAULT_MAP)
    parser.add_argument("--old-build",
                        help="checkout of the pre-migration Sphinx build, for "
                             "coverage checks against the legacy page set")
    args = parser.parse_args()

    if not os.path.isdir(args.site):
        raise SystemExit("error: no such build directory: %s (run `mkdocs build`)"
                         % args.site)

    mapping = load_map(args.map)
    failures = []

    for legacy, new_path in sorted(mapping.items()):
        if not target_exists(args.site, new_path):
            failures.append("target missing from build: %s -> %s"
                            % (legacy, new_path or "<site root>"))

    checked_coverage = False
    if args.old_build:
        if not os.path.isdir(args.old_build):
            raise SystemExit("error: no such directory: %s" % args.old_build)
        checked_coverage = True
        legacy_pages = old_pages(args.old_build)
        for page in sorted(legacy_pages - set(mapping) - MIKE_OWNED):
            failures.append("legacy page has no redirect: %s" % page)
        for page in sorted(set(mapping) & MIKE_OWNED):
            failures.append("map must not claim a mike-owned file: %s" % page)
        for page in sorted(set(mapping) - legacy_pages):
            failures.append("map names a page absent from the old build: %s" % page)

    print("map entries : %d" % len(mapping))
    print("targets ok  : %d" % (len(mapping) - sum(
        1 for f in failures if f.startswith("target missing"))))
    print("coverage    : %s" % ("checked against old build" if checked_coverage
                                else "skipped (--old-build not given)"))

    if failures:
        print("\n%d problem(s):" % len(failures))
        for f in failures:
            print("  " + f)
        return 1

    print("\nOK")
    return 0


if __name__ == "__main__":
    sys.exit(main())

# navigation.ros.org
https://navigation.ros.org/

This folder holds the source and configuration files used to generate the
[Navigation2 documentation](https://navigation.ros.org) web site.

Dependencies for Build: 
* [Sphinx](https://www.sphinx-doc.org/en/master/usage/installation.html)
* `sudo apt install python3-pip`
* `pip3 install breathe==4.12.0 sphinx_rtd_theme sphinxcontrib-plantuml`

Build the docs locally with `make html` and you'll find the built docs entry point in `_build/html/index.html`.


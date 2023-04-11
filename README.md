# navigation.ros.org
https://navigation.ros.org/

This folder holds the source and configuration files used to generate the
[Navigation2 documentation](https://navigation.ros.org) web site.

Dependencies for Build: 
* `sudo apt install python3-pip`
* `pip3 install sphinx==3.5.0 breathe==4.28.0 sphinx_rtd_theme sphinxcontrib-plantuml jinja2==3.0.3 myst-parser`

Build the docs locally with `make html` and you'll find the built docs entry point in `_build/html/index.html`.

Any images, diagrams, or videos are subject to their own copyrights, trademarks, and licenses. 

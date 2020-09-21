# navigation.ros.org
https://navigation.ros.org/

This folder holds the source and configuration files used to generate the
[Navigation2 documentation](https://navigation.ros.org) web site.

Dependencies for Build: 

* `sudo apt install python3-pip`
* `pip3 install sphinx==1.7.5 docutils==0.14 sphinx_rtd_theme breathe==4.9.1 sphinxcontrib-plantuml`

(as in the .circleci/config.yaml)

Build the docs locally with `make html` and you'll find the built docs entry point in `_build/html/index.html`.


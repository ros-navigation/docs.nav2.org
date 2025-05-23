# docs.nav2.org
https://docs.nav2.org/

This folder holds the source and configuration files used to generate the
[Navigation2 documentation](https://docs.nav2.org) web site.


### Native Installation

Dependencies for Build:

``` bash
sudo apt install python3-pip
pip3 install -r requirements.txt
```

### Installation in a Virtual Environment

Install `pip` and `venv` if not already installed:

``` bash
sudo apt install python3-pip python3-venv
```

Create a virtual environment and install the dependencies:

``` bash
python3 -m venv venv
source venv/bin/activate
pip3 install -r requirements.txt
```

### Build the Docs
Build the docs locally with `make html` and you'll find the built docs entry point in `_build/html/index.html`.

Any images, diagrams, or videos are subject to their own copyrights, trademarks, and licenses.

Want a local PDF version? Follow the [instructions here](https://gist.github.com/alfredodeza/7fb5c667addb1c6963b9).

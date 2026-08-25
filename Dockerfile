# This dockerfile is for building the Navigation2 documentation.
#
# To build the image (from the root of the repository):
#   $ docker build -f Dockerfile --build-arg user=$(id -un) --build-arg uid=$(id -u) -t nav2_docs .
#
# To use the image to build the docs:
#   $ docker run --rm -v .:/docs nav2_docs mkdocs build
#
# To run autobuild (watches for changes and rebuilds automatically):
#   $ docker run --init --rm -it -v .:/docs -p 127.0.0.1:8000:8000 nav2_docs mkdocs serve --dev-addr 0.0.0.0:8000
#   Then browse to http://127.0.0.1:8000
#   Use Ctrl+C to stop (--init flag enables proper signal handling)
#
# The built documentation will be in site/

FROM ubuntu:noble

ARG user=nav2doc
ARG uid=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL=/bin/bash

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $uid ; then userdel `id -un $uid` ; fi

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
        git \
        openssh-server \
        python3-pip && \
    rm -rf /var/lib/apt/lists/*

RUN useradd -u $uid -m $user

ENV HOME=/home/$user
ENV PATH="$HOME/.local/bin:$PATH"

USER $user

COPY requirements.txt ./
RUN pip3 install --no-warn-script-location --user --break-system-packages -r requirements.txt

WORKDIR /docs

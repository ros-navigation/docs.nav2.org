FROM ubuntu:focal

ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && \
    apt install -y \
        doxygen \
        git \
        graphviz \
        make \
        openjdk-8-jre \
        openssh-server \
        python3-pip \
        ttf-dejavu

COPY requirements.txt ./
RUN pip3 install -r requirements.txt

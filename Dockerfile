FROM alpine:latest

RUN apk --no-cache add \
        doxygen \
        graphviz \
        make \
        openjdk8-jre \
        python3 \
        ttf-dejavu

RUN python3 -m ensurepip

COPY requirements.txt ./
RUN pip3 install -r requirements.txt

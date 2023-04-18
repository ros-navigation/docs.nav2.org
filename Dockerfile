FROM alpine:latest

#ENV http_proxy <myproxy>
#ENV https_proxy <myproxy>

RUN apk --no-cache add python3
RUN python3 -m ensurepip
COPY requirements.txt ./
RUN pip3 install -r requirements.txt
RUN apk --no-cache add make
RUN apk --no-cache add doxygen
RUN apk --no-cache add graphviz
RUN apk --no-cache add ttf-dejavu
RUN apk --no-cache add openjdk8-jre

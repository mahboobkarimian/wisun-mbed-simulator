FROM ubuntu:22.04
LABEL Maintainer="mahboob.karimian@heig-cd.ch"
RUN apt update && apt install -y \
    build-essential git python3 unzip pkg-config libnl-3-dev libnl-route-3-dev libpcap-dev libsystemd-dev cargo cmake ninja-build

RUN mkdir -p /home/src
WORKDIR /home/src
RUN git clone --branch=v3.0.0 https://github.com/ARMmbed/mbedtls
WORKDIR /home/src/mbedtls
RUN cmake -G Ninja .
RUN ninja
RUN ninja install
RUN mkdir -p /home/wsbr_mounted
RUN apt install -y iputils-ping
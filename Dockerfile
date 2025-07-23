FROM cr.yandex/mirror/ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive

ARG SDK_PKG_NAME
ARG SDK_FOLDER_NAME

ENV SDK_PKG_NAME=${SDK_PKG_NAME}
ENV SDK_FOLDER_NAME=${SDK_FOLDER_NAME}

ENV PATH="${PATH}:/opt/${SDK_FOLDER_NAME}/toolchain/bin:/home/user/.local/bin"
RUN apt-get update && \
    apt upgrade -y && \
    apt install -y \
        net-tools \
        python3 \
        python3-dev \
        python3-pip \
        python3-serial \
        python3-opencv \
        python3-wxgtk4.0 \
        python3-matplotlib \
        python3-lxml \
        python3-pygame \
        python3-future \
        sudo \
        mc \
        vim \
        curl \
        tar \
        expect \
        build-essential \
        device-tree-compiler \
        parted \
        fdisk \
        dosfstools \
        jq \
        linux-firmware \
        && adduser --disabled-password --gecos "" user \
        && echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

COPY ./${SDK_PKG_NAME} /tmp

RUN apt install /tmp/${SDK_PKG_NAME} -y
RUN rm /tmp/${SDK_PKG_NAME} \
    && echo '/opt/${SDK_FOLDER_NAME}/toolchain/lib' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && echo '/opt/${SDK_FOLDER_NAME}/toolchain/x86_64-pc-linux-gnu/aarch64-kos/lib/' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && ldconfig

RUN su -c 'pip3 install PyYAML mavproxy pymavlink --user --upgrade' user

COPY ./ardupilot /home/user/ardupilot
COPY ./kos /home/user/kos
COPY ./planner /home/user/planner
COPY ./tests /home/user/tests
COPY ./mavproxy_buttons /home/user/.local/lib/python3.10/site-packages/MAVProxy/modules/mavproxy_buttons

RUN chown -R 1000:1000 /home/user

CMD ["bash"]

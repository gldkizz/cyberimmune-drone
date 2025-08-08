FROM cr.yandex/mirror/ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && \
    apt install -y \
        mosquitto

COPY ./mqtt-server/default.conf /etc/mosquitto/conf.d/default.conf
COPY ./mqtt-server/aclfile /etc/mosquitto/conf.d/aclfile
COPY ./mqtt-server/pwfile /etc/mosquitto/conf.d/pwfile

CMD ["mosquitto", "-c", "/etc/mosquitto/conf.d/default.conf"]

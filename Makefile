.PHONY: docker-compose-stop docs

## -----------------------------------------------------------------------------
## Makefile используется для автоматизации рутинных задач, таких как:
## запуск, сборка, тестирование, подчистка проекта и отдельных его компонентов.
##
## Подсказки можно увидеть запустив:
## make или make help
## Для сборки и запуска модуля безопасности потребуется установка
## KasperskyOS Community Edition SDK который можно скачать с сайта
## https://os.kaspersky.ru/development/
## Переменная SDK_FOLDER_NAME указывает имя папки в /opt где установлен SDK
SDK_FOLDER_NAME=KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0.166
## Переменная SDK_PKG_NAME указывает имя пакета с Kaspersky OS CE SDK
SDK_PKG_NAME=KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0.166_ru.deb
## -----------------------------------------------------------------------------

# SDK_FOLDER_NAME=KasperskyOS-Community-Edition-RaspberryPi4b-wifi
# SDK_PKG_NAME=KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb

help: ## Покажет эту помощь
	@sed -ne '/@sed/!s/^## //p' $(MAKEFILE_LIST)
	@sed -ne '/@sed/!s/:.*## /\t/p' $(MAKEFILE_LIST) |column -tl 2

docs:
	doxygen Doxyfile

docker: docker-image	## Сборка образов docker (короткая команда)

docker-image: docker-image-simulator docker-image-orvd docker-image-mqtt-server docker-image-ntp-server ## Сборка образов docker

docker-image-simulator: ## Сборка образа docker с модулем безопасности
	docker build ./ -t simulator --build-arg SDK_FOLDER_NAME=$(SDK_FOLDER_NAME) --build-arg SDK_PKG_NAME=$(SDK_PKG_NAME)

docker-image-orvd: ## Сборка образа docker с ОрВД
	docker build -f orvd.Dockerfile -t orvd ./

docker-image-mqtt-server: ## Сборка образа docker с MQTT сервером
	docker build -f mqtt-server.Dockerfile -t mqtt-server ./

docker-image-ntp-server: ## Сборка образа docker с NTP сервером
	docker build -f ntp-server.Dockerfile -t ntp-server ./

clean-docker-compose: ## Подчистка docker-compose после запуска проекта или тестов
	docker compose -f docker-compose-offline.yml down
	docker compose -f docker-compose-online.yml down
	docker compose -f docker-compose-offline-obstacles.yml down
	docker compose -f docker-compose-online-obstacles.yml down
	docker compose -f tests/e2e-offline-docker-compose.yml down
	docker compose -f tests/e2e-online-docker-compose.yml down
	docker compose -f tests/e2e-offline-obstacles-docker-compose.yml down
	docker compose -f tests/e2e-online-obstacles-docker-compose.yml down

clean-containers: clean-docker-compose ## clean-docker-compose + удаление контейнеров
	docker ps -a -q |xargs docker rm

clean-images: ## Удаление образов docker
	docker images --format json |jq -r ".ID" |xargs docker rmi

clean-network: clean-docker-compose ## clean-docker-compose + удаление временных сетей
	docker network rm -f simulator

clean: clean-containers clean-images clean-network ## Запускает все три цели clean-*

offline: docker ## Запуск проекта в режиме offline
	docker compose -f docker-compose-offline.yml up

online: docker ## Запуск проекта в режиме online
	docker compose -f docker-compose-online.yml up

offline-obstacles: docker ## Запуск проекта в режиме offline с киберпрепятствиями
	docker compose -f docker-compose-offline-obstacles.yml up

online-obstacles: docker ## Запуск проекта в режиме online с киберпрепятствиями
	docker compose -f docker-compose-online-obstacles.yml up

offline-multi: docker
	docker compose -f docker-compose-offline-multi.yml up

online-multi: docker
	docker compose -f docker-compose-online-multi.yml up

docker-compose-stop: ## Остановка docker-compose проектов
	docker compose stop

docker-compose-up: docker docker-compose-stop ## Запуск docker-compose проектов
	docker compose up -d

network: ## Создание виртуальной сети simulator для docker
	docker network rm -f simulator
	docker network create --subnet=172.28.0.0/16 --gateway=172.28.5.254 simulator

shell-kos:
	docker run --name kos -w /home/user/kos --user user --net simulator --ip 172.28.0.1 -it --rm simulator /bin/bash -i

shell-kos-real:
	docker run --volume="`pwd`:/home/user/" --name kos -w /home/user/kos --user user --net simulator --ip 172.28.0.1 -it --rm simulator /bin/bash -i

shell-arducopter:
	docker run --name arducopter -w /home/user/ardupilot --user user --net simulator --ip 172.28.0.2 -it --rm simulator /bin/bash -i

shell-arducopter-real:
	docker run --volume="`pwd`:/home/user/" --name arducopter -w /home/user/ardupilot --user user --net simulator --ip 172.28.0.2 -it --rm simulator /bin/bash -i

shell-mavproxy:
	docker run --name mavproxy -w /home/user/mavproxy --user user --net simulator --ip 172.28.0.3 -it --rm simulator /bin/bash -i

shell-mavproxy-real:
	docker run --volume="`pwd`:/home/user/" --name mavproxy -w /home/user/mavproxy --user user --net simulator --ip 172.28.0.3 -it --rm simulator /bin/bash -i

shell-orvd:
	docker run --name orvd -w /home/user/orvd --net simulator -p 8080:8080 --ip 172.28.0.4 -it --rm orvd /bin/bash -i

shell-orvd-real:
	docker run --volume="`pwd`:/home/user/" --name orvd -w /home/user/orvd --net simulator -p 8080:8080 --ip 172.28.0.4 -it --rm orvd /bin/bash -i

start-mqtt-server: ## запуск mqtt сервера в docker контейнере
	docker run --name mqtt-server -p 1883:1883 -p 8883:8883 --rm mqtt-server

start-ntp-server: ## Запуск ntp сервера в docker контейнере
	docker run --name ntp-server -p 123:123/udp --rm ntpd-server

start-orvd: ## Запуск ОрВД в docker контейнере
	[ -n "$$mqttserver" ] || read -p "Please enter MQTT server IP: " mqttserver; \
	docker run --name orvd -ti -e MQTT_HOST=$$mqttserver -w /home/user/orvd -p 8080:8080 --rm orvd

start-mavproxy-client: ## Запуск MAVProxy как ground control с графикой
	xhost +local:
	docker run --name mavproxy-client -w /home/user/mavproxy --user user --net host -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$$DISPLAY -it --rm simulator /bin/bash -c "mavproxy.py --master udp:0.0.0.0:14550 --logfile /home/user/mav.tlog --console --map --load-module=horizon --load-module=buttons" || true
	xhost -local:

e2e-offline: docker-image ## Запуск сквозных тестов в режиме offline
	docker compose -f tests/e2e-offline-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker compose -f tests/e2e-offline-docker-compose.yml down

e2e-offline-real: ## Запуск сквозных тестов в режиме offline на квадрокоптере
	docker compose -f tests/e2e-offline-real-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker compose -f tests/e2e-offline-real-docker-compose.yml down

e2e-online: docker-image ## Запуск сквозных тестов в режиме online
	docker compose -f tests/e2e-online-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker compose -f tests/e2e-online-docker-compose.yml down

e2e-offline-obstacles: docker-image ## Запуск сквозных тестов в режиме offline с киберпрепятствиями
	docker compose -f tests/e2e-offline-obstacles-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker compose -f tests/e2e-offline-obstacles-docker-compose.yml down

e2e-online-obstacles: docker-image ## Запуск сквозных тестов в режиме online с киберпрепятствиями
	docker compose -f tests/e2e-online-obstacles-docker-compose.yml up --abort-on-container-exit --exit-code-from mavproxy
	docker compose -f tests/e2e-online-obstacles-docker-compose.yml down

e2e-tests: e2e-offline e2e-online ## Запуск сквозных тестов e2e-offline и e2e-online

unit-tests: docker-image-simulator ## Запуск unit тестов модуля безопасности
	docker compose -f tests/unit-tests-docker-compose.yml up --abort-on-container-exit --exit-code-from kos
	docker compose -f tests/unit-tests-docker-compose.yml down

unit-orvd-tests: docker-image-orvd ## Запуск unit тестов ОрВД
	docker compose -f tests/unit-orvd-tests-docker-compose.yml up --abort-on-container-exit --exit-code-from orvd
	docker compose -f tests/unit-orvd-tests-docker-compose.yml down

pal-tests: docker-image ## Запуск PAL тестов модуля безопасности
	docker compose -f tests/pal-tests-docker-compose.yml up --abort-on-container-exit --exit-code-from kos
	docker compose -f tests/pal-tests-docker-compose.yml down

all-tests: e2e-tests unit-tests unit-orvd-tests pal-tests ## Запуск всех тестов

#! /usr/bin/bash
export SDK_FOLDER_NAME=KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0.166
export SDK_PKG_NAME=KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0.166_ru.deb
#export SDK_FOLDER_NAME=KasperskyOS-Community-Edition-RaspberryPi4b-wifi
#export SDK_PKG_NAME=KasperskyOS-Community-Edition-RaspberryPi4b-1.3.0_amd64.deb



sudo add-apt-repository universe -y && sudo apt-get update
sudo apt-get upgrade
sudo apt-get install tmux unzip python3 python3-pip python3-future libfuse2 linux-firmware ./${SDK_PKG_NAME}
sudo pip3 install pyserial mavproxy
pip install --target mavproxy/ mavproxy
sudo chmod -R 777 mavproxy

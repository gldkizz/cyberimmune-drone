/**
 * \file
 * \~English
 * \brief Implementation of methods for communication with the ATM server.
 * \details The file contains implementation of methods
 * for requests to an ATM server send and for received responses process.
 *
 * \~Russian
 * \brief Реализация методов для общения с сервером ОРВД.
 * \details В файле реализованы методы для отправки запросов на сервер ОРВД
 * и обработке полученных ответов.
 */

#include "../include/server_connector.h"
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <vector>
#include <thread>

#include <kos_net.h>
#include <mosquittopp.h>

/** \cond */
#define BUFFER_SIZE 1024
#define CONTENT_SIZE 4096
#define CONNECTION_TIMEOUT 5

uint16_t serverPort = 8080;
uint16_t publishPort = 1883;
mosqpp::mosquittopp *publisher, *subscriber;
std::thread subscriberThread;
std::vector<mosquitto_message*> messages;

class MqttSubscriber : public mosqpp::mosquittopp {
    void on_message(const struct mosquitto_message *message) override {
        mosquitto_message* newMessage = new mosquitto_message();
        mosquitto_message_copy(newMessage, message);
        for (int i = 0; i < messages.size(); i++)
            if (!strcmp(messages[i]->topic, newMessage->topic)) {
                mosquitto_message* tmp = messages[i];
                messages[i] = newMessage;
                mosquitto_message_free(&tmp);
                return;
            }
        messages.push_back(newMessage);
    }
};
/** \endcond */

/**
 * \~English Sets "en0" interface MAC-address as drone ID.
 * \return Returns 1 on successful set, 0 otherwise.
 * \~Russian Устанавливает MAC-адрес интерфейса "en0" в качестве идентификатора дрона.
 * \return Возвращает 1 при успешной установке, иначе -- 0.
 */
int setMacId() {
    ifaddrs *address;
    if (getifaddrs(&address) == -1) {
        logEntry("Failed to get MAC-address", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    uint8_t mac[ETHER_ADDR_LEN] = {0};
    for (ifaddrs *ifa = address; ifa != NULL; ifa = ifa->ifa_next) {
        char *name = ifa->ifa_name;
        if ((strcmp(name, "en0") && strcmp(name, "wl0")) || (ifa->ifa_flags & IFF_LOOPBACK))
            continue;
        struct sockaddr_in *sock = (struct sockaddr_in*)(ifa->ifa_addr);
        if ((sock == NULL) || (sock->sin_family != AF_LINK))
            continue;
        struct sockaddr_dl *sdl = satosdl(sock);
        if (sdl->sdl_alen != ETHER_ADDR_LEN)
            continue;
        memcpy(mac, LLADDR(sdl), sdl->sdl_alen);
        break;
    }
    freeifaddrs(address);

    char name[32] = {0};
    snprintf(name, 32, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    setBoardName(name);

    return 1;
}

int initServerConnector() {
    if (!wait_for_network()) {
        logEntry("Connection to network has failed", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }

    mosqpp::lib_init();
    publisher = new mosqpp::mosquittopp();
    publisher->username_pw_set(MQTT_USERNAME, MQTT_PASSWORD);
    publisher->connect(MQTT_IP, publishPort, 3600);
    subscriber = new MqttSubscriber();
    subscriber->username_pw_set(MQTT_USERNAME, MQTT_PASSWORD);
    subscriber->connect(MQTT_IP, publishPort, 3600);

    if (strlen(BOARD_ID))
        setBoardName(BOARD_ID);
    else if (!setMacId())
        return 0;

    char topic[64] = {0};
    char* boardName = getBoardName();
    snprintf(topic, 64, "ping/%s", boardName);
    subscriber->subscribe(NULL, topic);
    snprintf(topic, 64, "api/flight_status/%s", boardName);
    subscriber->subscribe(NULL, topic);
    snprintf(topic, 64, "api/fmission_kos/%s", boardName);
    subscriber->subscribe(NULL, topic);
    snprintf(topic, 64, "api/arm/response/%s", boardName);
    subscriber->subscribe(NULL, topic);
    snprintf(topic, 64, "api/nmission/response/%s", boardName);
    subscriber->subscribe(NULL, topic);
    snprintf(topic, 64, "api/forbidden_zones", boardName);
    subscriber->subscribe(NULL, topic);
    subscriberThread = std::thread([&](){ subscriber->loop_forever(); });

        return 1;
    }

int requestServer(char* query, char* response, uint32_t responseSize) {
    char request[BUFFER_SIZE] = {0};
    snprintf(request, BUFFER_SIZE, "GET %s HTTP/1.1\r\nHost: %s\r\nConnection: close\r\n\r\n", query, SERVER_IP);

    int socketDesc = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socketDesc < 0) {
        logEntry("Failed to create a socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    if (fcntl(socketDesc, F_SETFL, O_NONBLOCK) < 0) {
        logEntry("Failed to configure a socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    sockaddr_in serverAddress = {0};
    serverAddress.sin_family = AF_INET;
    serverAddress.sin_port = htons(serverPort);
    serverAddress.sin_addr.s_addr = inet_addr(SERVER_IP);
    connect(socketDesc, (struct sockaddr *)&serverAddress, sizeof(serverAddress));

    fd_set read, write, except;
    FD_ZERO(&read);
    FD_ZERO(&write);
    FD_ZERO(&except);
    FD_SET(socketDesc, &read);
    FD_SET(socketDesc, &write);
    FD_SET(socketDesc, &except);

    timeval tv;
    tv.tv_sec = CONNECTION_TIMEOUT;
    tv.tv_usec = 0;

    int res = select(NULL, &read, &write, &except, &tv);
    if (res < 0) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Connection to %s:%d has failed", SERVER_IP, serverPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        close(socketDesc);
        return 0;
    }
    else if (res == 0) {
        char logBuffer[256] = {0};
        snprintf(logBuffer, 256, "Connection to %s:%d is timed out", SERVER_IP, serverPort);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        close(socketDesc);
        strncpy(response, "TIMEOUT", 8);
        return 1;
    }
    if (fcntl(socketDesc, F_SETFL, 0) < 0) {
        logEntry("Failed to configure a socket", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    if (send(socketDesc, request, sizeof(request), 0) < 0) {
        logEntry("Failed to send a request", ENTITY_NAME, LogLevel::LOG_WARNING);
        close(socketDesc);
        return 0;
    }

    uint32_t contentLength = 0;
    ssize_t bufferLength = 0;
    char buffer[BUFFER_SIZE] = {0};
    char content[CONTENT_SIZE] = {0};
    while ((bufferLength = recv(socketDesc, buffer, sizeof(buffer), 0)) > 0)
        if (contentLength + bufferLength < CONTENT_SIZE) {
            strncat(content, buffer, bufferLength);
            contentLength += bufferLength;
        }
        else {
            logEntry("Failed to parse response content: the content is too big", ENTITY_NAME, LogLevel::LOG_WARNING);
            close(socketDesc);
            return 0;
        }
    close(socketDesc);

    char* msg = strstr(content, "$");
    uint32_t len = strlen(msg);
    if (msg == NULL) {
        logEntry("Failed to parse response content", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    else if (len > responseSize) {
        logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }

    strncpy(response, msg, len);

    return 1;
}

int publish(char* topic, char* publication) {
    char idTopic[256];
    snprintf(idTopic, 256, "%s/%s", topic, getBoardName());

    if (!publisher->publish(NULL, idTopic, strlen(publication), publication))
        return 1;
    else {
        logEntry("Failed to publish message", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
}

int getSubscription(char* topic, char* message, uint32_t messageSize) {
    int idx = -1;
    for (int i = 0; i < messages.size(); i++)
        if (strstr(messages[i]->topic, topic)) {
            idx = i;
            break;
        }

    if (idx == -1)
        strncpy(message, "", messageSize);
    else {
        strncpy(message, (char*)(messages[idx]->payload), messageSize);
        mosquitto_message_free(&messages[idx]);
        messages.erase(messages.begin() + idx);
    }

    return 1;
}

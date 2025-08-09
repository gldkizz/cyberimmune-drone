/**
 * \file
 * \~English
 * \brief Implementation of methods for ATM server communication simulation.
 * \details The file contains implementation of methods, that simulate
 * requests to an ATM server send and received responses process.
 *
 * \~Russian
 * \brief Реализация методов для имитации общения с сервером ОРВД.
 * \details В файле реализованы методы, имитирующие отправку запросов на сервер ОРВД
 * и обработку полученных ответов.
 */

#include "../include/server_connector.h"

#include <stdio.h>
#include <string.h>

int flightStatusSend, missionSend, areasSend, armSend, newMissionSend;

int initServerConnector() {
    if (strlen(BOARD_ID))
        setBoardName(BOARD_ID);
    else
        setBoardName("00:00:00:00:00:00");

    flightStatusSend = true;
    missionSend = true;
    areasSend = true;
    armSend= false;
    newMissionSend = false;

    return 1;
}

int requestServer(char* query, char* response, uint32_t responseSize) {
    if (strstr(query, "/api/auth?")) {
        if (responseSize < 10) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$Success#", 10);
    }
    else {
        if (responseSize < 3) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(response, "$#", 3);
    }

    return 1;
}

int publish(char* topic, char* publication) {
    if (strstr(topic, "api/arm/request"))
        armSend = true;
    else if (strstr(topic, "api/nmission/request"))
        newMissionSend = true;

    return 1;
}

int getSubscription(char* topic, char* message, uint32_t messageSize) {
    if (strstr(topic, "ping/")) {
        if (messageSize < 10) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Delay 1#", 10);
    }
    else if (strstr(topic, "api/flight_status/") && flightStatusSend) {
        if (messageSize < 11) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Flight 0#", 11);
        flightStatusSend = false;
    }
    else if (strstr(topic, "api/fmission_kos/") && missionSend) {
        if (messageSize < 880) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$FlightMission H60.0025609_27.8572917_0.0&T1.0&W60.002597_27.8572915_1.0&W60.002588_27.8572015_1.0&W60.002642_27.8572015_1.0&W60.002642_27.8572915_1.0&D1.0&W60.002642_27.8572015_1.0&W60.002615_27.8572015_1.0&W60.002615_27.8572555_1.0&W60.002615_27.8572015_1.0&W60.002642_27.8572015_1.0&W60.002642_27.8571475_1.0&W60.002642_27.8571115_1.0&W60.002615_27.8571115_1.0&W60.002588_27.8571475_1.0&W60.002597_27.8570755_1.0&W60.002588_27.8570755_1.0&W60.002588_27.8570215_1.0&W60.002588_27.8569855_1.0&W60.002642_27.8569855_1.0&D3.0&S5.0_1200.0&D1.0&S5.0_1800.0&W60.002561_27.8569855_1.0&W60.002561_27.8572915_1.0&L0.0_0.0_0.0&I60.002597_27.8572915_0.0&I60.002588_27.8572015_0.0&I60.002615_27.8572555_0.0&I60.002642_27.8572915_0.0&I60.002642_27.8571475_0.0&I60.002615_27.8571115_0.0&I60.002588_27.8571475_0.0&I60.002597_27.8570755_0.0&I60.002588_27.8570215_0.0&I60.002642_27.8570575_0.0#", 880);
        missionSend = false;
    }
    else if (strstr(topic, "api/forbidden_zones") && areasSend) {
        if (messageSize < 974) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$ForbiddenZones 5&outerOne&7&60.002543_27.8573275&60.00266_27.8573275&60.00266_27.8569675&60.002651_27.8569675&60.002651_27.8573095&60.002543_27.8573095&60.002543_27.8573275&outerTwo&7&60.002552_27.8573095&60.002552_27.8569675&60.00266_27.8569675&60.00266_27.8569495&60.002543_27.8569495&60.002543_27.8573095&60.002552_27.8573095&innerOne&7&60.002606_27.8573095&60.002606_27.8572735&60.002624_27.8572735&60.002624_27.8572375&60.002633_27.8572375&60.002633_27.8573095&60.002606_27.8573095&innerTwo&11&60.00257_27.8572735&60.002579_27.8572735&60.002579_27.8571835&60.002633_27.8571835&60.002633_27.8571295&60.002624_27.8571295&60.002624_27.8571655&60.002579_27.8571655&60.002579_27.8570035&60.00257_27.8570035&60.00257_27.8572735&innerThree&11&60.002597_27.8570395&60.002624_27.8570395&60.002624_27.8570935&60.002651_27.8570935&60.002651_27.8570755&60.002633_27.8570755&60.002633_27.8570395&60.002651_27.8570395&60.002651_27.8570035&60.002597_27.8570035&60.002597_27.8570395#", 974);
        areasSend = false;
    }
    else if (strstr(topic, "api/arm/response/") && armSend) {
        if (messageSize < 16) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Arm 0$Delay 1#", 16);
    }
    else if (strstr(topic, "api/nmission/response/") && newMissionSend) {
        if (messageSize < 13) {
            logEntry("Size of response does not fit given buffer", ENTITY_NAME, LogLevel::LOG_WARNING);
            return 0;
        }
        strncpy(message, "$Approve 0#", 13);
    }
    else
        strcpy(message, "");

    return 1;
}
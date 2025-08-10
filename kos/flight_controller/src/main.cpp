/**
 * \file
 * \~English \brief Implementation of the security module FlightController component main loop.
 * \~Russian \brief Реализация основного цикла компонента FlightController модуля безопасности.
 */

#include "../include/flight_controller.h"
#include "../../shared/include/initialization_interface.h"
#include "../../shared/include/ipc_messages_initialization.h"
#include "../../shared/include/ipc_messages_autopilot_connector.h"
#include "../../shared/include/ipc_messages_credential_manager.h"
#include "../../shared/include/ipc_messages_navigation_system.h"
#include "../../shared/include/ipc_messages_periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"
#include "../../shared/include/ipc_messages_logger.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <thread>
#include <chrono> //

/** \cond */
#define RETRY_DELAY_SEC 1
#define RETRY_REQUEST_DELAY_SEC 5
#define FLY_ACCEPT_PERIOD_US 500000

// Конфигурация разрыва связи с ОРВД
uint32_t lastSuccessfulContact = 0;
const uint32_t CONNECTION_TIMEOUT_SEC = 3; // Таймаут разрыва связи
bool connectionLost = false;

char boardId[32] = {0};
uint32_t sessionDelay;
std::thread sessionThread, updateThread;
/** \endcond */

uint32_t getCurrentTime() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

/**
 * \~English Procedure that checks connection to the ATM server.
 * \~Russian Процедура, проверяющая наличие соединения с сервером ОРВД.
 */

void pingSession() {
    sleep(sessionDelay);
    char pingMessage[1024] = {0};
    uint32_t lastResponseTime = getCurrentTime(); // Добавляем отслеживание времени последнего ответа
    const uint32_t RESPONSE_TIMEOUT_MS = 3000;    // Таймаут 3 секунды
    bool isFlightPaused = false;                  // Флаг состояния паузы
    while (true) {
        if (!receiveSubscription("ping/", pingMessage, 1024)) {
            logEntry("Failed to receive ping through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            continue;
        }
        if (strcmp(pingMessage, "")) {
            uint8_t authenticity = 0;
            if (!checkSignature(pingMessage, authenticity) || !authenticity) {
                logEntry("Failed to check signature of ping received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
                continue;
            }

            if (isFlightPaused) {
                if (resumeFlight()) {
                    logEntry("Connection restored - flight resumed", ENTITY_NAME, LogLevel::LOG_INFO);
                    isFlightPaused = false;
                } else {
                    logEntry("Failed to resume flight", ENTITY_NAME, LogLevel::LOG_ERROR);
                }
            }

            lastResponseTime = getCurrentTime();

            //Processing delay until next session
            sessionDelay = parseDelay(strstr(pingMessage, "$Delay "));
        }
        else {
            //No response from the server
            //If server does not respond for 3 more seconds, flight must be paused until the response is received
            uint32_t currentTime = getCurrentTime();
            if (currentTime - lastResponseTime > RESPONSE_TIMEOUT_MS && !isFlightPaused) {
                logEntry("CRITICAL: No server response for 3 seconds - pausing flight", 
                        ENTITY_NAME, LogLevel::LOG_CRITICAL);
                if (pauseFlight()) {
                    isFlightPaused = true;
                    logEntry("Flight paused successfully", ENTITY_NAME, LogLevel::LOG_INFO);
                } else {
                    logEntry("Failed to pause flight", ENTITY_NAME, LogLevel::LOG_ERROR);
                }
            }
        }
        sleep(sessionDelay);
    }
}

/**
 * \~English Procedure that tracks flight status and no flight areas changes.
 * \~Russian Процедура, отслеживающая изменение статуса полета и запретных зон.
 */
void serverUpdateCheck() {
    char message[4096] = {0};

    while (true) {
        if (receiveSubscription("api/flight_status/", message, 4096)) {
            if (strcmp(message, "")) {
                uint8_t authenticity = 0;
                if (checkSignature(message, authenticity) || !authenticity) {
                    if (strstr(message, "$Flight -1$")) {
                        logEntry("Emergency stop request is received. Disabling motors", ENTITY_NAME, LogLevel::LOG_INFO);
                        if (!enableBuzzer())
                            logEntry("Failed to enable buzzer", ENTITY_NAME, LogLevel::LOG_WARNING);
                        while (!setKillSwitch(false)) {
                            logEntry("Failed to forbid motor usage. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
                            sleep(1);
                        }
                    }
                    //The message has two other possible options:
                    //  "$Flight 1$" that requires to pause flight and remain landed
                    //  "$Flight 0$" that requires to resume flight and keep flying
                    //Implementation is required to be done
                }
                else
                    logEntry("Failed to check signature of flight status received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            }
        }
        else
            logEntry("Failed to receive flight status through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);

        if (receiveSubscription("api/forbidden_zones", message, 4096)) {
            if (strcmp(message, "")) {
                uint8_t authenticity = 0;
                if (checkSignature(message, authenticity) || !authenticity) {
                    deleteNoFlightAreas();
                    loadNoFlightAreas(message);
                    logEntry("New no-flight areas are received from the server", ENTITY_NAME, LogLevel::LOG_INFO);
                    printNoFlightAreas();
                    //Path recalculation must be done if current path crosses new no-flight areas
                }
                else
                    logEntry("Failed to check signature of no-flight areas received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            }
        }
        else
            logEntry("Failed to receive no-flight areas through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);

        sleep(1);
    }
}

/**
 * \~English Auxiliary procedure. Asks the ATM server to approve new mission and parses its response.
 * \param[in] mission New mission in string format.
 * \param[out] result ATM server response: 1 if mission approved, 0 otherwise.
 * \return Returns 1 on successful send, 0 otherwise.
 * \~Russian Вспомогательная процедура. Просит у сервера ОРВД одобрения новой миссии и обрабатывает ответ.
 * \param[in] mission Новая миссия в виде строки.
 * \param[out] result Ответ сервера ОРВД: 1 при одобрении миссии, иначе -- 0.
 * \return Возвращает 1 при успешной отправке, иначе -- 0.
 */
int askForMissionApproval(char* mission, int& result) {
    int messageSize = 512 + strlen(mission);
    char *message = (char*)malloc(messageSize);
    char signature[257] = {0};

    snprintf(message, messageSize, "/api/nmission?id=%s&mission=%s", boardId, mission);
    if (!signMessage(message, signature, 257)) {
        logEntry("Failed to sign new mission at Credential Manager", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(message);
        return 0;
    }

    snprintf(message, 512, "mission=%s&sig=0x%s", mission, signature);
    if (!publishMessage("api/nmission/request", message)) {
        logEntry("Failed to publish new mission through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(message);
        return 0;
    }

    while (!receiveSubscription("api/nmission/response/", message, 512) || !strcmp(message, ""))
        sleep(1);

    uint8_t authenticity = 0;
    if (!checkSignature(message, authenticity) || !authenticity) {
        logEntry("Failed to check signature of new mission received through Server Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(message);
        return 0;
    }

    if (strstr(message, "$Approve 0#") != NULL)
        result = 1;
    else if (strstr(message, "$Approve 1#") != NULL)
        result = 0;
    else {
        logEntry("Failed to parse server response on New Mission request", ENTITY_NAME, LogLevel::LOG_WARNING);
        free(message);
        return 0;
    }

    free(message);
    return 1;
}

/**
 * \~English Security module main loop. Waits for all other components to initialize. Authenticates
 * on the ATM server and receives the mission from it. After a mission and an arm request from the autopilot
 * are received, requests permission to take off from the ATM server. On receive supplies power to motors.
 * Then flight control must be performed.
 * \return Returns 1 on completion with no errors.
 * \~Russian Основной цикл модуля безопасности. Ожидает инициализации всех остальных компонентов. Аутентифицируется
 * на сервере ОРВД и получает от него миссию. После получения миссии и запроса на арминг от автопилота, запрашивает разрешение
 * на взлет у сервера ОРВД. При его получении подает питание на двигатели. Далее должен выполняться контроль полета.
 * \return Возвращает 1 при завершении без ошибок.
 */
int main(void) {
    char logBuffer[256] = {0};
    char signBuffer[257] = {0};
    char publicationBuffer[1024] = {0};
    char subscriptionBuffer[4096] = {0};
    //Before do anything, we need to ensure, that other modules are ready to work
    while (!waitForInit("logger_connection", "Logger")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Logger. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("periphery_controller_connection", "PeripheryController")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Periphery Controller. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("autopilot_connector_connection", "AutopilotConnector")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Autopilot Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("navigation_system_connection", "NavigationSystem")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Navigation System. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("server_connector_connection", "ServerConnector")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    while (!waitForInit("credential_manager_connection", "CredentialManager")) {
        snprintf(logBuffer, 256, "Failed to receive initialization notification from Credential Manager. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }

    //Get ID from ServerConnector
    while (!getBoardId(boardId)) {
        logEntry("Failed to get board ID from ServerConnector. Trying again in 1s", ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(1);
    }
    snprintf(logBuffer, 256, "Board '%s' is initialized", boardId);
    logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_INFO);

    //Enable buzzer to indicate, that all modules has been initialized
    if (!enableBuzzer())
        logEntry("Failed to enable buzzer at Periphery Controller", ENTITY_NAME, LogLevel::LOG_WARNING);

    //Copter need to be registered at ORVD
    char authRequest[512] = {0};
    char authSignature[257] = {0};
    snprintf(authRequest, 512, "/api/auth?id=%s", boardId);
    while (!signMessage(authRequest, authSignature, 257)) {
        snprintf(logBuffer, 256, "Failed to sign auth message at Credential Manager. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }

    char authResponse[1024] = {0};
    snprintf(authRequest, 512, "%s&sig=0x%s", authRequest, authSignature);
    while (!sendRequest(authRequest, authResponse, 1024) || !strcmp(authResponse, "TIMEOUT")) {
        snprintf(logBuffer, 256, "Failed to send auth request through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }

    uint8_t authenticity = 0;
    while (!checkSignature(authResponse, authenticity) || !authenticity) {
        snprintf(logBuffer, 256, "Failed to check signature of auth response received through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
    logEntry("Successfully authenticated on the server", ENTITY_NAME, LogLevel::LOG_INFO);

    //Constantly ask server, if mission for the drone is available. Parse it and ensure, that mission is correct
    while (!receiveSubscription("api/fmission_kos/", subscriptionBuffer, 4096) || !strcmp(subscriptionBuffer, ""))
        sleep(1);

    authenticity = 0;
    while (!checkSignature(subscriptionBuffer, authenticity) || !authenticity) {
        snprintf(logBuffer, 256, "Failed to check signature of mission received through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
        logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
        sleep(RETRY_DELAY_SEC);
    }
        if (loadMission(subscriptionBuffer)) {
        logEntry("Successfully received mission from the server", ENTITY_NAME, LogLevel::LOG_INFO);
        printMission();
    }

    //The drone is ready to arm
    logEntry("Ready to arm", ENTITY_NAME, LogLevel::LOG_INFO);
    while (true) {
        //Wait, until autopilot wants to arm (and fails so, as motors are disabled by default)
        while (!waitForArmRequest()) {
            snprintf(logBuffer, 256, "Failed to receive an arm request from Autopilot Connector. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }
        logEntry("Received arm request. Notifying the server", ENTITY_NAME, LogLevel::LOG_INFO);

        //When autopilot asked for arm, we need to receive permission from ORVD
        snprintf(publicationBuffer, 1024, "/api/arm?id=%s", boardId);
        while (!signMessage(publicationBuffer, signBuffer, 257)) {
            snprintf(logBuffer, 256, "Failed to sign arm request at Credential Manager. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }

        snprintf(publicationBuffer, 1024, "sig=0x%s", signBuffer);
        while (!publishMessage("api/arm/request", publicationBuffer)) {
            snprintf(logBuffer, 256, "Failed to publish arm request through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }

        while (!receiveSubscription("api/arm/response/", subscriptionBuffer, 4096) || !strcmp(subscriptionBuffer, ""))
            sleep(1);

        authenticity = 0;
        while (!checkSignature(subscriptionBuffer, authenticity) || !authenticity) {
            snprintf(logBuffer, 256, "Failed to check signature of arm response received through Server Connector. Trying again in %ds", RETRY_DELAY_SEC);
            logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
            sleep(RETRY_DELAY_SEC);
        }

        if (strstr(subscriptionBuffer, "$Arm 0$")) {
            //If arm was permitted, we enable motors
            logEntry("Arm is permitted", ENTITY_NAME, LogLevel::LOG_INFO);
            while (!setKillSwitch(true)) {
                snprintf(logBuffer, 256, "Failed to permit motor usage at Periphery Controller. Trying again in %ds", RETRY_DELAY_SEC);
                logEntry(logBuffer, ENTITY_NAME, LogLevel::LOG_WARNING);
                sleep(RETRY_DELAY_SEC);
            }
            if (!permitArm())
                logEntry("Failed to permit arm through Autopilot Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
            //Get time until next session
            sessionDelay = parseDelay(strstr(subscriptionBuffer, "$Delay "));
            //Start ORVD threads
            sessionThread = std::thread(pingSession);
            updateThread = std::thread(serverUpdateCheck);
            break;
        }
        else if (strstr(subscriptionBuffer, "$Arm 1$")) {
            logEntry("Arm is forbidden", ENTITY_NAME, LogLevel::LOG_INFO);
            if (!forbidArm())
                logEntry("Failed to forbid arm through Autopilot Connector", ENTITY_NAME, LogLevel::LOG_WARNING);
        }
        else
            logEntry("Failed to parse server response", ENTITY_NAME, LogLevel::LOG_WARNING);
        logEntry("Arm was not allowed. Waiting for another arm request from autopilot", ENTITY_NAME, LogLevel::LOG_WARNING);
    };

    //If we get here, the drone is able to arm and start the mission
    //The flight is need to be controlled from now on
    
    // Конфигурация защиты груза
    const int32_t TARGET_LAT = 600026420;
    const int32_t TARGET_LON = 278569855;
    const double EPSILON = 1000; // погрешность
    setCargoLock(0);
    bool cargoLocked = true; // флаг блокировки сброса груза

    // Конфигурация защиты скорости
    const float ALLOWED_SPEED = 0.5f; // 0.5 м/с
    const float SPEED_TOLERANCE = 0.2f; // допустимое превышение скорости
    const float SPEED_CHECK_INTERVAL_MS = 500; // мс
    const int EMERGENCY_RESPONSE_DELAY_MS = 1000; // задержка экстренного реагирования
    static uint32_t lastSpeedCheckTime = 0;    
    static bool speedViolationDetected = false;

    // Конфигурация защиты высоты
    const int32_t MAX_ALTITUDE = 200;
    const int32_t ALTITUDE_TOLERANCE = 10;
    bool altitudeViolationDetected = false;

    // Конфигурация точек интереса
    struct PointOfInterest {
        int32_t latitude;
        int32_t longitude;
        double epsilon; // Радиус зоны точки интереса
        bool scanned;  // Флаг, была ли точка уже отсканирована
        bool scanning; // Флаг, что в данный момент идет сканирование
        uint8_t scan_attempts; // Добавляем счетчик попыток
        uint32_t last_scan_time; // Время последней попытки
    };

    // Точки интереса
    std::vector<PointOfInterest> pointsOfInterest = {
        {600025970, 278572915, 100, false, false, 0, 0}, //  Точки сканирования 1
        {600025880, 278572015, 100, false, false, 0, 0},  //  Точка сканирования 2
        {600026150, 278572555, 100, false, false, 0, 0},   //  Точка сканирования 3
        {600026420, 278572915, 100, false, false, 0, 0},   //  Точка сканирования 4
        {600026420, 278571475, 100, false, false, 0, 0},   //  Точка сканирования 5
        {600026150, 278571115, 100, false, false, 0, 0},   //  Точка сканирования 6
        {600025880, 278571475, 100, false, false, 0, 0},   //  Точка сканирования 7
        {600025970, 278570755, 100, false, false, 0, 0},   //  Точка сканирования 8
        {600025880, 278570215, 100, false, false, 0, 0},   //  Точка сканирования 9
        {600026420, 278570575, 100, false, false, 0, 0}   //  Точка сканирования 10
    };

    // Конфигурация сканирования RFID
    const double POI_CHECK_INTERVAL_MS = 500; // Интервал проверки POI (мс)
    uint32_t lastPoiCheckTime = getCurrentTime();


    while (true) {
        // 1. Проверка сброса груза
        int32_t latitude, longitude, currentAlt;
        
        if (getCoords(latitude,longitude,currentAlt)) {
            bool inZone = (abs(latitude - TARGET_LAT) < EPSILON) && (abs(longitude - TARGET_LON) < EPSILON);
            // char coordMsg[128];
            // snprintf(coordMsg, sizeof(coordMsg), 
            //         "Coords: lat=%.7f, lon=%.7f, alt=%d, inZone=%d, cargoLocked=%d",
            //         latitude/1e7, longitude/1e7, currentAlt, inZone, cargoLocked);
            // logEntry(coordMsg, ENTITY_NAME, LogLevel::LOG_INFO);
            if(inZone && cargoLocked) {
                setCargoLock(1);
                cargoLocked = false;
                logEntry("Cargo unlocked - in drop zone", ENTITY_NAME, LogLevel::LOG_INFO);
            } else if (!inZone && !cargoLocked) {
                setCargoLock(0);
                cargoLocked = true;
                logEntry("Cargo locked - left drop zone", ENTITY_NAME, LogLevel::LOG_INFO);
            }

            // логирование высоты
            char altBuffer[64];

            // 2. Проверка превышения высоты
            if(currentAlt > MAX_ALTITUDE + ALTITUDE_TOLERANCE) {
                if(!altitudeViolationDetected) {
                    char altMsg[128];
                    snprintf(altMsg, sizeof(altMsg),
                            "ALTITUDE VIOLATION: Current %.2f (Allowed %.2f ± %.2f)",
                            currentAlt, MAX_ALTITUDE, ALTITUDE_TOLERANCE);
                    logEntry(altMsg, ENTITY_NAME, LogLevel::LOG_WARNING);
                    altitudeViolationDetected = true;
                    changeAltitude(static_cast<int32_t>(MAX_ALTITUDE));
                }
            } else if(altitudeViolationDetected) {
                logEntry("Altitude returned to normal limits", ENTITY_NAME, LogLevel::LOG_INFO);
                altitudeViolationDetected = false;
            }
        } else {
            logEntry("Failed to get coordinates - locking cargo", ENTITY_NAME, LogLevel::LOG_WARNING);
            setCargoLock(0);
            cargoLocked = true;
        }

        // 3. Проверка скорости
        float currentSpeed = 0.0f;
        uint32_t currentTime = getCurrentTime();
        if (currentTime - lastSpeedCheckTime >= SPEED_CHECK_INTERVAL_MS) {
            lastSpeedCheckTime = currentTime;
        
            if (getEstimatedSpeed(currentSpeed)) {
                bool isViolation = (currentSpeed > ALLOWED_SPEED + SPEED_TOLERANCE) || 
                                (currentSpeed < ALLOWED_SPEED - SPEED_TOLERANCE);

                if (isViolation) {
                    if (!speedViolationDetected) {
                        // Форматируем информативное сообщение
                        char speedMsg[128];
                        snprintf(speedMsg, sizeof(speedMsg),
                                "SPEED VIOLATION: Current %.2f m/s (Allowed %.2f ± %.2f m/s)",
                                currentSpeed, ALLOWED_SPEED, SPEED_TOLERANCE);
                        
                        logEntry(speedMsg, ENTITY_NAME, LogLevel::LOG_WARNING);
                        speedViolationDetected = true;
                        
                        // Корректируем скорость
                        changeSpeed(static_cast<int32_t>(ALLOWED_SPEED * 100));
                        
                        // Добавляем задержку для стабилизации
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(EMERGENCY_RESPONSE_DELAY_MS));
                    }
                } else {
                    if (speedViolationDetected) {
                        logEntry("Speed returned to normal limits", ENTITY_NAME, LogLevel::LOG_INFO);
                        speedViolationDetected = false;
                    }
                    // // Логируем текущую скорость (для отладки)
                    // char normalMsg[64];
                    // snprintf(normalMsg, sizeof(normalMsg), 
                    //         "Current speed: %.2f m/s (normal)", currentSpeed);
                    // logEntry(normalMsg, ENTITY_NAME, LogLevel::LOG_INFO); // TODO: Поменять на LOG_INFO
                }
            } else {
                logEntry("Failed to get speed data - activating safety mode", 
                        ENTITY_NAME, LogLevel::LOG_ERROR);
                pauseFlight();
            }
        }

        // 4. Сканирование RFID меток  
        if (getCoords(latitude, longitude, currentAlt)) {
            for (auto& poi : pointsOfInterest) {
                if (!poi.scanned) {
                    bool inPoiZone = (abs(latitude - poi.latitude) < poi.epsilon) && (abs(longitude - poi.longitude) < poi.epsilon);
                    
                    if (!poi.scanning && inPoiZone) {
                        // Впервые достигли точки интереса - приостанавливаем полет
                        poi.scan_attempts = 0;
                        logEntry("Approached POI - pausing flight for RFID scan", ENTITY_NAME, LogLevel::LOG_INFO);

                        std::this_thread::sleep_for(std::chrono::seconds(2));


                        if (!pauseFlight()) {
                            logEntry("Pause command failed", ENTITY_NAME, LogLevel::LOG_ERROR);
                            continue;
                        }
                        
                        poi.scanning = true;
                        poi.last_scan_time = getCurrentTime();
                    }

                    if (poi.scanning) {
                        if(poi.scan_attempts >= 3) {
                            logEntry("Max scan attempts (3) reached for this POI", ENTITY_NAME, LogLevel::LOG_WARNING);
                            poi.scanned = true; // Помечаем как обработанную
                            poi.scanning = false;

                            if (!resumeFlight()) {
                                logEntry("Failed to resume flight after max attempts", ENTITY_NAME, LogLevel::LOG_ERROR);
                            } else {
                                logEntry("\n\nResume flight\n\n", ENTITY_NAME, LogLevel::LOG_ERROR);
                            }
                            continue; // Переходим к следующей точке
                        }
                        // Продолжаем сканирование в приостановленном состоянии

                        uint32_t current_time = getCurrentTime();
                        if (current_time - poi.last_scan_time < 1000) {
                            continue; // Ждем между попытками
                        }

                        logEntry("Starting RFID scan", ENTITY_NAME, LogLevel::LOG_INFO);
                        
                        uint8_t scanResult = 0;
                        if (scanRfid(scanResult)) {
                            poi.scan_attempts++;
                            poi.last_scan_time = current_time;
                            if (scanResult) {
                                logEntry("RFID scan successful", ENTITY_NAME, LogLevel::LOG_INFO);
                                poi.scanned = true;
                                poi.scanning = false;

                                if (!resumeFlight()) {
                                    logEntry("Failed to resume flight after RFID scan", ENTITY_NAME, LogLevel::LOG_ERROR);
                                }
                            } else {
                                logEntry("RFID scan failed - no tag detected", ENTITY_NAME, LogLevel::LOG_WARNING);
                                // После неудачного сканирования все равно возобновляем полет

                                if (poi.scan_attempts >= 3) {
                                    logEntry("All scan attempts exhausted for this POI", ENTITY_NAME, LogLevel::LOG_WARNING);
                                    poi.scanned = true; // Помечаем как обработанную
                                    poi.scanning = false;

                                    if (!resumeFlight()) {
                                        logEntry("Failed to resume flight after RFID scan", ENTITY_NAME, LogLevel::LOG_ERROR);
                                    }
                                }
                            }
                        } else {
                            logEntry("RFID scan procedure failed", ENTITY_NAME, LogLevel::LOG_ERROR);
                            poi.scan_attempts++;
                            poi.last_scan_time = current_time;
                            // В случае ошибки сканирования возобновляем полет
                            if (poi.scan_attempts >= 3) {
                                poi.scanned = true;
                                poi.scanning = false;
                                
                                if (!resumeFlight()) {
                                    logEntry("Failed to resume flight after scan error", ENTITY_NAME, LogLevel::LOG_ERROR);
                                }
                            }
                        }
                    }
                }
            }
        }

        usleep(100000);
    }

    return EXIT_SUCCESS;
}

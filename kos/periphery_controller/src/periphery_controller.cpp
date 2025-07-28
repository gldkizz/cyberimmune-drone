/**
 * \file
 * \~English
 * \brief Implementation of methods for peripherals control.
 * \details The file contains implementation of methods of general peripherals control logic,
 * regardless of peripheral implementation.
 *
 * \~Russian
 * \brief Реализация методов для управления периферией.
 * \details В файле реализованы методы, реализующие общую логику управления периферийными
 * устройствами, вне знависимости от реализации периферии.
 */

#include "../include/periphery_controller.h"
#include "../../shared/include/ipc_messages_server_connector.h"

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <thread>

/** \cond */
std::thread buzzerThread;
bool buzzerEnabled = false;
int buzzTime = 2;
/** \endcond */

/**
 * \~English The procedure turns on the buzzer, waits for 2 seconds, and then turns it off.
 * It is assumed that this procedure is performed in a parallel thread.
 * \~Russian Процедура включает зуммер, ожидает 2 секунды, после чего отключает его.
 * Предполагается, что данная процедура выполняется в параллельной нити.
 */
void buzz() {
    buzzerEnabled = true;
    setBuzzer(true);
    clock_t startTime = clock();
    while (true) {
        clock_t time = clock() - startTime;
        if ((time / CLOCKS_PER_SEC) >= buzzTime)
            break;
    }
    setBuzzer(false);
    buzzerEnabled = false;
}

int startBuzzer() {
    if (buzzerEnabled)
        return 0;
    if (buzzerThread.joinable())
        buzzerThread.join();
    buzzerThread = std::thread(buzz);
    if (!publishMessage("api/events", "type=buzzer&event=Buzzer is enabled"))
        logEntry("Failed to publish event message", ENTITY_NAME, LogLevel::LOG_WARNING);
    return 1;
}

int readRfid(uint8_t &foundTag) {
    foundTag = 0;
    char tag[36] = {0};

    if (!readRfid(tag))
        return 0;

    if (strcmp(tag, "")) {
        foundTag = 1;
        char publication[1024] = {0};
        snprintf(publication, 1024, "tag=%s", tag);
        if (!publishMessage("api/rfid", publication))
            logEntry("Failed to publish scanned RFID tag", ENTITY_NAME, LogLevel::LOG_WARNING);
    }

    return 1;
}
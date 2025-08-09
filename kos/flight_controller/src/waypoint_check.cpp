#include <ipc_messages_credential_manager.h>
#include <ipc_messages_logger.h>
#include <cstddef>
#include <cstring>
#include <flight_controller.h>

// TODO перенести в main.cpp после askForMissionApproval

int secureMissionUpdate(char* newMission) {
    char logBuffer[256] = {0};
    int approvalResult = 0;
    
    // 1. Проверка подписи
    uint8_t authenticity = 0;
    if (!checkSignature(newMission, authenticity) || !authenticity) {
        logEntry("Mission signature verification failed", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    
    // 2. Запрос подтверждения от сервера
    if (!askForMissionApproval(newMission, approvalResult)) {
        logEntry("Failed to get mission approval from server", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    
    if (!approvalResult) {
        logEntry("Server rejected mission update", ENTITY_NAME, LogLevel::LOG_WARNING);
        return 0;
    }
    
    // 3. Загрузка новой миссии
    if (!loadMission(newMission)) {
        logEntry("Failed to load approved mission", ENTITY_NAME, LogLevel::LOG_ERROR);
        return 0;
    }
    
    logEntry("Mission successfully updated with server approval", ENTITY_NAME, LogLevel::LOG_INFO);
    printMission();
    return 1;
}

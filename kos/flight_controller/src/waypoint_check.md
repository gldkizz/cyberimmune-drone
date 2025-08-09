```
    //If we get here, the drone is able to arm and start the mission
    //The flight is need to be controlled from now on

    // ********** ЗАМЕНА ПРОСТОГО ЦИКЛА НА ЗАЩИТУ МИССИИ ********** //
    char currentMission[4096] = {0};
    strncpy(currentMission, subscriptionBuffer, 4096); // Сохраняем исходную миссию
    
    while (true) {
        // 1. Проверка обновлений миссии
        if (receiveSubscription("api/fmission_kos/", subscriptionBuffer, 4096) && 
            strcmp(subscriptionBuffer, "")) {
            
            if (!secureMissionUpdate(subscriptionBuffer)) {
                // При неудачной проверке продолжаем текущую миссию
                logEntry("Mission update rejected - using current mission", ENTITY_NAME, LogLevel::LOG_WARNING);
                loadMission(currentMission);
            } else {
                // Обновляем текущую миссию при успешной проверке
                strncpy(currentMission, subscriptionBuffer, 4096);
                logEntry("Mission updated successfully", ENTITY_NAME, LogLevel::LOG_INFO);
            }
        }
        
        // 2. Проверка команд экстренной остановки (существующий функционал)
        if (receiveSubscription("api/flight_status/", subscriptionBuffer, 4096)) {
            // ... существующий код обработки ...
        }
        
        // 3. Короткая задержка для снижения нагрузки на CPU
        usleep(100000); // 100ms
    }
    // ********** КОНЕЦ ЗАМЕНЫ ********** //

    return EXIT_SUCCESS;
```
/**
 * \file
 * \~English \brief Declaration of wrapper methods that send IPC messages to PeripheryController component.
 * \~Russian \brief Объявление методов-оберток для отправки IPC-сообщений компоненту PeripheryController.
 */

#pragma once

#include <stdint.h>

/**
 * \~English Enables the buzzer. The buzzer will automatically turn off in 2 seconds.
 * \return Returns 1 on successful turn on, 0 otherwise.
 * \~Russian Включает зуммер. Зуммер автоматически отключится через 2 секунды.
 * \return Возвращает 1, если зуммер был успешно включен, иначе -- 0.
 */
int enableBuzzer();
/**
 * \~English Switches the power supply to the drone motors.
 * \param[in] enable Power supply mode. Motors are de-energized at 0, energized at 1.
 * \return Returns 1 on successful mode set, 0 otherwise.
 * \~Russian Переключает подачу питания на двигатели.
 * \param[in] enable Режим подачи питания. Двигатели обесточены при 0, запитаны -- при 1.
 * \return Возвращает 1, если режим был успешно установлен, иначе -- 0.
 */
int setKillSwitch(uint8_t enable);
/**
 * \~English Switches the power supply to the cargo drop motor.
 * \param[in] enable Power supply mode. Motor is de-energized at 0, energized at 1.
 * \return Returns 1 on successful mode set, 0 otherwise.
 * \~Russian Переключает подачу питания на мотор сброса груза.
 * \param[in] enable Режим подачи питания. Мотор обесточен при 0, запитан -- при 1.
 * \return Возвращает 1, если режим был успешно установлен, иначе -- 0.
 */
int setCargoLock(uint8_t enable);
/**
 * \~English Performs RFID tag scan for a short period of time.
 * \param[out] scanResult Result of scan: 1 if tag was read, 0 otherwise.
 * \return Returns 1 on successful attempt (no matter if tag was found or not), 0 otherwise.
 * \~Russian Производит поиск RFID-метки в течение короткого промежутка времени.
 * \param[out] scanResult Результат сканирования метки: 1, если метка была найдена, иначе -- 0.
 * \return Возвращает 1, если сканирование было проведено (неважно, была ли найдена метка), иначе -- 0.
 */
int scanRfid(uint8_t &scanResult);
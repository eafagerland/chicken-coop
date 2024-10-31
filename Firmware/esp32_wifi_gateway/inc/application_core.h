/**
 * @file application_core.h
 *
 * @date Okt 13, 2024
 * @author Erik
 * 
 * @brief Main Application for handling tasks.
 */

#ifndef APPLICATION_CORE_H_
#define APPLICATION_CORE_H_

// Project headers.
#include "inc/rtos/stm32_serial_task.h"
#include "inc/rtos/wifi_task.h"

/**
 * @brief Main Application task.
 */
class ApplicationCore
{

public:

    /**
     * @brief Initializes the core application task.
     * 
     * @remarks
     *  All tasks for the application will be created.
     *  Status of each operation is returned, if any tasks
     *  failed to be created the hardfaultHandler should be called.
     * 
     * @retval TaskResult::Success
     *  If the initialization was successful.
     * @retval TaskResult::TaskCreationError
     *  If the tasks creation failed.
     */
    TaskResult init();

private:

    /**
     * @brief Initalizes the Stm32 serialport.
     */
    SerialportResult initStm32Serialport();

    /**
     * @brief The STM32 serialport instance.
     * 
     * @remarks
     *  Bridges the serial communication between 
     *  the ESP32 and the STM32.
     *  This runs in its own thread. 
     */
    Stm32SerialportTask m_stm32Serialport;

    /**
     * @brief The WiFi instance for communicating with the ESP32 and the HMI client.
     */
    WifiTask m_wifi;
};

#endif // APPLICATION_CORE_H_
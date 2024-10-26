/**
 * @file applicationcore.h
 *
 * @date Okt 13, 2024
 * @author Erik
 * 
 * @brief Main Application for handling tasks.
 */

#ifndef APPLICATIONCORE_H_
#define APPLICATIONCORE_H_

// Project headers.
#include "inc/statictask.h"
#include "inc/serialport.h"

/**
 * @brief Main Application task.
 */
class ApplicationCore
{

public:

    /**
     * @brief Initializes the core application task.
     * 
     * @retval TaskResult::Success
     *  If the initialization was successful.
     * @retval TaskResult::TaskCreationError
     *  If the tasks creation failed.
     */
    TaskResult init();

private:

    /**
     * @brief
     *  Holds the serialport that bridges the serial communication between 
     *  the ESP32 and the STM32.
     *  This runs in its own thread. 
     */
    Serialport m_stm32Serialport;
};

#endif // APPLICATIONCORE_H_
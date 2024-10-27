/**
 * @file applicationcore.cpp
 *
 * @date Okt 13, 2024
 * @author Erik
 * 
 * @brief Main Application for handling tasks.
 */

// Own header
#include "inc/applicationcore.h"

// Project headers.
#include "inc/debug/debugutils.h"
#include "inc/applicationconfig.h"

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
TaskResult ApplicationCore::init()
{

// Initialize the debug serial task if current build is a DEBUG build.
#ifdef DEBUG
    if (initDebug())
    {
        Debug::out << "Successfully started debug task!" << Debug::endl;
    }
    else
    {
        // Debug serial task failed to be created, return error so hardfaultHandler can be called.
        return TaskResult::TaskCreationError;
    }
#endif

    // Initialize the STM32 serialport.
    SerialportResult result = initStm32Serialport();
    if (result == SerialportResult::Success)
    {
        Debug::out << "Successfully started STM32 serialport!" << Debug::endl;
    }
    else
    {
        Debug::out << "Failed to start STM32 serialport task! Error: " << Serialport::resultCodeToString(result) << Debug::endl;
        return TaskResult::InitializationError;
    }

    // All tasks were created successfully.
    return TaskResult::Success;
}

/**
 * @brief Initalizes the Stm32 serialport.
 */
SerialportResult ApplicationCore::initStm32Serialport()
{
    // Init the serial task.
    // This will create the serialport task and the task will
    // start listening on the serialport.
    return m_stm32Serialport.init(
        Stm32SerialportTaskConfig::taskPriority,
        Stm32SerialportTaskConfig::taskName,
        &Stm32SerialportTaskConfig::serialHandle,
        SerialportPinConfig(Stm32SerialportTaskConfig::rxPin, Stm32SerialportTaskConfig::txPin),
        Stm32SerialportTaskConfig::baudRate);
}
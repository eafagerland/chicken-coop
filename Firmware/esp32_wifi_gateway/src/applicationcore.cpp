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
#include "inc/debug_utils.h"

/**
 * @brief Holds the task and serialport configuration for the Stm32 serialport.
 */
namespace Stm32SerialportTaskConfig 
{

    /** @brief The priority of the STM32 serialport task. */
    constexpr auto taskPriority = 1;

    /** @brief The taskname of the STM32 serialport task. */
    constexpr const char* taskName = "serialTask";

    /** @brief The ESP32 serial handle to be used to communicate with the Stm32. */
    constexpr HardwareSerial& serialHandle = Serial1;

    /** @brief The UART receive pin used on the ESP32 that is connected to the STM32. */
    constexpr uint8_t rxPin = 4;

    /** @brief The UART transmit pin used on the ESP32 that is connected to the STM32. */
    constexpr uint8_t txPin = 5;

    /** @brief The serialport baudrate used on the Stm32 serialport. */
    constexpr unsigned long baudRate = 115200;
}

TaskResult ApplicationCore::init()
{
    // Start the debug serial if in debug mode.
#ifdef DEBUG_INTERFACE
    DEBUG_INTERFACE.begin(DEBUG_SERIAL_BAUDRATE);
#endif

    // Stores the result of the serialport init operation.
    SerialResult result;

    // Init the serial task.
    // This will create the serialport task and the task will
    // start listening on the serialport.
    result = m_stm32Serialport.init(
        Stm32SerialportTaskConfig::taskPriority,
        Stm32SerialportTaskConfig::taskName,
        &Stm32SerialportTaskConfig::serialHandle,
        SerialportPinConfig(Stm32SerialportTaskConfig::rxPin, Stm32SerialportTaskConfig::txPin),
        Stm32SerialportTaskConfig::baudRate);

    // Check the result of the Stm32 serialport initialization process.
    if (result != SerialResult::Success)
    {
        logln(Serialport::resultCodeToString(result));
        return TaskResult::InitializationError;
    }
    else
    {
        logln("Succcessfully started serialport!");
        return TaskResult::Success;
    }
}
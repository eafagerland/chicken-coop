/**
 * @file debug_utils.h
 *
 * @date Okt 25, 2024
 * @author Erik Fagerland
 * 
 * @brief Provides macros for an easier way to output debug data on the serialport.
 * 
 * @note 
 *  To disable debug log comment the "#define DEBUG" line.
 *  All log entries in the rest of the code will then be removed automatically.
 */

#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

// Project headers.
#include "inc/debug/debug_config.h"
#include "inc/debug/debug_serial_task.h"

/**
 * @brief Initializes the ESP32 debug port.
 * 
 */
static SerialportResult initDebug()
{

#ifdef DEBUG

    // Holds the configuration of the debug port.
    SerialportInstanceConfig debugTaskConfig;

    // Set the configuration struct.
    debugTaskConfig.baudrate = DebugSerialTaskConfig::baudrate;
    debugTaskConfig.handle = &DebugSerialTaskConfig::serialHandle;
    debugTaskConfig.rxPin = DebugSerialTaskConfig::rxPin;
    debugTaskConfig.txPin = DebugSerialTaskConfig::txPin;

    // Attempt to initialize the debug serial task.
    SerialportResult debugResult = Debug::out.init(DebugSerialTaskConfig::taskPriority, DebugSerialTaskConfig::taskName, debugTaskConfig);
    
    // Return the result of the operation.
    return debugResult;

#endif // DEBUG

    // Just return success if the DEBUG preprocessor was not defined.
    return SerialportResult::Success;

}

#endif // DEBUG_UTILS_H
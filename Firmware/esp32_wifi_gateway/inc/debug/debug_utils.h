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
 */
static bool initDebug()
{
    // Holds the configuration of the debug port.
    SerialportInstanceConfig debugTaskConfig;
    debugTaskConfig.baudrate = DebugSerialTaskConfig::baudrate;
    debugTaskConfig.handle = &DebugSerialTaskConfig::serialHandle;
    debugTaskConfig.rxPin = DebugSerialTaskConfig::rxPin;
    debugTaskConfig.txPin = DebugSerialTaskConfig::txPin;

    // Attempt to initialize the debug serial task.
    SerialportResult debugResult = Debug::out.init(DebugSerialTaskConfig::taskPriority, DebugSerialTaskConfig::taskName, debugTaskConfig);
    
    if (debugResult != SerialportResult::Success)
    {
        return false;
    }
    else
    {
        return true;
    }
}

#endif // DEBUG_UTILS_H
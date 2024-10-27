/**
 * @file debugutils.h
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

#ifndef DEBUGUTILS_H
#define DEBUGUTILS_H

// Project headers.
#include "inc/debug/debugconfig.h"
#include "inc/debug/debugserialtask.h"

/**
 * @brief Initializes the ESP32 debug port.
 */
static bool initDebug()
{
    Debug::DebugSerialTaskResult debugResult = Debug::out.init(2, "debugLogTask", DebugSerialTaskConfig::baudrate);
    
    if (debugResult != Debug::DebugSerialTaskResult::Success)
    {
        return false;
    }
    else
    {
        return true;
    }
}

#endif // DEBUGUTILS_H
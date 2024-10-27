/**
 * @file debugconfig.h
 *
 * @date Okt 27, 2024
 * @author Erik Fagerland
 * 
 * @brief Configuration file for the debug port.
 * 
 * @remarks
 * This class provides an overloaded << operator to facilitate easy and flexible debugging output.
 * It operates within a separate FreeRTOS task, ensuring that debug messages can be processed 
 * asynchronously without blocking the main application flow. The debugging functionality is 
 * only included and defined when the preprocessor directive #define DEBUG is present, allowing 
 * user to enable or disable debug logging easily at compile time.
 *
 * The class is designed to be thread-safe, allowing multiple tasks to write debug messages 
 * concurrently without causing data corruption. It utilizes the Serial debug port on the 
 * ESP32 RX Pin: 3, TX Pin: 2.
 *
 * This implementation ensures minimal impact on the performance of the application while 
 * providing robust debugging capabilities.
 */

#ifndef DEBUGCONFIG_H_
#define DEBUGCONFIG_H_

// Uncomment the following line for debug mode (comment for release).
#define DEBUG

namespace DebugSerialTaskConfig
{

/** @brief The stack size of the debug FreeRTOS task in bytes.  */
constexpr auto stackSize = (1024UL);

/** @brief The maximum number of received strings in queue. */
constexpr auto queueMaxEntries = (5U);

/** @brief The maximum number of bytes for one debug message. */
constexpr auto maxStringLength = (128U);

/** @brief The number of ticks to wait for the serialport starting before timing out. */
constexpr auto waitForSerialportStartTimeout = (1000U);

/** @brief The number of ticks to wait for debug task starting before timing out. */
constexpr auto waitForDebugTaskStartTimeout = (1000U);

/** @brief The notification array index of the initialization value. */
constexpr auto initNotificationArrayIndex = (0U);

/** @brief The time in milliseconds before timing out on receive queue operations. */
constexpr auto receiveQueueTimeout = (10U);

/** @brief The baudrate to run the debug UART on. */
constexpr auto baudrate = (115200UL);

}

#endif // DEBUGCONFIG_H_
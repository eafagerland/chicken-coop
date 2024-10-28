/**
 * @file debug_serial_task.h
 *
 * @date Okt 26, 2024
 * @author Erik Fagerland
 * 
 * @brief Provides a class for flexiable debug.
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

#ifndef DEBUG_SERIAL_H_
#define DEBUG_SERIAL_H_

// Standard C++ headers.
#include <iostream>
#include <sstream>

// Project headers.
#include "inc/serialport_task.h"
#include "inc/Debug/debug_config.h"

// Third party header.
#include <Arduino.h>

namespace Debug
{

/**
 * @brief Defines the end line char so that a new line can be added to the stream easier.
 */
constexpr char endl = '\n';

/**
 * @brief Holds the queue item that is used
 */
struct StringQueueItem
{

    /** @brief The string to be outputted on the debug port. */
    char string[DebugSerialTaskConfig::maxStringLength];

};

/**
 * @brief The Debug serialport task class.
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
class DebugSerialTask : public SerialportTask
{

public:

    /**
     * @ The DebugSerial Constructor
     */
    DebugSerialTask() : SerialportTask(m_stack, DebugSerialTaskConfig::stackSize){};

    /**
     * @brief Overrides the << operator to add values to the FreeRTOS queue.
     *
     * This operator allows appending various types of values (e.g., strings, integers, floats)
     * to the internal stringstream of the DebugSerial object. When the operator is used,
     * the current contents of the stringstream are sent to a FreeRTOS queue for further processing.
     * The stringstream is then cleared to prepare it for the next set of data to be logged.
     *
     * If debugging is disabled (i.e., the DEBUG macro is not defined), this operator will effectively
     * do nothing, allowing for easy toggling of debug print statements in the code without requiring
     * changes to the actual logging statements.
     *
     * @tparam T The type of the value being logged. Can be any type that supports streaming with std::stringstream.
     * @param obj Reference to the DebugSerial object that is being logged to.
     * @param value The value to be logged. This value will be appended to the stringstream.
     * @return DebugSerial& A reference to the DebugSerial object, allowing for chaining of the << operator.
     */
    template <typename T>
    friend DebugSerialTask& operator<<(DebugSerialTask& obj, const T& value) 
    {
#ifdef DEBUG
        // Sanity check that the debug task is initialized.
        if (!obj.m_isInitialized)
        {
            return obj;
        }

        // Append the value to the internal stringstream
        obj.ss << value;

        // Store the current content of the stringstream in the rtos queue.
        obj.addStringToQueue(obj.ss.str());

        // Clear the stringstream for future use
        obj.ss.str(""); // Set the string content to an empty string
        obj.ss.clear(); // Reset the state flags (e.g., eof, fail)
#endif

        return obj;  // Return obj to allow chaining
    }

    /**
     * @brief Attempts to initializes the debug serialport task.
     * 
     * @remarks
     *  During initialization the serialport task will be created,
     *  queue for receive/transmitting data will be created, and the serialport
     *  will start.
     * 
     *  Result of each of these operations are tracked and will return with a
     *  error code if not succeded.
     * 
     *  When creating the task it will block until the notification that
     *  the initialization is complete.
     * 
     *  Overriden to initialize the stream string queue that it used when debugging
     *  messages.
     * 
     * @retval SerialportResult::Success,
     *  If the operation was successful.
     * @retval SerialportResult::TaskCreationFailed
     *  If the task creation failed.
     * @retval SerialportResult::QueueCreationFailedError
     *  If the receive queue creation failed.
     * @retval SerialportResult::SerialportStartTimeoutError
     *  If the serialport instance was unable to start before timing out.
     * @retval SerialportResult::TaskStartFailedError
     *  If the serialport task failed to start and operation timed out.
     * @retval SerialportResult::AlreadyInitializedError
     *  If the serialport was already initialized.
     */
    SerialportResult init(
        UBaseType_t taskPriority,
        const char * const taskName,
        const SerialportInstanceConfig instanceConfig) override;

private:

    /**
     * @brief Private function for adding a std::string to the freertos queue.
     * 
     * @param[in] string The string to be added to the queue.
     */
    static void addStringToQueue(const std::string& string);

    /**
     * @brief Holds the queue that contains to string that will be outputted to the serialport.
     */
    static StaticQueue<DebugSerialTaskConfig::queueMaxEntries, sizeof(StringQueueItem)> m_stringQueue;

    /**
     * @brief A std::stringstream for storing the output.
     */
    static std::stringstream ss;

    /**
     * @brief Holds the initialization state.
     */
    static bool m_isInitialized;

    /**
     * @brief Holds the stack for the task.
     */
    StackType_t m_stack[DebugSerialTaskConfig::stackSize];

protected:

    /**
     * @brief Overriden task worker function.
     * 
     * @remarks
     *  Upon starting this function will block until the notification
     *  from the @ref init function has been received.
     * 
     *  When running it continuously check if there any debug items in the queue.
     *  When data is available it will attempt to retreive it from the queue and it will be
     *  printed out on the serialport handler.
     */
    void run() override;
};

/**
 * The DebugSerialTask static instance for external access.
 */
static DebugSerialTask out;

} // namespace Debug

#endif // DEBUG_SERIAL_H_
/**
 * @file debug_serial_task.cpp
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

// Own header
#include "inc/debug/debug_serial_task.h"

// Standard C++ libraries.
#include <cstring>

// Initialize the init state.
bool Debug::DebugSerialTask::m_isInitialized = false;

// Initialize the static stringstream
std::stringstream Debug::DebugSerialTask::ss;

// Initialize the string queue.
StaticQueue<DebugSerialTaskConfig::queueMaxEntries, sizeof(Debug::StringQueueItem)> Debug::DebugSerialTask::m_stringQueue;

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
SerialportResult Debug::DebugSerialTask::init(
    UBaseType_t taskPriority,
    const char * const taskName,
    const SerialportInstanceConfig instanceConfig)
{
    // Holds the result of the operation.
    SerialportResult result;

    // Create the string queue.
    m_stringQueue.handle = xQueueCreateStatic(
        DebugSerialTaskConfig::queueMaxEntries,
        sizeof(StringQueueItem),
        m_stringQueue.storage,
        &m_stringQueue.buffer);
    
    // Check if the queue creation was successfull.
    if (m_stringQueue.handle == NULL)
    {
        result = SerialportResult::QueueCreationFailedError;
        return result;
    }

    // Initialize and start the task.
    result = SerialportTask::init(taskPriority, taskName, instanceConfig);

    // Initialization was successful.
    if (result == SerialportResult::Success)
    {
        m_isInitialized = true;
    }

    return result;
}

/**
 * @brief Private function for adding a std::string to the freertos queue.
 * 
 * @param[in] string The string to be added to the queue.
 */
void Debug::DebugSerialTask::addStringToQueue(const std::string& string)
{
    // Verify that the debug task is initialized.
    if (!m_isInitialized)
    {
        return;
    }

    // Holds the queue item.
    StringQueueItem item;

    // Truncate and copy the string safely
    std::strncpy(item.string, string.c_str(), DebugSerialTaskConfig::maxStringLength - 1);

    // Ensure null-termination
    item.string[DebugSerialTaskConfig::maxStringLength - 1] = '\0';

    // Add item to queue.
    xQueueSend(m_stringQueue.handle, (void*)&item, DebugSerialTaskConfig::receiveQueueTimeout);
}

/**
 * @brief Overriden task worker function.
 * 
 * @remarks
 *  Upon starting this function will block until the notification
 *  from the @ref init function has been received.
 * 
 *  When running it continuously checks if there any debug items in the queue.
 *  When data is available it will attempt to retreive it from the queue and it will be
 *  printed out on the serialport handler.
 */
void Debug::DebugSerialTask::run()
{
    // Stores the notification value.
    uint32_t notificationValue;
    
    // Block the task until initialization in completed.
    // The task taking care of initialization must take care of this task if it fails.
    xTaskNotifyWait(pdFALSE, pdFALSE, &notificationValue, portMAX_DELAY);
    while (notificationValue & (SerialportTaskConfig::initNotificationBit) == 0)
    {
        xTaskNotifyWait(pdFALSE, ULONG_MAX, &notificationValue, portMAX_DELAY);       
    }

    // Task is ready to run.
    while(true)
    {
        StringQueueItem stringItem;

        // Check the string queue to see if there are any pending debug messages to output.
        BaseType_t queueResult = xQueueReceive(m_stringQueue.handle, (void*)&stringItem, portMAX_DELAY);

        if (queueResult == pdTRUE)
        {
            // Print string on debug port.
            m_instanceConfig.handle->print(stringItem.string);
        }
    }
}
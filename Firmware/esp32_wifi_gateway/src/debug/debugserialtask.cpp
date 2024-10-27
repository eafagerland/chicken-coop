/**
 * @file debugserialtask.cpp
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
#include "inc/debug/debugserialtask.h"

// Standard C++ libraries.
#include <cstring>

// Init the init state.
bool Debug::DebugSerialTask::m_isInitialized = false;

// Init the serial handle to debug port.
HardwareSerial *Debug::DebugSerialTask::m_serialHandle = &Serial;

// Init the task handle.
TaskHandle_t Debug::DebugSerialTask::xTaskToNotify = NULL;

// Initialize the static stringstream
std::stringstream Debug::DebugSerialTask::ss;

StaticQueue<DebugSerialTaskConfig::queueMaxEntries, sizeof(Debug::StringQueueItem)> Debug::DebugSerialTask::m_stringQueue;

/**
 * @brief Initializes the debug serial task.
 * 
 * @remarks
 *  During initialization the debug serial task will be created,
 *  queue for transmitting data will be created, and the debug serial port
 *  will start.
 * 
 *  Result of each of these operations are tracked and will return with a
 *  error code if not succeded.
 * 
 *  When creating the task it will block until the notification that
 *  the initialization is complete.
 * 
 *  @retval DebugSerialResult::Success,
 *   If the operation was successful.
 *  @retval DebugSerialResult::TaskCreationFailed
 *   If the task creation failed.
 *  @retval DebugSerialResult::QueueCreationFailedError
 *   If the receive queue creation failed.
 *  @retval DebugSerialResult::SerialportStartTimeoutError
 *   If the serialport instance was unable to start before timing out.
 *  @retval DebugSerialResult::TaskStartFailedError
 *   If the serialport task failed to start and operation timed out.
 */
Debug::DebugSerialTaskResult Debug::DebugSerialTask::init(
    UBaseType_t taskPriority,
    const char * const taskName,
    const uint32_t baudrate)
{
    // Prevent the initialization to be run several times.
    if (m_isInitialized)
    {
        return DebugSerialTaskResult::AlreadyInitializedError;
    }

    // Store the result for the operation.
    TaskResult taskResult;

    // Attempt to create the serialport task and return its status.
    taskResult = createTask(
        taskPriority, 
        taskName
    );

    // Check if the task was created successfully.
    if (taskResult != TaskResult::Success)
    { 
        return DebugSerialTaskResult::TaskCreationFailedError;
    }

    // Create the string queue.
    m_stringQueue.handle = xQueueCreateStatic(
        DebugSerialTaskConfig::queueMaxEntries,
        sizeof(StringQueueItem),
        m_stringQueue.storage,
        &m_stringQueue.buffer);
    
    // Check if the queue creation was successfull.
    if (m_stringQueue.handle == NULL)
    {
        return DebugSerialTaskResult::QueueCreationFailedError;
    }

    // Get the current tick when serial is started.
    TickType_t timeoutTickCount = xTaskGetTickCount();

    // Start the serialport instance.
    m_serialHandle->begin(baudrate);

    // Wait for serialport to start.
    while(!m_serialHandle)
    {
        // If serialport doesnt start before timing out it will return error.
        if (xTaskGetTickCount() - timeoutTickCount > DebugSerialTaskConfig::waitForSerialportStartTimeout)
        {
            return DebugSerialTaskResult::SerialportStartTimeoutError;
        }
    }

    // Init was successful.
    m_isInitialized = true;

    // Set the tick count to current time, for verififying that task is running.
    timeoutTickCount = xTaskGetTickCount();

    // Wait for task to start running.
    while (xTaskToNotify == NULL)
    {
        if (xTaskGetTickCount() - timeoutTickCount > DebugSerialTaskConfig::waitForDebugTaskStartTimeout)
        {
            // The debug serial task never started and operation timed out.
            m_isInitialized = false;
            return DebugSerialTaskResult::TaskStartFailedError;
        }
    }

    // Notify the task that the initialization was succesfull.
    xTaskNotifyGiveIndexed(xTaskToNotify, DebugSerialTaskConfig::initNotificationArrayIndex);

    // Return the result of the serialport initialization.
    return DebugSerialTaskResult::Success;
}

/**
 * @brief Private function for adding a std::string to the freertos queue.
 * 
 * @param[in] string The string to be added to the queue.
 */
void Debug::DebugSerialTask::addStringToQueue(const std::string& string)
{
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
    // Set the current task handle so that this task can be notified when the initalization is complete.
    xTaskToNotify = xTaskGetCurrentTaskHandle();

    // Wait for notification that the task has been initialized.
    ulTaskNotifyTakeIndexed(DebugSerialTaskConfig::initNotificationArrayIndex, pdFALSE, portMAX_DELAY);

    // Task has been initalized and so no task to notify.
    xTaskToNotify = NULL;

    // Task is ready to run.
    while(true)
    {
        // Check string queue.
        StringQueueItem stringItem;

        BaseType_t queueResult = xQueueReceive(m_stringQueue.handle, &stringItem, portMAX_DELAY);

        if (queueResult == pdTRUE)
        {
            m_serialHandle->print(stringItem.string);
        }
    }
}
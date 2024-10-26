/**
 * @file serialport.cpp
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Task for handling serialport data receiving and transmitting.
 */

// Own header.
#include "inc/serialport.h"

// Project headers.
#include "inc/debug_utils.h"

/**
 * @brief Initializes the serialport task.
 * 
 * @remarks
 *  During initialization the serialport task will be created,
 *  queue for received data will be created, and the serialport
 *  will start.
 * 
 *  Result of each of these operations are tracked and will return with a
 *  error code if not succeded.
 * 
 *  When creating the task it will block until the notification that
 *  the initialization is complete.
 * 
 *  @retval SerialResult::Success,
 *   If the operation was successful.
 *  @retval SerialResult::TaskCreationFailed
 *   If the task creation failed.
 *  @retval SerialResult::QueueCreationFailedError
 *   If the receive queue creation failed.
 *  @retval SerialResult::SerialportStartTimeoutError
 *   If the serialport instance was unable to start before timing out.
 *  @retval SerialResult::TaskStartFailedError
 *   If the serialport task failed to start and operation timed out.
 */
SerialResult Serialport::init(
    UBaseType_t taskPriority,
    const char * const taskName,
    HardwareSerial *serialHandle,
    const SerialportPinConfig pinConfig,
    const uint32_t baudrate
    )
{
    // Set the serial handle for this instance.
    m_serialHandle = serialHandle;

    // Store the pin configuration.
    m_pinConfig = pinConfig;

    // Store the port config.
    m_portConfig.baudrate = baudrate;

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
        return SerialResult::TaskCreationFailedError;
    }

    // Create the receive queue.
    m_receiveQueue.handle = xQueueCreateStatic(
        SerialportTaskConfig::queueMaxEntries,
        sizeof(SerialReceiveData),
        m_receiveQueue.storage,
        &m_receiveQueue.buffer);
    
    // Check if the queue creation was successfull.
    if (m_receiveQueue.handle == NULL)
    {
        return SerialResult::QueueCreationFailedError;
    }

    // Get the current tick when serial is started.
    TickType_t timeoutTickCount = xTaskGetTickCount();

    // Start the serialport instance.
    m_serialHandle->begin(m_portConfig.baudrate, SERIAL_8N1, m_pinConfig.rxPin, m_pinConfig.txPin);

    // Wait for serialport to start.
    while(!m_serialHandle)
    {
        // If serialport doesnt start before timing out it will return error.
        if (xTaskGetTickCount() - timeoutTickCount > SerialportTaskConfig::waitForSerialportStartTimeout)
        {
            return SerialResult::SerialportStartTimeoutError;
        }
    }

    // Init was successful.
    m_isInitialized = true;

    // Set the tick count to current time, for verififying that task is running.
    timeoutTickCount = xTaskGetTickCount();

    // Wait for task to start running.
    while (xTaskToNotify == NULL)
    {
        if (xTaskGetTickCount() - timeoutTickCount > SerialportTaskConfig::waitForserialportTaskStartTimeout)
        {
            // The serialport never started and operation timed out.
            return SerialResult::TaskStartFailedError;
        }
    }

    // Notify the task that the initialization was succesfull.
    xTaskNotifyGiveIndexed(xTaskToNotify, SerialportTaskConfig::initNotificationArrayIndex);

    // Return the result of the serialport initialization.
    return SerialResult::Success;
}

/**
 * @brief Overriden task worker function.
 * 
 * @remarks
 *  Upon starting this function will block until the notification
 *  from the @ref init function has been received.
 * 
 *  When running it continuously checks if there are data available on the 
 *  serialport. When data is received it will queue it up in the receive queue to
 *  be handled by other tasks.
 */
void Serialport::run()
{
    // Set the current task handle so that this task can be notified when the initalization is complete.
    xTaskToNotify = xTaskGetCurrentTaskHandle();

    // Wait for notification that the task has been initialized.
    ulTaskNotifyTakeIndexed(SerialportTaskConfig::initNotificationArrayIndex, pdFALSE, portMAX_DELAY);

    // Task has been initalized and so no task to notify.
    xTaskToNotify = NULL;

    // Task is ready to run.
    while(true)
    {
        // Stores the number of bytes read on serialport.
        uint16_t bytesRead = 0;

        // Check if data available on the serialport.
        SerialResult readResult = serialRead(bytesRead);

        // Print the result of the read operation if not successful.
        if (readResult != SerialResult::Success)
        {
            logln(resultCodeToString(readResult));
        }

        // For testing serial receive:
        // Data is sent manually with terminal connected to dev board.
        SerialReceiveData data;
        SerialResult queueResult = frontReceiveQueueItem(&data, 100);
        if (queueResult != SerialResult::Success && queueResult != SerialResult::ReceiveQueueTimeoutError)
        {
            logln(resultCodeToString(queueResult));
        }
        else
        {
            // Print the number of bytes received.
            if (data.length != 0)
            {
                log("Received Bytes: ");
                logln(data.length);
            }
        }
    } // while
}

/**
 * @brief Reads data on the serialport.
 * 
 * @remarks
 *  If there is data available on the serialport they will
 *  all be buffered up. When all the data has been buffered up
 *  and there is no data available it will be queued up in the
 *  receive queue. 
 * 
 * @param[out] bytesRead The number of bytes that were read.
 * 
 * @retval SerialResult::Success
 *  If the operation was successful.
 * @retval SerialResult::ReadTimeoutError
 *  If the read operation timed out.
 * @retval SerialResult::ReceiveBufferOverflowError
 *  If the read buffer encountered an overflow.
 * @retval SerialResult::ReceiveQueueFullError
 *  If the receive queue was full and read data could not be added.
 */
SerialResult Serialport::serialRead(uint16_t &bytesRead)
{
    // Initalize the bytes read output to 0.
    bytesRead = 0;

    // Holds the buffer for storing the incomming data.
    uint8_t serialReadBuffer[SerialportTaskConfig::readBufferSize];

    // Holds the number of bytes received.
    uint32_t serialReadBytes = 0;

    // Holds the start tick of the read operation to keep track of timeout.
    uint32_t readStartTick = xTaskGetTickCount();

    // Checks if there are data available on the serialport.
    while (m_serialHandle->available() > 0)
    {
        // Check for timeout
        uint32_t currentTick = xTaskGetTickCount();
        if (currentTick - readStartTick > SerialportTaskConfig::readTimeout)
        {
            return SerialResult::ReadTimeoutError;
        }

        // Store the byte into buffer at current index.
        serialReadBuffer[serialReadBytes] = m_serialHandle->read();

        // Increment the number of bytes read.
        serialReadBytes++;

        // Check for overflow.
        if (serialReadBytes == SerialportTaskConfig::readBufferSize)
        {
            return SerialResult::ReceiveBufferOverflowError;
        }
    }

    // Check if there was any data read on the serialport.
    if (serialReadBytes < 1)
    {
        // No data available on serialport.
        return SerialResult::Success;
    }

    // Create data structure of the received string.
    SerialReceiveData receivedData;
    receivedData.length = serialReadBytes;
    memcpy(receivedData.buffer, serialReadBuffer, serialReadBytes);

    // Attempt to queue the received data.
    BaseType_t queueResult = xQueueSend(m_receiveQueue.handle, (void*)&receivedData, SerialportTaskConfig::receiveQueueTimeout);
    if (queueResult != pdTRUE)
    {
        return SerialResult::ReceiveQueueFullError;
    }

    // Set the bytes read output.
    bytesRead = serialReadBytes;

    return SerialResult::Success;
}

/**
 * @brief Attempts to retreive the first item in the serialport receive queue.
 * 
 * @remarks
 *  This will block the calling thread until queue item is received,
 *  or a timeout event occurs.
 * 
 * @param[out] data The serial received data that is retreived from the queue.
 * @param[in] timeout The maximum time in millisecond for attempting to get queue item before timeout.
 * 
 * @retval SerialResult::Success
 *  If the operation was successful.
 * @retval SerialResult::InitializationError
 *  If the operation failed due to serialport not being initialized.
 * @retval SerialResult::ReceiveQueueTimeoutError
 *  If the receive queue operation timed out before able to retreive data.
 * @retval SerialResut::ArgumentNullError
 *  If the passed data argument was nullptr.
 */
SerialResult Serialport::frontReceiveQueueItem(SerialReceiveData *data, const uint32_t timeout)
{
    // Sanity check the data input.
    if (data == nullptr)
    {
        return SerialResult::ArgumentNullError;
    }

    // Check that the serialport is initialized.
    if (!m_isInitialized)
    {
        return SerialResult::InitializationError;
    }

    // Attempt to receive item from queue.
    BaseType_t queueResult = xQueueReceive(m_receiveQueue.handle, data, timeout);

    if (queueResult == pdTRUE)
    { 
        return SerialResult::Success;
    }

    return SerialResult::ReceiveQueueTimeoutError;
}

/**
 * @brief Converts the result code into a string.
 * 
 * @remarks
 *  Uses 'unordered_map' to create a hash table of the different result codes.
 * 
 * @param[in] code The code to be converted into a string.
 * 
 * @returns
 *  The string that best matches the provided code.
 */
const char* Serialport::resultCodeToString(const SerialResult code) 
{
    static const std::unordered_map<SerialResult, const char*> resultMap = 
    {
        { SerialResult::Success, "The serialport operation was successful!" },
        { SerialResult::ReceiveBufferOverflowError, "The serialports receive buffer encountered an overflow!" },
        { SerialResult::ReceiveQueueTimeoutError, "The serialports receive queue operation timed out!" },
        { SerialResult::ReceiveQueueFullError, "The serialports receive queue was full!" },
        { SerialResult::ReadTimeoutError, "The serialport read operation timed out!" },
        { SerialResult::ArgumentNullError, "The serialport operation failed due to a nullptr being passed as an argument!" },
        { SerialResult::InitializationError, "The serialport operation failed due to not being initialized!" },
        { SerialResult::TaskStartFailedError, "The serialport operation failed due starting of task timed out!"}
    };

    auto it = resultMap.find(code);
    return it != resultMap.end() ? it->second : "Serialport operation resulted in an unknown result code!";
}
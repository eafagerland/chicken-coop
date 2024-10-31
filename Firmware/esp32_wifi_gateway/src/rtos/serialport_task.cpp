// Own header.
#include "inc/rtos/serialport_task.h"

// Standard C++ headers.
#include <unordered_map>

/**
 * @brief Attempts to initializes the serialport task.
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
 *  Declared as virtual for a overriden init in derrived classes.
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
SerialportResult SerialportTask::init(
    UBaseType_t taskPriority,
    const char *const taskName,
    const SerialportInstanceConfig instanceConfig)
{
    // Prevent the initialization to be run several times.
    if (this->m_isInitialized)
    {
        return SerialportResult::AlreadyInitializedError;
    }

    // Store the serialport configuration.
    m_instanceConfig = instanceConfig;

    // Store the result for the operation.
    TaskResult taskResult;

    // Attempt to create the serialport task and return its status.
    taskResult = this->createTask(taskPriority, taskName);

    // Check if the task was created successfully.
    if (taskResult != TaskResult::Success)
    {
        return SerialportResult::TaskCreationFailedError;
    }

    // Create the receive queue.
    m_receiveQueue.handle = xQueueCreateStatic(
        SerialportTaskConfig::queueMaxEntries,
        sizeof(SerialQueueData),
        m_receiveQueue.storage,
        &m_receiveQueue.buffer);

    // Check if the queue creation was successfull.
    if (m_receiveQueue.handle == NULL)
    {
        return SerialportResult::QueueCreationFailedError;
    }

    // Create the transmit queue.
    m_transmitQueue.handle = xQueueCreateStatic(
        SerialportTaskConfig::queueMaxEntries,
        sizeof(SerialQueueData),
        m_receiveQueue.storage,
        &m_receiveQueue.buffer);

    // Check if the queue creation was successfull.
    if (m_receiveQueue.handle == NULL)
    {
        return SerialportResult::QueueCreationFailedError;
    }

    // Get the current tick when serial is started.
    TickType_t timeoutTickCount = xTaskGetTickCount();

    // Start the serialport instance.
    m_instanceConfig.handle->begin(m_instanceConfig.baudrate, SERIAL_8N1, m_instanceConfig.rxPin, m_instanceConfig.txPin);

    // Wait for serialport to start.
    while (!m_instanceConfig.handle)
    {
        // If serialport doesnt start before timing out it will return error.
        if (xTaskGetTickCount() - timeoutTickCount > SerialportTaskConfig::waitForSerialportStartTimeout)
        {
            return SerialportResult::SerialportStartTimeoutError;
        }
    }

    // Init was successful.
    this->m_isInitialized = true;

    // Set the tick count to current time, for verififying that task is running.
    timeoutTickCount = xTaskGetTickCount();

    // Notify the task that the initialization was succesfull.
    xTaskNotify(this->m_taskHandle, SerialportTaskConfig::initNotificationBit, eSetBits);

    // Return the result of the serialport initialization.
    return SerialportResult::Success;
}

/**
 * @brief Proccesses the receive or transmit queue.
 *
 * @remarks
 *  Attempts to retreive an item from receive queue or set data in transmit queue.
 *  Depending on the queue type that is provided.
 *
 *  This will block the calling task until a queue item has been received/set or a timeout occurs.
 *
 *  When the calling task has has sent data to transmit queue it will notify the task that is sending the data
 *  that there is data available to be transmitted.
 *
 * @param data The pointer where the received queue data should be stored or the data to be sent to transmit queue.
 * @param[in] queueType The queue type to indicate receive or transmit queue.
 * @param[in] timeout The time in millisecond for attempting to receive item from queue before timeout.
 *
 * @retval SerialportResult::Success
 *  If the operation was successful and item was received from queue.
 * @retval SerialportResult::ArgumentNullError
 *  If the passed data pointer was nullptr.
 * @retval SerialportResult::InitializationError
 *  If the serialport was not initialized correctly.
 * @retval SerialportResult::InvalidQueueTypeError
 *  If the provided queue type was invalid.
 */
SerialportResult SerialportTask::processQueueItem(SerialQueueData *data, const SerialQueueType queueType, const uint32_t timeout)
{
    // Sanity check the data input.
    if (data == nullptr)
    {
        return SerialportResult::ArgumentNullError;
    }

    // Check that the serialport is initialized.
    if (!this->m_isInitialized)
    {
        return SerialportResult::InitializationError;
    }

    // Store the result of the operation,
    // initialize it to success so it just needs to be set to an error if present.
    SerialportResult serialResult = SerialportResult::Success;

    // Holds the queue result.
    BaseType_t queueResult;

    // Process the queue type that was selected.
    switch (queueType)
    {
    case SerialQueueType::Receive:
    {
        // Attempt to receive item from queue.
        queueResult = xQueueReceive(m_receiveQueue.handle, data, timeout);
        break;
    }
    case SerialQueueType::Transmit:
    {
        // Attempt to receive item from queue.
        queueResult = xQueueSend(m_receiveQueue.handle, data, timeout);
        break;
    }
    default:
    {
        // The queue type provided did not match any valid queue.
        serialResult = SerialportResult::InvalidQueueTypeError;
        break;
    }
    }

    if (queueResult != pdTRUE)
    {
        serialResult = SerialportResult::ReceiveQueueTimeoutError;
    }

    return serialResult;
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
 * @retval SerialportResult::Success
 *  If the operation was successful.
 * @retval SerialportResult::ReadTimeoutError
 *  If the read operation timed out.
 * @retval SerialportResult::ReceiveBufferOverflowError
 *  If the read buffer encountered an overflow.
 * @retval SerialportResult::ReceiveQueueFullError
 *  If the receive queue was full and read data could not be added.
 */
SerialportResult SerialportTask::serialRead(uint16_t &bytesRead, const uint32_t timeout)
{
    // Initalize the bytes read output to 0.
    bytesRead = 0;

    // Holds the buffer for storing the incomming data.
    uint8_t serialReadBuffer[SerialportTaskConfig::queueItemBufferMaxSize];

    // Holds the number of bytes received.
    uint32_t serialReadBytes = 0;

    // Holds the start tick of the read operation to keep track of timeout.
    uint32_t readStartTick = xTaskGetTickCount();

    // Checks if there are data available on the serialport.
    while (m_instanceConfig.handle->available() > 0)
    {
        // Check for timeout
        uint32_t currentTick = xTaskGetTickCount();
        if (currentTick - readStartTick > timeout)
        {
            return SerialportResult::ReadTimeoutError;
        }

        // Store the byte into buffer at current index.
        serialReadBuffer[serialReadBytes] = m_instanceConfig.handle->read();

        // Increment the number of bytes read.
        serialReadBytes++;

        // Check for overflow.
        if (serialReadBytes == SerialportTaskConfig::queueItemBufferMaxSize)
        {
            return SerialportResult::ReceiveBufferOverflowError;
        }
    }

    // Check if there was any data read on the serialport.
    if (serialReadBytes < 1)
    {
        // No data available on serialport.
        return SerialportResult::Success;
    }

    // Create data structure of the received string.
    SerialQueueData receivedData;
    receivedData.length = serialReadBytes;
    memcpy(receivedData.buffer, serialReadBuffer, serialReadBytes);

    // Attempt to queue the received data.
    BaseType_t queueResult = xQueueSend(m_receiveQueue.handle, (void *)&receivedData, pdTICKS_TO_MS(SerialportTaskConfig::receiveQueueTimeout));
    if (queueResult != pdTRUE)
    {
        return SerialportResult::ReceiveQueueFullError;
    }

    // Set the bytes read output.
    bytesRead = serialReadBytes;

    return SerialportResult::Success;
}

/**
 * @brief Write data on the serialport.
 *
 * @remarks
 *  If there is data available in the transmit queue it will attempt
 *  to transmit it on the serialport.
 *
 * @param[out] bytesTransmitted The number of bytes transmitted.
 *
 * @retval SerialportResult::Success
 *  If the write operation was success.
 * @retval SerialportResult::ReceiveQueueTimeoutError
 *  If the operation was unable to retreive item from queue.
 * @retval SerialportResult::TransmitQueueItemEmptyError
 *  If the received item from queue had empty 0 length.
 * @retval SerialportResult::WriteOperationFailedError
 *  If the write operation was not able to write all data.
 */
SerialportResult SerialportTask::serialWrite(uint16_t &bytesTransmitted, const TickType_t timeout)
{
    // Initalize the bytes transmitted output to 0.
    bytesTransmitted = 0;

    // Holds the data to be received from the transmit queue.
    SerialQueueData transmitData;

    // Attempt to get the queue item from the queue.
    BaseType_t queueResult = xQueueReceive(m_transmitQueue.handle, (void *)&transmitData, pdTICKS_TO_MS(timeout));

    if (queueResult != pdTRUE)
    {
        return SerialportResult::ReceiveQueueTimeoutError;
    }

    // Check that there is data in the item received.
    if (transmitData.length < 1)
    {
        return SerialportResult::TransmitQueueItemEmptyError;
    }

    // Write the data on the serialport and store the number of bytes sent.
    bytesTransmitted = m_instanceConfig.handle->write(transmitData.buffer, transmitData.length);

    // Check if all butes were transmitted.
    if (bytesTransmitted != transmitData.length)
    {
        SerialportResult::WriteOperationFailedError;
    }

    // The write operation was successful.
    return SerialportResult::Success;
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
const char *SerialportTask::resultCodeToString(const SerialportResult code)
{
    static const std::unordered_map<SerialportResult, const char *> resultMap =
        {
            {SerialportResult::Success, "The serialport operation was successful!"},
            {SerialportResult::ReceiveBufferOverflowError, "The serialports receive buffer encountered an overflow!"},
            {SerialportResult::ReceiveQueueTimeoutError, "The serialports receive queue operation timed out!"},
            {SerialportResult::ReceiveQueueFullError, "The serialports receive queue was full!"},
            {SerialportResult::ReadTimeoutError, "The serialport read operation timed out!"},
            {SerialportResult::ArgumentNullError, "The serialport operation failed due to a nullptr being passed as an argument!"},
            {SerialportResult::InitializationError, "The serialport operation failed due to not being initialized!"},
            {SerialportResult::TaskStartFailedError, "The serialport operation failed due starting of task timed out!"},
            {SerialportResult::AlreadyInitializedError, "The serialport init operation failed because it was already initialzied!"},
            {SerialportResult::TaskHandleNullError, "The serialport operation failed beacuse the task handle was NULL!"},
            {SerialportResult::QueueHandleNullError, "The serialport operation failed because the receive queue handle was NULL!"},
            {SerialportResult::InvalidQueueTypeError, "The serialport operation failed because an invalid queue type was handled!"},
            {SerialportResult::TransmitQueueTimeoutError, "The serialport transmit queue operation timed out!"},
            {SerialportResult::WriteOperationFailedError, "The serialport write operation was not able to write all data!"},
            {SerialportResult::TransmitQueueItemEmptyError, "The serialport write operation failed because queue item was empty!"}
        };

    auto it = resultMap.find(code);
    return it != resultMap.end() ? it->second : "Serialport operation resulted in an unknown result code!";
}
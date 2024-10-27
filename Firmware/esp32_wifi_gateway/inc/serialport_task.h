#ifndef SERIALPORT_TASK_H_
#define SERIALPORT_TASK_H_

// Project headers.
#include "inc/static_task.h"

/**
 * @brief Holds the configuration of the serialport task.
 */
namespace SerialportTaskConfig
{

/** @brief The maximum number of received strings in queue. */
constexpr auto queueMaxEntries = (10U);

/** @brief The maximum number of bytes the queue item buffer can hold. */
constexpr auto queueItemBufferMaxSize = (128U);

/** @brief  The maximum amount of ticks before serial read operation times out. */
constexpr auto readTimeout = (100UL);

/** @brief The time in milliseconds before timing out on receive queue operations. */
constexpr auto receiveQueueTimeout = (100UL);

/** @brief The number of ticks to wait on serialport starting before timing out. */
constexpr auto waitForSerialportStartTimeout = (1000U);

/** @brief The notification bit position of the initialization value. */
constexpr auto initNotificationBit = (0U);

/** @brief The notification bit position of ready transmit, i.e there is data available in transmit queue. */
constexpr auto readyTransmitNotificationBit = (1 << 1U);

/** @brief The notification bit position of ready send, i.e there is data available in transmit queue. */
constexpr auto readyReceiveNotificationBit = (1 << 2U);

/** @brief The number of ticks to wait for serialport task to start running. */
constexpr auto waitForSerialportTaskStartTimeout = (1000U);

} // namespace SerialportTaskConfig

/**
 * @brief Represents the different outcomes of the serialport operations.
 */
enum class SerialportResult
{

    /** @brief Indicates that the operation was successful. */
    Success = 0,

    /** @brief Indicates that the serial read operation timed out. */
    ReadTimeoutError,

    /** @brief Indicates that the queue was full. */
    ReceiveQueueFullError,

    /** @brief Indicates that the receive queue operation failed due to timeout. */
    ReceiveQueueTimeoutError,

    /** @brief Indicates that the receive buffered encountered an overflow. */
    ReceiveBufferOverflowError,

    /** @brief Indicates that the transmit queue operation failed due to timeout. */
    TransmitQueueTimeoutError,

    /** @brief Indicates that a passed argument was nullptr. */
    ArgumentNullError,

    /** @brief Indicates that the operation failed due to initialization error. */
    InitializationError,

    /** @brief Indicates that a queue creation failed. */
    QueueCreationFailedError,

    /** @brief Indicates that the task creation failed. */
    TaskCreationFailedError,

    /** @brief Indicates that the serialport instance failed to start. */
    SerialportStartTimeoutError,

    /** @brief Indicates that the serialport task failed to start. */
    TaskStartFailedError,

    /** @brief Indicates that the serialport was attempted to be initalized when it already was. */
    AlreadyInitializedError,

    /** @brief Indicates that the task handle was NULL. */
    TaskHandleNullError,

    /** @brief Indicates that the queue handle was NULL. */
    QueueHandleNullError,

    /** @brief Indicates that an invalid queue type was handled. */
    InvalidQueueTypeError,

    /** @brief Indicates that there was an error while transmitting data on the serialport. */
    WriteOperationFailedError,

    /** @brief Indicates that the received data from transmit queue was empty. */
    TransmitQueueItemEmptyError,

};

/**
 * @brief Represents the different queue types of the serialport.
 */
enum class SerialQueueType
{
    
    /** @brief Indicates that it is the receive data queue. */
    Receive = 0,

    /** @brief Indicates that it is the transmit data queue. */
    Transmit,

};

/**
 * @brief Holds the data buffer and length of a received string.
 */
struct SerialQueueData
{

    /** @brief The length in bytes of the array. */
    uint16_t length;

    /** @brief Buffer containing the data. */
    uint8_t buffer[SerialportTaskConfig::queueItemBufferMaxSize];

    /**
     * @brief A default constructor for initializing the members to 0.
     */
    SerialQueueData()
    {
        length = 0;
        memset(buffer, 0, SerialportTaskConfig::queueItemBufferMaxSize);
    }

};

/**
 * @brief Holds the serialport configuration.
 */
struct SerialportInstanceConfig
{

    /** @brief Holds the serial handle. */
    HardwareSerial *handle;

    /** @brief Holds the UART receive pin number. */
    uint8_t rxPin;

    /** @brief Holds the UART transmit pin number. */
    uint8_t txPin;

    /** @brief Holds the boadrate of the serialport. */
    uint32_t baudrate;

    /**
     * @brief A default constructor initalizing members to 0.
     */
    SerialportInstanceConfig()
    {
        handle = nullptr;
        rxPin = 0;
        txPin = 0;
        baudrate = 0;
    }

    /**
     * @brief A overloaded constructor for initalizing the private members.
     */
    SerialportInstanceConfig (uint8_t rxPin, uint8_t txPin)
    {
        this->rxPin = rxPin;
        this->txPin = txPin;
    }

};

/**
 * @brief Static Serialport task class.
 */
template <size_t StackSize>
class SerialportTask : public StaticTask<StackSize>
{

public:

    /**
     * @brief The SerialportTask constructor.
     */
    SerialportTask() : StaticTask<StackSize>(){}

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
    virtual SerialportResult init(
        UBaseType_t taskPriority,
        const char * const taskName,
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
        while(!m_instanceConfig.handle)
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
    SerialportResult processQueueItem(SerialQueueData *data, const SerialQueueType queueType, const uint32_t timeout)
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
    static const char* resultCodeToString(const SerialportResult code) 
    {
        static const std::unordered_map<SerialportResult, const char*> resultMap = 
        {
            { SerialportResult::Success, "The serialport operation was successful!" },
            { SerialportResult::ReceiveBufferOverflowError, "The serialports receive buffer encountered an overflow!" },
            { SerialportResult::ReceiveQueueTimeoutError, "The serialports receive queue operation timed out!" },
            { SerialportResult::ReceiveQueueFullError, "The serialports receive queue was full!" },
            { SerialportResult::ReadTimeoutError, "The serialport read operation timed out!" },
            { SerialportResult::ArgumentNullError, "The serialport operation failed due to a nullptr being passed as an argument!" },
            { SerialportResult::InitializationError, "The serialport operation failed due to not being initialized!" },
            { SerialportResult::TaskStartFailedError, "The serialport operation failed due starting of task timed out!"},
            { SerialportResult::AlreadyInitializedError, "The serialport init operation failed because it was already initialzied!"},
            { SerialportResult::TaskHandleNullError, "The serialport operation failed beacuse the task handle was NULL!"},
            { SerialportResult::QueueHandleNullError, "The serialport operation failed because the receive queue handle was NULL!"},
            { SerialportResult::InvalidQueueTypeError, "The serialport operation failed because an invalid queue type was handled!"},
            { SerialportResult::TransmitQueueTimeoutError, "The serialport transmit queue operation timed out!" },
            { SerialportResult::WriteOperationFailedError, "The serialport write operation was not able to write all data!"},
            { SerialportResult::TransmitQueueItemEmptyError, "The serialport write operation failed because queue item was empty!"}
    };

    auto it = resultMap.find(code);
    return it != resultMap.end() ? it->second : "Serialport operation resulted in an unknown result code!";

    }

protected:

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
    virtual void run() override;

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
    SerialportResult serialRead(uint16_t &bytesRead, const uint32_t timeout)
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
        BaseType_t queueResult = xQueueSend(m_receiveQueue.handle, (void*)&receivedData, pdTICKS_TO_MS(SerialportTaskConfig::receiveQueueTimeout));
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
    SerialportResult serialWrite(uint16_t &bytesTransmitted, const TickType_t timeout)
    {
        // Initalize the bytes transmitted output to 0.
        bytesTransmitted = 0;

        // Holds the data to be received from the transmit queue.
        SerialQueueData transmitData;

        // Attempt to get the queue item from the queue.
        BaseType_t queueResult = xQueueReceive(m_transmitQueue.handle, (void*)&transmitData, pdTICKS_TO_MS(timeout));

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
     * @brief Holds the serialport configuration for this instance.
     */
    SerialportInstanceConfig m_instanceConfig;

private:

    /**
     * @brief Holds the static queue that is used for received data on the serialport.
     */
    StaticQueue<SerialportTaskConfig::queueMaxEntries, sizeof(SerialQueueData)> m_receiveQueue;

    /**
     * @brief Holds the static queue that is used for transmitting data on the serialport.
     */
    StaticQueue<SerialportTaskConfig::queueMaxEntries, sizeof(SerialQueueData)> m_transmitQueue;
};

#endif // SERIALPORT_TASK_H_
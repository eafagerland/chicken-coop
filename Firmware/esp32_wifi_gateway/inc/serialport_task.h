/**
 * @file serialport_task.h
 *
 * @date Okt 26, 2024
 * @author Erik Fagerland
 * 
 * @brief A Class for implementing a serialport RTOS task.
 * 
 * @remarks
 *  
 */

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
class SerialportTask : public StaticTask
{

public:

    /**
     * @brief The SerialportTask constructor.
     */
    SerialportTask(StackType_t *stackPointer, const uint32_t stackSize) : StaticTask(stackPointer, stackSize){}

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
        const SerialportInstanceConfig instanceConfig);

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
    SerialportResult processQueueItem(SerialQueueData *data, const SerialQueueType queueType, const uint32_t timeout);

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
    static const char* resultCodeToString(const SerialportResult code);

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
    SerialportResult serialRead(uint16_t &bytesRead, const uint32_t timeout);

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
    SerialportResult serialWrite(uint16_t &bytesTransmitted, const TickType_t timeout);

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
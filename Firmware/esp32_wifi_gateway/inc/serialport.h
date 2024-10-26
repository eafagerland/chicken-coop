/**
 * @file serialport.h
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Task for handling serialport data receiving and transmitting.
 */

#ifndef SERIALPORT_H_
#define SERIALPORT_H_

// Project headers.
#include "inc/statictask.h"

/**
 * @brief Holds the configuration of the serialport task.
 */
namespace SerialportTaskConfig
{

    /** @brief The stack size of the FreeRTOS application core in bytes.  */
    constexpr auto stackSize = (2048UL);

    /** @brief The maximum number of received strings in queue. */
    constexpr auto queueMaxEntries = (10U);

    /** @brief The maximum number of bytes received in one messagge. */
    constexpr auto readBufferSize = (128U);

    /** @brief  The maximum amount of ticks before serial read operation times out. */
    constexpr auto readTimeout = (100UL);

    /** @brief The time in milliseconds before timing out on receive queue operations. */
    constexpr auto receiveQueueTimeout = (100UL);

    /** @brief The number of ticks to wait on serialport starting before timing out. */
    constexpr auto waitForSerialportStartTimeout = (1000U);

    /** @brief The notification array index of the initialization value. */
    constexpr auto initNotificationArrayIndex = (0U);

    /** @brief The number of ticks to wait for serialport task to start running. */
    constexpr auto waitForserialportTaskStartTimeout = (1000U);

}

/** @brief Holds the different statuses of the serial operations. */
enum class SerialResult
{

    /** @brief Indicates that the operation was successful. */
    Success,

    /** @brief Indicates that the serial read operation timed out. */
    ReadTimeoutError,

    /** @brief Indicates that the queue was full. */
    ReceiveQueueFullError,

    /** @brief Indicates that the receive queue operation failed due to timeout. */
    ReceiveQueueTimeoutError,

    /** @brief Indicates that the receive buffered encountered an overflow. */
    ReceiveBufferOverflowError,

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
};

/**
 * @brief Holds the data buffer and length of a received string.
 */
struct SerialReceiveData
{

    /** @brief The length in bytes of the string. */
    uint16_t length;

    /** @brief buffer containing the string data. */
    uint8_t buffer[SerialportTaskConfig::readBufferSize];

    /**
     * @brief A default constructor for initializing the members to 0.
     */
    SerialReceiveData()
    {
        length = 0;
        memset(buffer, 0, SerialportTaskConfig::readBufferSize);
    }

};

/**
 * @brief Holds the serialport pin configuration.
 */
struct SerialportPinConfig
{

    /** @brief Holds the UART receive pin. */
    uint8_t rxPin;

    /** @brief Holds the UART transmit pin. */
    uint8_t txPin;

    /**
     * @brief A default constructor initalizing members to 0.
     */
    SerialportPinConfig()
    {
        rxPin = 0;
        txPin = 0;
    }

    /**
     * @brief A overloaded constructor for initalizing the private members.
     */
    SerialportPinConfig (uint8_t rxPin, uint8_t txPin)
    {
        this->rxPin = rxPin;
        this->txPin = txPin;
    }

};

/**
 * @brief Holds the serialport configuration.
 */
struct SerialportConfig
{
    uint32_t baudrate;
};

/**
 * @brief Static task for handling serialport (uart).
 */
class Serialport : public StaticTask<SerialportTaskConfig::stackSize>
{

public:

    /**
     * @brief The Serialport constructor.
     */
    Serialport() : StaticTask(){};

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
    SerialResult init(
        UBaseType_t taskPriority,
        const char * const taskName,
        HardwareSerial *serialHandle,
        const SerialportPinConfig pinConfig,
        const uint32_t baudrate
    );

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
    SerialResult frontReceiveQueueItem(SerialReceiveData *data, const uint32_t timeout);

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
    static const char *resultCodeToString(const SerialResult code);

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
    void run() override;

private:

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
    SerialResult serialRead(uint16_t &bytesRead);

    /**
     * @brief Holds the static queue that is used for received data on the serialport.
     */
    StaticQueue<SerialportTaskConfig::queueMaxEntries, sizeof(SerialReceiveData)> m_receiveQueue;

    /**
     * @brief Stores the pin configuration for this instance.
     */
    SerialportPinConfig m_pinConfig;

    /**
     * @brief Stores the serialport configuration for this instance.
     */
    SerialportConfig m_portConfig;

    /**
     * @brief Holds the serial handle.
     */
    HardwareSerial *m_serialHandle;

    /**
     * @brief Stores the handle for the task to receive notification.
     * 
     * @remarks
     *  This handle is used in order to syncronize the start-up sequence.
     *  So that the task will only start running after initialization is complete.
     */
    TaskHandle_t xTaskToNotify;
};

#endif // SERIALPORT_H_
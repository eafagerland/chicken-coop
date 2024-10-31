/**
 * @file stm32_serial_task.h
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Task for handling the communication with the STM32 MCU.
 * 
 * @remarks
 *  Reads the serialport connected to STM32 for received data. On valid reception of data 
 *  it will be added to WiFi data queue to be handled further.
 * 
 *  Writing data to the STM32 is also possible by adding data to the write queue with @ref processQueueItem function. 
 */

#ifndef STM32_SERIAL_TASK_H_
#define STM32_SERIAL_TASK_H_

// Project headers.
#include "inc/rtos/serialport_task.h"
#include "inc/application_config.h"
#include "inc/devices/stm32/esp_stm_comm_definitions.h"
#include "inc/rtos/wifi_task.h"

/** @brief Represents the minimum valid packet size received from the STM32. */
constexpr auto STM32_SERIAL_MINIMUM_PACKET_SIZE = 5U;

/** @brief Represents the payload start index of the received packet from the STM32. */
constexpr auto STM32_PAYLOAD_START_INDEX = 4U;

/** @brief Represents the first sync byte index of the received packet. */
constexpr auto STM32_FIRST_SYNC_BYTE_INDEX = 0U;

/** @brief Represents the second sync byte index of the received packet. */
constexpr auto STM32_SECOND_SYNC_BYTE_INDEX = 1U;

/** @brief Represents the start index of the payload length of the received packet. */
constexpr auto STM32_PAYLOAD_LENGTH_START_INDEX = 2U;

/**
 * @brief Task for handling the communication with the STM32 MCU.
 */
class Stm32SerialportTask : public SerialportTask
{

public:

    /**
     * @brief Stm32SerialPort Constructor.
     */
    Stm32SerialportTask() : 
        SerialportTask(m_stack, Stm32SerialportTaskConfig::stackSize), // Initliaze the serialport task.
        m_wifi(nullptr) // Initialize the pointer to the WiFi task.
        {};

    /**
     * @brief Sets the wifi handler for the stm32 task.
     */
    void setWifiHandler(WifiTask *wifi);

    /**
     * @brief Calculates a 1 byte sum-of-bytes checksum.
     * 
     * @param[in] data The data pointer containing the data to calculate the checksum of.
     * @param[in] length The length of the data pointer.
     * 
     * @returns
     *  The calculated 1-byte checksum.
     */
    static uint8_t calculateChecksum(uint8_t *data, uint16_t length);

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
    static const char *resultCodeToString(const Stm32SerialResult code);

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
     * 
     *  It also checks the notification flag is set for transmitting data.
     *  If the flag is set it will retreive the data from the transmitt queue and transmit
     *  data on the serialport.
     */
    void run() override;

private:

    /**
     * @brief Parses the received data from the serialport.
     * 
     * @remarks
     *  During parsing it will validate the received message to confirm that it is valid.
     *  In order to validate the message these methods are used:
     *   - Sync bytes valid.
     *   - Message size valid.
     *   - Payload length valid.
     *   - Checksum valid.
     * 
     * @param[in] data The received message from the serialport.
     * @param[out] packet The received message parsed from the received data.
     * 
     * @retval Stm32SerialResult::Success
     *  If the operation was successfull.
     * @retval Stm32SerialResult::ReceivedPacketSizeError
     *  If the received packet size was not what was expected.
     * @retval Stm32SerialResult::SyncByteError
     *  If the sync bytes in the message was not valid.
     * @retval Stm32SerialResult::PayloadEmptyError
     *  If the payload length was zero.
     * @retval Stm32SerialResult::ChecksumError;
     *  If the checksum in the received message was invalid.
     */
    Stm32SerialResult parseReceivedData(SerialQueueData data, Stm32SerialDataPacket *packet);

    /**
     * @brief Transmits the status to the STM32.
     * 
     * @remarks
     *  When the ESP32 receives a message from the STM32 it will
     *  verify the message. The status is then sent back to the STM32.
     * 
     * @param[in] parseResult The result from the receive data operation.
     * 
     * @retval Stm32SerialResult::Success
     *  If the operation was successful.
     * @retval Stm32SerialResult::WriteFailedError
     *  If the write operation failed.
     * @retval Stm32SerialResult::WriteSizeIncorrectError
     *  If the number of bytes written was incorrect.
     */
    Stm32SerialResult transmitResponse(const Stm32SerialResult parseResult);

    /**
     * @brief Holds the stack for the task.
     */
    StackType_t m_stack[Stm32SerialportTaskConfig::stackSize];

    /**
     * @brief Holds the pointer to the wifi task for setting data in the wifi outgoing queue.
     */
    WifiTask *m_wifi;
};

#endif // STM32_SERIAL_TASK_H_
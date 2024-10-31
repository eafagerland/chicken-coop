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

// Own header.
#include "inc/rtos/stm32_serial_task.h"

// Project headers.
#include "inc/debug/debug_utils.h"

// Standard C++ headers.
#include <unordered_map>

/** @brief Holds the timeout value in milliseconds during write operations before timeout. */
constexpr auto WRITE_TIMEOUT = 100U;

/**
 * @brief Sets the wifi handler for the stm32 task.
 */
void Stm32SerialportTask::setWifiHandler(WifiTask *wifi)
{
    m_wifi = wifi;
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
 *
 *  It also checks the notification flag is set for transmitting data.
 *  If the flag is set it will retreive the data from the transmitt queue and transmit
 *  data on the serialport.
 */
void Stm32SerialportTask::run()
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
        // Stores the number of bytes read on serialport.
        uint16_t bytesRead = 0;

        // Check if data available on the serialport.
        SerialportResult readResult = serialRead(bytesRead, 10);

        // Print the result of the read operation if not successful.
        if (readResult != SerialportResult::Success)
        {
            Debug::out << "Failed to read serialport: " << SerialportTask::resultCodeToString(readResult) << Debug::endl;
        }

        // Should be moved to WiFi tasked!
        SerialQueueData data;
        SerialportResult queueResult = processQueueItem(&data, SerialQueueType::Receive, 10);

        // Check if 
        if (queueResult != SerialportResult::Success && queueResult != SerialportResult::ReceiveQueueTimeoutError)
        {
            Debug::out << "Failed to receive from queue: " << SerialportTask::resultCodeToString(queueResult) << Debug::endl;
        }
        else
        {
            // With the received data length being more than 0 means data was successfully received on the serialport.
            if (data.length != 0)
            {
                Debug::out << "Received bytes: " << data.length << Debug::endl;

                // Stores the parsed data packet from received data.
                Stm32SerialDataPacket receivedPacket;

                // Parse the data.
                Stm32SerialResult parseResult = parseReceivedData(data, &receivedPacket);

                if (parseResult == Stm32SerialResult::Success)
                {
                    // Verify that the Wifi handler is valid.
                    if (m_wifi == nullptr)
                    {
                        Debug::out << resultCodeToString(Stm32SerialResult::WifiHandlerNullptrError) << Debug::endl;
                    }
                    else
                    {
                        // Parsing the message was successfull. Queue the data in the WiFi outgoing queue.
                        //m_wifi->addItemOutgoingQueue(data); // Uncomment when wifi task is implemented.
                        Debug::out << "Data packet was 'put' in wifi outgoing queue" << Debug::endl;
                    } 
                }
                else
                {
                    // Print error message.
                    Debug::out << resultCodeToString(parseResult) << Debug::endl;
                }

                // Transmit the result back to the STM32.
                Stm32SerialResult transmitResult = transmitResponse(parseResult);
                
                // Print out the write result.
                Debug::out << "Write result: " << resultCodeToString(transmitResult) << Debug::endl;

            }
        }
    } // while
}

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
Stm32SerialResult Stm32SerialportTask::parseReceivedData(SerialQueueData data,  Stm32SerialDataPacket *packet)
{
    // Verify that the received data is large enough to actually hold the stm command.
    if (data.length < STM32_SERIAL_MINIMUM_PACKET_SIZE) // minimum size including the sync bytes and payload length and crc
    {
        // Received packet did not have the minimum size requirement to be valid.
        return Stm32SerialResult::ReceivedPacketSizeError;
    }

    // Check that the sync bytes are in the index 0 and 1 to verify message is a stm32 message.
    if (data.buffer[STM32_FIRST_SYNC_BYTE_INDEX] != ESP_STM_FIRST_SYNC_BYTE && 
        data.buffer[STM32_SECOND_SYNC_BYTE_INDEX] != ESP_STM_SECOND_SYNC_BYTE)
    {
        // Received data did not contain sync bytes.
        return Stm32SerialResult::SyncByteError;
    }

    // Parse the received payload length.
    packet->payloadLength = 
        ((data.buffer[STM32_PAYLOAD_LENGTH_START_INDEX] & 0xFF) << 8U) |
        (data.buffer[STM32_PAYLOAD_LENGTH_START_INDEX + 1] & 0xFF);

    // Payload should never be empty.
    if (packet->payloadLength == 0)
    {
        // Payload was empty.
        return Stm32SerialResult::PayloadEmptyError;
    }

    // Verify that the data packet contains the full payload
    if (data.length < 5 + packet->payloadLength)
    {
        // Received packet did not have the required size.
        return Stm32SerialResult::ReceivedPacketSizeError;
    } 

    // Set the payload buffer pointer.
    packet->payload = &data.buffer[STM32_PAYLOAD_START_INDEX];

    // Get the crc.
    packet->crc = data.buffer[STM32_PAYLOAD_START_INDEX + packet->payloadLength];

    // Calculate the checksum.
    uint8_t calculatedCrc = calculateChecksum(&data.buffer[STM32_PAYLOAD_START_INDEX], packet->payloadLength);

    // Verify that the received checksum matches the calculated one.
    if (calculatedCrc != packet->crc)
    {
        // The checksum did not match.
        return Stm32SerialResult::ChecksumError; // checksum error.
    }

    // Message was successfully parsed.
    return Stm32SerialResult::Success;
}

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
Stm32SerialResult Stm32SerialportTask::transmitResponse(const Stm32SerialResult parseResult)
{
    // Holds the data to be transmitted to the STM32.
    SerialQueueData responseData;

    // Set the data buffer to be transmitted.
    responseData.buffer[0] = ESP_STM_FIRST_SYNC_BYTE;                   // Set the first sync byte.  
    responseData.buffer[1] = ESP_STM_SECOND_SYNC_BYTE;                  // Set the second sync byte.
    responseData.buffer[2] = static_cast<uint8_t>(parseResult);         // Set the result of the operation.
    responseData.buffer[3] = calculateChecksum(responseData.buffer, 3); // Calculate and set the checksum.
    responseData.length = 4; // Set the length of the buffer.

    // Add the data to the transmit queue.
    SerialportResult queueResult = processQueueItem(&responseData, SerialQueueType::Transmit, pdMS_TO_TICKS(WRITE_TIMEOUT));

    // Check the status of the queue operation.
    if (queueResult != SerialportResult::Success)
    {
        // Queue operation failed.
        Debug::out << SerialportTask::resultCodeToString(queueResult) << Debug::endl;
        return Stm32SerialResult::SendDataTransmitQueueError;
    }

    // Stores the number of bytes transmitted.
    uint16_t bytesTransmitted = 0;

    // Transmit the data on the serialport.
    SerialportResult writeResult = serialWrite(bytesTransmitted, pdMS_TO_TICKS(WRITE_TIMEOUT));

    // Check the status of the write operation.
    if (writeResult != SerialportResult::Success)
    {
        // Write operation failed.
        Debug::out << SerialportTask::resultCodeToString(writeResult) << Debug::endl;
        return Stm32SerialResult::WriteFailedError;
    }

    // Verify that the correct size was transmitted.
    if (bytesTransmitted != responseData.length)
    {
        // The number of bytes that was supposed to be transmitted was not transmitted.
        return Stm32SerialResult::WriteSizeIncorrectError;
    }

    // Operation was successful.
    return Stm32SerialResult::Success;
}

/**
 * @brief Calculates a 1 byte sum-of-bytes checksum.
 *
 * @param[in] data The data pointer containing the data to calculate the checksum of.
 * @param[in] length The length of the data pointer.
 *
 * @returns
 *  The calculated 1-byte checksum.
 */
uint8_t Stm32SerialportTask::calculateChecksum(uint8_t *data, uint16_t length)
{

    // Holds the checksum to be calculated.
    uint8_t checksum = 0;

    // Iterate the data buffer and add the bytes to the checksum.
    for (uint16_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }

    // Return the calculated checksum.
    return checksum;
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
const char *Stm32SerialportTask::resultCodeToString(const Stm32SerialResult code)
{
    static const std::unordered_map<Stm32SerialResult, const char *> resultMap =
        {
            {Stm32SerialResult::Success, "The STM32 serial operation was successful!"},
            {Stm32SerialResult::ReceivedPacketSizeError, "The received packet from STM32 had a packet size error!"},
            {Stm32SerialResult::SyncByteError, "The received packet from the STM32 had a sync byte error!"},
            {Stm32SerialResult::ChecksumError, "The received packet from the STM32 had a checksum error!"},
            {Stm32SerialResult::PayloadEmptyError, "The received packet from the STM32 had a empty payload!"},
            {Stm32SerialResult::WriteFailedError, "The STM32 write operation failed!"},
            {Stm32SerialResult::WriteSizeIncorrectError, "The STM32 write operation was not able to write all the data!"},
            {Stm32SerialResult::WifiHandlerNullptrError, "The STM32's wifi handler was nullptr!"},
            {Stm32SerialResult::SendDataTransmitQueueError, "Failed to add data in STM32 serialports transmit queue!"}
        };

    auto it = resultMap.find(code);
    return it != resultMap.end() ? it->second : "STM32 Serialport operation resulted in an unknown result code!";
}
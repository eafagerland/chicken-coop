/**
 * @file esp_stm_comm_definitions.h
 *
 * @date Okt 28, 2024
 * @author Erik Fagerland
 * 
 * @brief Contains data definitions for the communication between the ESP32 and STM32.
 * 
 * @remarks
 *  
 *  Data from the STM32 will in the following structure:
 * 
 *  [0] - First sync byte.
 *  [1] - Second sync byte.
 *  [2] - Payload length MSB
 *  [3] - Payload length LSB
 *  [4] - Start of payload.
 *  [4 + payload length] - Checksum (calculated from payload.).
 * 
 *  The payload will consist of multiple messages. Each message will be defined with a
 *  message identifier and a fixed size for each message.
 * 
 */

#ifndef ESP_STM_COMM_DEFINITIONS_H_
#define ESP_STM_COMM_DEFINITIONS_H_

// Standard C++ headers.
#include <stdint.h>

/** @brief Defines the first sync byte. This should always be the first byte in a esp/stm transmission. */
constexpr auto ESP_STM_FIRST_SYNC_BYTE = 0xF0;

/** @brief Defines the second sync byte. This should always be the second byte in a esp/stm transmission. */
constexpr auto ESP_STM_SECOND_SYNC_BYTE = 0xFE;

/** 
 * @brief Represents the different outcomes of the Stm32 serial operations. 
 * 
 * @remarks
 *  The status code will be transmitted back to the STM32 after proccessing the received message.
 *  This is so the STM32 can handle this with retransmitting message or debugging purposes. 
 */
enum class Stm32SerialResult
{

    /** @brief Indicates that the operation was successful. */
    Success = 0,

    /** @brief Indicates that the received packet was not the size that was expected. */
    ReceivedPacketSizeError,

    /** @brief Indicates that the sync bytes in received packet was invalid. */
    SyncByteError,

    /** @brief Indicates that the received packet had a checksum error. */
    ChecksumError,

    /** @brief Indicates that the payload was empty. */
    PayloadEmptyError,

    /** @brief Indicates that the serial write operation failed. */
    WriteFailedError,

    /** @brief Indicates the incorrect number of bytes was transmitted on the serialport. */
    WriteSizeIncorrectError,

    /** @brief Indicates that the Wifi handler was nullptr. */
    WifiHandlerNullptrError,

    /** @brief Indicates that adding data to the transmit queue failed. */
    SendDataTransmitQueueError,

};

/**
 * @brief Represents the data packet structure for transmitted by the STM32 to the ESP32.
 */
struct Stm32SerialDataPacket
{

    /** @brief A static sync byte to determine start of message. */
    uint8_t firstSyncByte;

    /** @brief A static sync byte to determine start of message. */
    uint8_t secondSyncByte;

    /** @brief Holds the length in bytes of the payload. */
    uint16_t payloadLength;

    /** @brief Holds the pointer to the payload data. 
     * 
     * @remarks
     *  Payload may consist of many messages, first byte in payload will be the 
     *  MID (message identifier).
    */
    uint8_t *payload;

    /** @brief A 1byte checksum for the packet (sum-of-bytes). */
    uint8_t crc;

};

#endif // ESP_STM_COMM_DEFINITIONS_H_
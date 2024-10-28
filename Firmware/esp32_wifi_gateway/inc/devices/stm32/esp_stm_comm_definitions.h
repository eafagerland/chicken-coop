/**
 * @file esp_stm_comm_definitions.h
 *
 * @date Okt 28, 2024
 * @author Erik Fagerland
 * 
 * @brief Contains data definitions for the communication between the ESP32 and STM32.
 * 
 * @remarks
 *  Structs and enums contains both C and C++ style declarations with use of macros.
 *  This is so they can be used for both C and C++ compilers.
 */

#ifndef ESP_STM_COMM_DEFINITIONS_H_
#define ESP_STM_COMM_DEFINITIONS_H_

// Standard C++ headers.
#include <stdint.h>

#ifdef __cplusplus
/** @brief Represents the different status during stm32/esp32 serial transmission. */
enum class Stm32SerialDataStatus : uint8_t
#else
typedef enum stm32_serial_data_status
#endif
{

    /** @brief Indicates that the operation was success. */
    Success = 0x20,

    /** @brief Indicates that a invalid command was recieved. */
    InvalidCommandReceivedError,

    /** @brief Indicates that there was a data overflow. */
    PayloadOverflowError,

    /** @brief Indicates that there was aa checksum error. */
    ChecksumError,

#ifdef __cplusplus
};
#else
} Stm32SerialDataStatus_t;
#endif

/**
 * @brief Represents the data packet structure for communicating with the Stm32.
 */
#ifdef __cplusplus
struct Stm32SerialDataPacket
#else
typedef struct stm32_serial_data_packet
#endif
{

    /** @brief A static sync byte to determine start of message. */
    uint8_t firstSyncByte;

    /** @brief A static sync byte to determine start of message. */
    uint8_t secondSyncByte;

    /** @brief Holds the status of the transmission. */
    Stm32SerialDataStatus status;

    /** @brief The current command for identifying the packet. */
    uint8_t command;

    /** @brief Holds the length of the payload. */
    uint16_t payloadLength;

    /** @brief The payload containing the data. */
    uint8_t *payload;

    /** @brief A 1byte checksum for the packet (sum-of-bytes). */
    uint8_t crc;

#ifdef __cplusplus
};
#else
} Stm32SerialDataPacket_t;
#endif

#endif // ESP_STM_COMM_DEFINITIONS_H_
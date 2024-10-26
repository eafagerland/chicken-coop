/**
 * @file debug_utils.h
 *
 * @date Okt 25, 2024
 * @author Erik Fagerland
 * 
 * @brief Provides macros for an easier way to output debug data on the serialport.
 * 
 * @note 
 *  To disable debug log comment the "#define DEBUG" line.
 *  All log entries in the rest of the code will then be removed automatically.
 */

#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

/**
 * @brief The baudrate to run the debug UART on.
 */
constexpr auto DEBUG_SERIAL_BAUDRATE = (115200UL);

// Uncomment the following line for debug mode (comment for release).
#define DEBUG

// Macro for easier print to serial for debugging.
#ifdef DEBUG
#define DEBUG_INTERFACE Serial
#define log(x) Serial.print(x)
#define logln(x) Serial.println(x)
#else
#define log(x) (void)0
#define logln(x) (void)0
#endif

#endif // DEBUG_UTILS_H
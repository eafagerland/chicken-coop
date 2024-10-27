#ifndef APPLICATION_CONFIG_H_
#define APPLICATION_CONFIG_H_

// Third part headers.
#include <Arduino.h>

/**
 * @brief Holds the task and serialport configuration for the Stm32 serialport.
 */
namespace Stm32SerialportTaskConfig 
{

/** @brief The stacksize of the STM32 serialport task. */
constexpr uint32_t stackSize = 2048UL;

/** @brief The priority of the STM32 serialport task. */
constexpr auto taskPriority = 1;

/** @brief The taskname of the STM32 serialport task. */
constexpr const char* taskName = "serialTask";

/** @brief The ESP32 serial handle to be used to communicate with the Stm32. */
static constexpr HardwareSerial& serialHandle = Serial1;

/** @brief The UART receive pin used on the ESP32 that is connected to the STM32. */
constexpr uint8_t rxPin = 4;

/** @brief The UART transmit pin used on the ESP32 that is connected to the STM32. */
constexpr uint8_t txPin = 5;

/** @brief The serialport baudrate used on the Stm32 serialport. */
constexpr unsigned long baudRate = 115200;

} // namespace Stm32SerialportTaskConfig

#endif // APPLICATION_CONFIG_H_
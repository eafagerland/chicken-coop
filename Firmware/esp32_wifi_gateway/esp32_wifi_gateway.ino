
/**
 * @file esp32_wifi.ino
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Arduino sketch for ESP32 onbard ChickenCoop PCB.
 * 
 * @remarks
 *  Acts as a UART <-> WiFi gateway between the HMI client and onbard MCU (STM32) 
 *  on the ChickenCoop Control system.
 * 
 *  All tasks uses FreeRTOS.
 * 
 *  On the WiFi network it will transmit data packets on a fixed interval containing:
 *      - Temperature Inner/External readings.
 *      - Humidity Inner/External readings.
 *      - Current measurements for 230VAC Output #1(Heating), #2(Lights) and #3(Reserved).
 *      - Flap doors open/closed state.
 *      - Flap doors motor diagnostics.
 *      - Errors if present.
 * 
 *  It will also receive data packets from the HMI client as WiFi data packets:
 *      - Flap door to garden manual control.
 *      - Flap door to coop manual override.
 *      - Coop lights manual override.
 *      - Temperature control setting.
 * 
 * @note
 *  Chicken heads might get chopped while closing doors.
 */

// Third party headers.
#include "inc/application_core.h"

// Declare the application core task.
ApplicationCore app;

/**
 * @brief A hardfault handler, infinite loop.
 * 
 * @param[in] code The error that occured when handler was called.
 */
static void hardfaultHandler(const TaskResult code)
{
    // Prevent unused warning.
    (void)code;

    // Loop of death.
    while(true)
        ;
}

/**
 * @brief System startup.
 * 
 * @remarks
 *  Starts the main application where all tasks are inititated as FreeRTOS tasks.
 */
void setup() 
{
    // Store the result of the operation.
    TaskResult result;

    // Initalize and start the main application.
    result = app.init();

    // Check if the operation was successful.
    // If it fails to initialize the main application, go to hardfaultHandler.
    if (result != TaskResult::Success)
    {
        hardfaultHandler(result);
    }
}

/**
 * @brief Unused loop function (Must be implemented in order to build). What goes around comes around.
 */
void loop()
{
    vTaskDelay(portMAX_DELAY);  // Block the loop indefinitely
}

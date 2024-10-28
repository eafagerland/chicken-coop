/**
 * @file stm32_serial_task.cpp
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Task for handling serialport data receiving and transmitting.
 */

// Own header.
#include "inc/stm32_serial_task.h"

// Project headers.
#include "inc/debug/debug_utils.h"
#include "inc/devices/stm32/esp_stm_comm_definitions.h"

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
            Debug::out << "Failed to read serialport: " << resultCodeToString(readResult) << Debug::endl;
        }

        // For testing serial receive:
        // Data is sent manually with terminal connected to dev board.
        SerialQueueData data;
        SerialportResult queueResult = processQueueItem(&data, SerialQueueType::Receive, 10);
        if (queueResult != SerialportResult::Success && queueResult != SerialportResult::ReceiveQueueTimeoutError)
        {
            Debug::out << "Failed to receive from queue: " << resultCodeToString(queueResult) << Debug::endl;
        }
        else
        {
            // Print the number of bytes received.
            if (data.length != 0)
            {
                Debug::out << "Received bytes: " << data.length << Debug::endl;
            }
        }
    } // while
}
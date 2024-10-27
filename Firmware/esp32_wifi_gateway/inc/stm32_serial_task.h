/**
 * @file stm32_serial_task.h
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Task for handling the communication with the STM32 MCU.
 */

#ifndef STM32_SERIAL_TASK_H_
#define STM32_SERIAL_TASK_H_

// Project headers.
#include "inc/serialport_task.h"
#include "inc/application_config.h"

/**
 * @brief Task for handling the communication with the STM32 MCU.
 */
class Stm32SerialportTask : public SerialportTask<Stm32SerialportTaskConfig::stackSize>
{

public:

    /**
     * @brief Stm32SerialPort Constructor
     */
    Stm32SerialportTask() : SerialportTask(){};

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
};

#endif // STM32_SERIAL_TASK_H_
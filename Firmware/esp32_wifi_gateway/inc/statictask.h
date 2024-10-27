/**
 * @file statictask.h
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Abstract base class for static tasks.
 */

#ifndef STATIC_TASK_H_
#define STATIC_TASK_H_

// Standard C++ headers.
#include <stdint.h>
#include <unordered_map>

// Third party header.
#include <Arduino.h>

/**
 * @brief Stores the static queue.
 * 
 * @tparam QueueEntriesMax The maximum number of entries in the queue.
 * @tparam QueueEntrySize The size of each entry in the queue.
 */
template <size_t QueueEntriesMax, size_t QueueEntrySize>
struct StaticQueue
{

    /** @brief A handle to the USB queue items */
    QueueHandle_t handle;

    /** @brief Backing storage for the USB queue */
    StaticQueue_t buffer;

    /** @brief Array that stores the queue data */
    uint8_t storage[QueueEntriesMax * QueueEntrySize];
    
};

/**
 * @brief Holds the different result from task operations.
 */
enum class TaskResult
{

    /** @brief Indicates that the operation was successfull. */
    Success,

    /** @brief Indicates that the task creation failed. */
    TaskCreationError,

    /** @brief Indicates that the operation failed due to initialization error. */
    InitializationError,

};

/**
 * @brief Base abstract class for static FreeRTOS tasks.
 * 
 * @tparam StackSize The amount of bytes to allocate for the stack size of the task.
 */
template<size_t StackSize>
class StaticTask
{

public:

    /**
     * @brief The StaticTask constructor.
     * 
     * @remarks
     *  Initializes the initialized state to false.
     */
    StaticTask()
    {
        m_isInitialized = false;
    }


protected:

    /** 
     * @brief Creates the task.
     * 
     * @param[in] priority Priority of the task.
     * @param[in] name Name of the task.
     * 
     * @retval TaskResult::Success
     *  If the task creation was successfull.
     * @retval TaskResult::TaskCreationError
     *  If the task creation failed.
     */
    TaskResult createTask(UBaseType_t priority, const char * const name)
    {
        // Initialize the init flag.
        m_isInitialized = false;

        // Initialize the task handle.
        m_taskHandle = NULL;

        // Attempt to create the core task.
        m_taskHandle = xTaskCreateStatic(
            taskInit,       // The task function.
            name,           // The task name.
            StackSize,      // The stack size of the task.
            this,           // pvParameters, "this" is passed so that it can call its run() function.
            1,              // The priority of the task.
            xStack,         // Pointer to the stack buffer.
            &xTaskBuffer    // Task buffer. 
        );

        // Return error if unable to create task.
        if (m_taskHandle == NULL)
        {
            return TaskResult::TaskCreationError;
        }
        else
        {
            return TaskResult::Success;
        }
    }

    /** 
     * @brief Abstract task worker function to be overriden by derrived class. 
     * 
     * @remarks
     *  This is the tasks worker function where it will perform its work. i.e infinite loop.
    */
    virtual void run() = 0;

    /** @brief Stores the initialization flag for the task. */
    bool m_isInitialized;

    /** @brief Stores the task handle for the task. */
    TaskHandle_t m_taskHandle;

    /**
    * @brief Clears the stack and TCB memory bufers.
    */
    void clearMemory()
    {
        memset(xStack, 0, sizeof(xStack));
        memset(&xTaskBuffer, 0, sizeof(xTaskBuffer));
    }

private:

    /**
     * @brief Function for initializing the task.
     * 
     * @param[in] pvParameters "this" pointer is passed for calling run().
     */
    static void taskInit(void *pvParameters)
    {
        // Cast the given parameter to StaticTask object.
        StaticTask *task = static_cast<StaticTask<StackSize>*>(pvParameters);

        if (task == nullptr)
        {
            // Failed to retreive the pointer for the task,
            // and cannot start the task.
            return;
        }

        // Start the task.
        task->run();
    }

    /** @brief Structure that will hold the TCB of the task being created. */
    StaticTask_t xTaskBuffer;

    /** @brief Holds the buffer for the stack being created. */
    StackType_t xStack[StackSize];

public:

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
    static const char *resultCodeToString(const TaskResult code)
    {
        static const std::unordered_map<TaskResult, const char*> resultMap = 
        {
            { TaskResult::Success, "The static task operation was successful!" },
            { TaskResult::TaskCreationError, "The static task failed to be created!" },
            { TaskResult::InitializationError, "The static task operation failed due to a initialization error!" }
        };

        auto it = resultMap.find(code);
        return it != resultMap.end() ? it->second : "Serialport operation resulted in an unknown result code!";
    }
};

#endif // STATIC_TASK_H_
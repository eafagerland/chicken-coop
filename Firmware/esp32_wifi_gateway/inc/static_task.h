/**
 * @file static_task.h
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Abstract base class for static tasks.
 */

#ifndef STATIC_TASK_H_
#define STATIC_TASK_H_

// Third party header.
#include <Arduino.h>

// Standard C++ headers.
#include <unordered_map>

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
    Success = 0,

    /** @brief Indicates that the task creation failed. */
    TaskCreationError,

    /** @brief Indicates that the operation failed due to initialization error. */
    InitializationError,

    /** @brief Indicates that the instanced task handle was nullptr. */
    TaskHandleNullError,

};

/**
 * @brief Base abstract class for static FreeRTOS tasks.
 */
class StaticTask
{

public:

    /**
     * @brief The StaticTask constructor.
     * 
     * @remarks
     *  Initializes the initialized state to false.
     */
    StaticTask(StackType_t *stackPointer, const uint32_t stackSize) : 
        m_isInitialized(false),
        m_stack(stackPointer),
        m_stackSize(stackSize) {}

    /**
     * @brief Destructor of the StaticTask task.
     * 
     * @remarks
     *  Deletes the task and clears the memory.
     */
    ~StaticTask()
    {
        // Delete the task.
        deleteTask();
        // Clear memory.
        clearMemory();
    }

    /**
     * @brief Attempts to delete the task.
     * 
     * @remarks
     *  If the task is not responding as expected the task can be
     *  tried to be restarted by deleting and calling @ref init again.
     * 
     * @retval TaskResult::Success
     *  If the operation was successful.
     * @retval TaskResult::InitializationError
     *  If the serialport was not initialized.
     * @retval TaskResult::TaskHandleNullError
     *  If the serialport task handle was NULL.
     */
    TaskResult deleteTask();

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
    TaskResult createTask(UBaseType_t priority, const char * const name);

    /** @brief Stores the initialization flag for the task. */
    bool m_isInitialized;

    /** @brief Stores the task handle for the task. */
    TaskHandle_t m_taskHandle;

    /**
    * @brief Clears the stack and TCB memory bufers.
    */
    void clearMemory();

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
    static const char *resultCodeToString(const TaskResult code);

    /** 
     * @brief Abstract task worker function to be overriden by derrived class. 
     * 
     * @remarks
     *  This is the tasks worker function where it will perform its work. i.e infinite loop.
    */
    virtual void run() = 0;

private:

    /**
     * @brief Function for initializing the task.
     * 
     * @param[in] pvParameters "this" pointer is passed for calling run().
     */
    static void taskInit(void *pvParameters);

    /** @brief Structure that will hold the TCB of the task being created. */
    StaticTask_t m_taskBuffer;

    /** @brief Holds the pointer to the buffer for the stack being created. */
    StackType_t *m_stack;

    /** @brief Stores the size of the stack in bytes. */
    uint32_t m_stackSize;
};

#endif // STATIC_TASK_H_
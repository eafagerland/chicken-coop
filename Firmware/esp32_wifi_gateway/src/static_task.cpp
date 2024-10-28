
/**
 * @file static_task.h
 *
 * @date Okt 13, 2024
 * @author Erik Fagerland
 * 
 * @brief Abstract base class for static tasks.
 */

// Own header.
#include "inc/static_task.h"

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
TaskResult StaticTask::createTask(UBaseType_t priority, const char *const name)
{
    // Initialize the init flag.
    m_isInitialized = false;

    // Initialize the task handle.
    m_taskHandle = NULL;

    // Attempt to create the core task.
    m_taskHandle = xTaskCreateStatic(
        taskInit,     // The task function.
        name,         // The task name.
        m_stackSize,  // The stack size of the task.
        this,         // pvParameters, "this" is passed so that it can call its run() function.
        1,            // The priority of the task.
        m_stack,      // Pointer to the stack buffer.
        &m_taskBuffer // Task buffer.
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
 * @brief Function for initializing the task.
 *
 * @param[in] pvParameters "this" pointer is passed for calling run().
 */
void StaticTask::taskInit(void *pvParameters)
{
    // Cast the given parameter to StaticTask object.
    StaticTask *task = static_cast<StaticTask *>(pvParameters);

    if (task == nullptr)
    {
        // Failed to retreive the pointer for the task,
        // and cannot start the task.
        return;
    }

    // Start the task.
    task->run();
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
TaskResult StaticTask::deleteTask()
{
    // Check that the task was initialized shutting it down.
    if (!m_isInitialized)
    {
        return TaskResult::InitializationError;
    }
    // Check that the task handle is valid.
    else if (m_taskHandle == NULL)
    {
        return TaskResult::TaskHandleNullError;
    }

    // Delete the task.
    vTaskDelete(m_taskHandle);

    // Clear the Stack and TCB memory.
    clearMemory();

    // Reset the init flag.
    m_isInitialized = false;

    return TaskResult::Success;
}

/**
 * @brief Clears the stack and TCB memory bufers.
 */
void StaticTask::clearMemory()
{
    memset(m_stack, 0, m_stackSize);
    memset(&m_taskBuffer, 0, sizeof(m_taskBuffer));
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
const char *StaticTask::resultCodeToString(const TaskResult code)
{
    static const std::unordered_map<TaskResult, const char *> resultMap =
        {
            {TaskResult::Success, "The static task operation was successful!"},
            {TaskResult::TaskCreationError, "The static task failed to be created!"},
            {TaskResult::InitializationError, "The static task operation failed due to an initialization error!"},
            {TaskResult::TaskHandleNullError, "The static task operation failed because the instanced task handler was nullptr!"}
        };

    auto it = resultMap.find(code);
    return it != resultMap.end() ? it->second : "Serialport operation resulted in an unknown result code!";
}
#include "minipilot/mp_main.hpp"
#include "tasks/task_logger.hpp"
#include "util/logger.hpp"

namespace mp {

int mp_main(const mp_devices_s& devices)
{
    /* If the logging task is not created, this stays uninitialized */
    task_logger* task_logger_ptr = nullptr;

    /* Initialize the logging system (task) if there is an available logging device */
    if (devices.log_device && devices.log_device->probe()) {
        /* Create the logging task */
        static task_logger task_logger(*devices.log_device);
        task_logger_ptr = &task_logger;

        /* Logging device is used directly until the scheduler starts */
        logger::get_instance().set_output_device(*devices.log_device);
        log_info("Logging available!\n");
    }

    log_info("Starting the scheduler...\n");
    
    /* If a logging device exists, it means the task was already
    created, so now switch the logging to go through the logging task */
    if (task_logger_ptr) {
        logger::get_instance().set_output_device(*task_logger_ptr);
    }

    /* Start the scheduler */
    emblib::task::start_tasks();
    
    /* Should never reach this */
    return 1;
}

}
#include "mp/main.hpp"
#include "tasks/task_logger.hpp"
#include "tasks/task_telemetry.hpp"
#include "tasks/task_accelerometer.hpp"
#include "tasks/task_gyroscope.hpp"
#include "tasks/task_state_estimator.hpp"
#include "tasks/task_receiver.hpp"
#include "util/logger.hpp"

namespace mp {

int main(const devices_s& devices, vehicle& vehicle)
{
    // If the logging task is not created, this stays uninitialized
    task_logger* task_logger_ptr = nullptr;

    // Initialize the logging system (task) if there is an available logging device
    if (devices.log_device && devices.log_device->probe()) {
        // Create the logging task
        static task_logger task_logger(*devices.log_device);
        task_logger_ptr = &task_logger;

        // Logging device is used directly until the scheduler starts
        logger::get_instance().set_output_device(*devices.log_device);
        log_info("Logging available!");
    }

    // Accelerometer is required
    if (!devices.accelerometer.probe()) {
        log_error("Accelerometer not available!");
        return 1;
    }
    // Create the accelerometer task
    static task_accelerometer task_accelerometer(devices.accelerometer);

    // Receiver is required
    if (!devices.receiver_device.probe()) {
        log_error("Receiver not available!");
        return 1;
    }
    // Create the receiver task
    static task_receiver task_receiver(devices.receiver_device);

    // Gyroscope is required
    if (!devices.gyroscope.probe()) {
        log_error("Gyroscope not available!");
        return 1;
    }
    // Create the gyroscope task
    static task_gyroscope task_gyroscope(devices.gyroscope);

    // Create the state estimator task
    static task_state_estimator task_state_estimator(
        vehicle,
        task_accelerometer,
        task_gyroscope
    );

    // If there is a telemetry device available, create the telemetry task
    // Telemetry could also be required (not optional)
    if (devices.telemetry_device && devices.telemetry_device->probe()) {
        static task_telemetry task_telemetry(
            *devices.telemetry_device,
            task_accelerometer,
            task_gyroscope,
            task_state_estimator
        );
        log_info("Telemetry available!");
    } else {
        log_warning("Telemetry not available!");
    }


    log_info("Starting the scheduler...");
    
    // If a logging device exists, it means the task was already
    // created, so now switch the logging to go through the logging task
    if (task_logger_ptr) {
        logger::get_instance().set_output_device(*task_logger_ptr);
    }

    // Start the scheduler
    emblib::task::start_tasks();
    
    // Should never reach this
    return 1;
}

}
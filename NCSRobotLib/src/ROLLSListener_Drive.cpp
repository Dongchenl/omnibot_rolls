#include <ROLLSListener_Drive.h>
#include <chrono>  // milliseconds

// Future improvement: Let this be specified in constructor
const int INTERVAL = 50;  // time in milliseconds to check firing counts
const double WEIGHT_OLD = 0.6;  // Factor for exponential window
const double WEIGHT_NEW = 0.4;  // Factor for exponential window

void ROLLSListener_Drive::receivedNewROLLSEvent(
        unsigned int event) {
    counter_mutex.lock();
    if (drive_left.isOrigin(event)) left_counter++;
    else if (drive_right.isOrigin(event)) right_counter++;
    else if (speed.isOrigin(event)) speed_counter++;
    counter_mutex.unlock();
}

void ROLLSListener_Drive::loop() {
    // Every INTERVAL time, update motors, then sleep for remaining time
    // Note: Execution takes less than a millisecond, so we don't time anything
    while (!_stop) {
        counter_mutex.lock();
        // Update states with old state and new counter
        n_left = WEIGHT_OLD * n_left + WEIGHT_NEW * left_counter;
        n_right = WEIGHT_OLD * n_right + WEIGHT_NEW * right_counter;
        n_speed = WEIGHT_OLD * n_speed + WEIGHT_NEW * speed_counter;
        // Reset counters
        left_counter = 0;
        right_counter = 0;
        speed_counter = 0;
        counter_mutex.unlock();

        // norm over size of populations and length of interval
        // Result dimension will be events / (neurons * s)
        double time_factor = 1000 / INTERVAL;
        double normed_left = n_left * time_factor / drive_left.size();
        double normed_right = n_right * time_factor / drive_right.size();
        double normed_speed = n_speed * time_factor / speed.size();

        // set speed and angular velocity
        pushbot->move(
            (int)(normed_speed * speed_factor),
            (int)( (normed_left - normed_right) * turn_factor)
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(INTERVAL));
    }
}

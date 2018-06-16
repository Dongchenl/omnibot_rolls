#include <EDVSListener_Log.h>

void EDVSListener_Log::receivedNewEDVSEvent(EDVSEvent &e){
    // check if updated timestamp is needed
    if (--delay < 0) {
        delay = LOG_TIMESTAMP_RENEWAL_DELAY;
        // time since listening started, in ms
        duration = pt::microsec_clock::local_time() - start;
        timestamp = duration.total_milliseconds();
    }

    fprintf(log, "%ld,%d,%d,%d\n", timestamp, e.m_x, e.m_y, e.m_pol);
}

void EDVSIMUListener_Log::receivedNewEDVSIMUEvent(EDVSIMUEvent &e){
    // We only get IMU Events every 50 ms or so
    // Only log gyro events at the moment
    duration = pt::microsec_clock::local_time() - start;
    timestamp = duration.total_milliseconds();

    if (e.m_type == EDVSIMUEvent::IMU_GYRO) {
        fprintf(log, "%ld,gyro,%d,%d,%d\n", timestamp, e.m_x, e.m_y, e.m_z);
    }
}

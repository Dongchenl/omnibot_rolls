#ifndef __EDVS_LISTENER_LOG_H__
#define __EDVS_LISTENER_LOG_H__

#include <EDVSListener.h>
#include <Logger.h>
#include <boost/date_time.hpp>

namespace pt = boost::posix_time;

#define LOG_TIMESTAMP_RENEWAL_DELAY 1000

//! This class provides a concrete implementation of a eDVS event listener that logs the received events to a file
class EDVSListener_Log : public EDVSListener, public Logger {
public:
    // Initialize Logger.
    EDVSListener_Log(std::string filename,
                     pt::ptime time_start = pt::microsec_clock::local_time())
        : Logger(filename), start(time_start), delay(0), timestamp(0)
        {fputs("timestamp,x,y,pol\n", log);};

    void receivedNewEDVSEvent(EDVSEvent &e);
    void receivedNewEDVSIMUEvent(EDVSIMUEvent &e) {};

private:
    pt::ptime start;
    pt::time_duration duration;
    int delay;
    long timestamp;
};


class EDVSIMUListener_Log : public EDVSListener, public Logger {
public:
    // Initialize Logger.
    EDVSIMUListener_Log(
        std::string filename,
        pt::ptime time_start = pt::microsec_clock::local_time())
        : Logger(filename), start(time_start), timestamp(0)
        {fputs("timestamp,type,x,y,z\n", log);};

    void receivedNewEDVSEvent(EDVSEvent &e) {};
    void receivedNewEDVSIMUEvent(EDVSIMUEvent &e);

private:
    pt::ptime start;
    pt::time_duration duration;
    long timestamp;
};

#endif

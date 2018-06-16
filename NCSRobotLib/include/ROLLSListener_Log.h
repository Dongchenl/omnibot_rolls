#ifndef __ROLLS_LISTENER_LOG_H__
#define __ROLLS_LISTENER_LOG_H__

#include <ROLLSListener.h>
#include <Logger.h>
#include <boost/date_time.hpp>

namespace pt = boost::posix_time;

class ROLLSListener_Log : public ROLLSListener, public Logger {
public:
    // Initialize Logger
    ROLLSListener_Log(std::string filename,
                     pt::ptime time_start = pt::microsec_clock::local_time())
        : Logger(filename), start(time_start)
    {fputs("timestamp,neuron\n", log);};

    void receivedNewROLLSEvent(unsigned int neuron);

private:
    pt::ptime start;
    long timestamp;
};

#endif

#ifndef __ROBOT_LISTENER_LOG_H__
#define __ROBOT_LISTENER_LOG_H__

#include <string>

#include <RobotListener.h>
#include <Logger.h>
#include <boost/date_time.hpp>

namespace pt = boost::posix_time;

class RobotListener_Log : public RobotListener, public Logger {
public:
    // Initialize Logger
    RobotListener_Log(std::string filename,
                      pt::ptime time_start = pt::microsec_clock::local_time())
        : Logger(filename), start(time_start)
    {fputs("timestamp,x,y\n", log);};

    void receivedNewServoEvent(PushbotSignals &cmd);
    void receivedNewBumperEvent(unsigned int event);
    void receivedNewOmniRobotEvent( double xposition,
                                    double yposition );

private:
    pt::ptime start;
    long timestamp;
};

#endif

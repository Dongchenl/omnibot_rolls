#ifndef __LOG_LISTENER_ROBOT_H__
#define __LOG_LISTENER_ROBOT_H__

#include <queue>
#include <boost/chrono.hpp>

#include "RobotListener.h"

#define EVENTS_TO_LOG 1000

class myLogListenerRobot : public RobotListener
{
    
public:
    myLogListenerRobot(void);
    ~myLogListenerRobot(void);
    void receivedNewSerialUSBEvent( std::queue<servoSignals>& );
    void receivedNewSerialUSBEvent( std::queue<bumper>& );
    void flush();
    
private:
    FILE *f_servo;
    FILE *f_bumper;
    
    servoSignals recordedServoSignalsArray[EVENTS_TO_LOG];
    bumper       recordedBumperSignalsArray[EVENTS_TO_LOG];
    int numOfEventsServo;
    int numOfEventsBumper;
    // for performance measurments, remove later
    boost::chrono::high_resolution_clock::time_point start;
    boost::chrono::high_resolution_clock::time_point stop;
};

#endif

#include <RobotListener_UDP.h>

namespace pt = boost::posix_time;

void RobotListener_UDP::receivedNewServoEvent(PushbotSignals &cmd) {
    // time since listening started, in ms
    pt::time_duration duration = pt::microsec_clock::local_time() - start;
    long timestamp = duration.total_milliseconds();

    // send log-line to listener
    sprintf(buffer, "%ld,%d,%d\n", timestamp, cmd.left, cmd.right);
    send(buffer);
}

void RobotListener_UDP::receivedNewBumperEvent(unsigned int event){

}

void RobotListener_UDP::receivedNewOmniRobotEvent( double xposition,
                                double yposition ){

}

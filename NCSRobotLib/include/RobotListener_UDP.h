#ifndef ROBOT_LISTENER_UDP
#define ROBOT_LISTENER_UDP

#include <RobotListener.h>
#include <UDPCommunication.h>

#include <boost/date_time.hpp>

using boost::asio::ip::udp;
namespace pt = boost::posix_time;

class RobotListener_UDP : public RobotListener, public UDPSender {
public:
    // open socket
    RobotListener_UDP(boost::asio::ip::address_v4 receiver_ip,
                      int receiver_port = DEFAULT_ROBOTSIGNAL_UDP_PORT,
                      pt::ptime time_start = pt::microsec_clock::local_time())
        : UDPSender(receiver_ip, receiver_port), start(time_start) {};
    // add event to buffer, eventually send it
    void receivedNewServoEvent(PushbotSignals &cmd);
    void receivedNewBumperEvent(unsigned int event);
    void receivedNewOmniRobotEvent( double xposition,
                                    double yposition );

private:
    pt::ptime start;
    char buffer[50];
};

#endif

#ifndef EDVS_LISTENER_UDP
#define EDVS_LISTENER_UDP

#include <EDVSListener.h>
#include <UDPCommunication.h>
#include <cstdio>
#include <boost/date_time.hpp>


#define TIMESTAMP_RENEWAL_DELAY 1000

using boost::asio::ip::udp;
namespace pt = boost::posix_time;

/*!
 * \file EDVSListener_udp_forward.h
 * \author Alexander Dietmüller, Hermann Blum
 *
 * This file implements a EDVS Listener that forwards all events via udp.
 *
 */

//! This class provides a concrete implementation of a eDVS event listener that forwards the events via udp
class EDVSListener_UDP : public EDVSListener, public UDPSender
{

public:
    //! Open socket
    EDVSListener_UDP(boost::asio::ip::address_v4 receiver_ip,
                     int receiver_port = DEFAULT_EDVS_UDP_PORT,
                     pt::ptime time_start = pt::microsec_clock::local_time())
        : UDPSender(receiver_ip, receiver_port), buffer{'\0'},
          start(time_start), delay(0), timestamp(0),
          tempfile(std::tmpfile()) {};

    // Send everything stored in tempfile and close
    ~EDVSListener_UDP();

    //! events come one by one,
    void receivedNewEDVSEvent(EDVSEvent &e);
    void receivedNewEDVSIMUEvent(EDVSIMUEvent &e) {};

private:
    char buffer[50];
    pt::ptime start;
    int delay;
    pt::time_duration duration;
    long timestamp;
    std::FILE *tempfile;
};

#endif

#ifndef ROLLS_LISTENER_UDP
#define ROLLS_LISTENER_UDP

#include "ROLLSListener.h"
#include "UDPCommunication.h"

#include <boost/date_time.hpp>

using boost::asio::ip::udp;
namespace pt = boost::posix_time;

/*! \class ROLLSListener_UDP
 *  \author Hermann Blum

 *  \brief Sends Rolls-Events via UDP
 */

class ROLLSListener_UDP : public ROLLSListener, public UDPSender
{
public:
    //! Open socket
    ROLLSListener_UDP(boost::asio::ip::address_v4 receiver_ip,
                      int receiver_port = DEFAULT_ROLLS_UDP_PORT,
                      pt::ptime time_start = pt::microsec_clock::local_time())
        : UDPSender(receiver_ip, receiver_port), start(time_start) {};
    //! events come one by one,
    void receivedNewROLLSEvent( unsigned int event );

private:
    pt::ptime start;
    char buffer[50];
};

#endif

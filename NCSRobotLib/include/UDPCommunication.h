#ifndef UDP_COMMUNICATION
#define UDP_COMMUNICATION

#include <boost/asio.hpp>
#include <string>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>


using boost::asio::ip::udp;


#define DEFAULT_ROLLS_UDP_PORT 12346
#define DEFAULT_EDVS_UDP_PORT 12345
#define DEFAULT_ROBOTSIGNAL_UDP_PORT 12347

#define UDP_BUFFER_SIZE 480

// class for senders
class UDPSender {
public:
    // open socket
    UDPSender(boost::asio::ip::address_v4 receiver_ip, int receiver_port);
    // flush and close socket
    ~UDPSender();

    void send(std::string data);
    void send(const char * data);
    void flush();
private:
    char buffer[UDP_BUFFER_SIZE];
    int buffer_size, temp_size;
    boost::asio::io_service io_service;
    udp::endpoint receiver_endpoint;
    udp::socket socket;
};

#endif

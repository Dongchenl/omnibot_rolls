#include <UDPCommunication.h>

UDPSender::UDPSender(boost::asio::ip::address_v4 receiver_ip,
                     int receiver_port)
: receiver_endpoint(receiver_ip, receiver_port),
  socket(io_service) {
    socket.open(udp::v4());
    buffer[0] = '\0';
    buffer_size = 0;
}

// send remaining messages and close socket
UDPSender::~UDPSender() {
    flush();
    socket.close();
}

void UDPSender::send(std::string data) {
    send(data.c_str());
}

void UDPSender::send(const char * data) {
    // Check if buffer has enough space, otherwise flush
    temp_size = strlen(data);

    if ((temp_size + buffer_size) >= UDP_BUFFER_SIZE ) {
        flush();
    };

    // Append to buffer
    strncat(buffer, data, temp_size);
    buffer_size += temp_size;
}


void UDPSender::flush() {
    socket.send_to(boost::asio::buffer(buffer, UDP_BUFFER_SIZE),
                   receiver_endpoint);
    buffer[0] = '\0';
    buffer_size = 0;
}

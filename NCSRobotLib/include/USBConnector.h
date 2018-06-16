#ifndef USB_CONNECTOR_H
#define USB_CONNECTOR_H

#include <linux/serial.h>
#include <termio.h>

int getUSBFileDescriptor(const char* port_name ="/dev/ttyUSB0", unsigned int arg_baudrate=2000000 )
{
    int m_ttyFd;
    std::cout<<"> Initializing OmniBot_Driver ... ";
    std::cout<<"\t Trying to open on port:";
    std::cout<<port_name<<std::endl;
    int* baudrate = new int();
    *baudrate = arg_baudrate;

    struct termios tty;
    struct serial_struct ser;

    if (!baudrate) {
        return -1;
    }
    if ((m_ttyFd = open(port_name, O_RDWR)) < 0)
    {
        std::cout<<"\t Could not find/open device\n";
        return 0;
    }
    std::cout<<"\t Found/opened device\n";
    speed_t baudrateFlags = getBaudrate(*baudrate);
    if (baudrateFlags == 0) {
        baudrateFlags = B38400;
        if (ioctl(m_ttyFd, TIOCGSERIAL, &ser) < 0)
            return 0;
        ser.flags &= ~ASYNC_SPD_MASK;
        ser.flags |= ASYNC_SPD_CUST;
        ser.custom_divisor = ser.baud_base / *baudrate;
        if (ioctl(m_ttyFd, TIOCSSERIAL, &ser) < 0)
            return 0;
        if (ioctl(m_ttyFd, TIOCGSERIAL, &ser) < 0)
            return 0;
        if (*baudrate != (ser.baud_base / ser.custom_divisor))
        {
            *baudrate = ser.baud_base / ser.custom_divisor;
            std::cout<<"\t Failed to set requested baudrate.\n";
            return 0;
        }
    }
    std::cout << "Baudrate: " << baudrateFlags << std::endl;
    tcgetattr(m_ttyFd, &tty); /* get record of parameters */
    cfsetospeed(&tty, baudrateFlags); /* set output speed */
    cfsetispeed(&tty, baudrateFlags); /* set input speed */

    /*
     * The serial port is configured as non canonical mode, ie, raw mode.
     */
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cflag &= ~(CSIZE | PARENB);
    tty.c_cflag |= CS8;/* 8 bits */
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag |= CRTSCTS;						// use hardware handshaking
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;						// 1 second timeout
    /* write modified record of parameters to port */
    if (tcsetattr(m_ttyFd, TCSANOW, &tty))
    {
        std::cout<<"\tSerPort fatal: error during configuration!\n";
        return 0;
    }
    //Set the serial port to blocking mode
    if (fcntl(m_ttyFd, F_SETFL, 0))
    {
        ::close(m_ttyFd);
        return -1;
    }

    std::cout<<"\t Initialization Successful!\n";
    return m_ttyFd;
}

static speed_t getBaudrate(unsigned int baudrate)
{
    switch (baudrate) {
    case 50:
        return B50;
    case 75:
        return B75;
    case 110:
        return B110;
    case 134:
        return B134;
    case 150:
        return B150;
    case 200:
        return B200;
    case 300:
        return B300;
    case 600:
        return B600;
    case 1200:
        return B1200;
    case 1800:
        return B1800;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    }
    return 0;
}

#endif

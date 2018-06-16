//#include "/home/mario/Semester_Project/src/NCSRobotExamples/omnibot_parallella_mario/include/Robot.h"
#include "Robot.h"
#include <sys/time.h>
#include <mutex>
#include <iostream>


#define ECHO_ON true

Robot::Robot(int fileDescriptor) {
    this->fileDescriptor = fileDescriptor;
    m_toggleListening = false;
    m_readThreadStatus = TERMINATE_THREAD;
    m_status = 0;
		//m_started(false);
    count = new int;
    *count = 0;
    ios = new boost::asio::io_service();
    readTimer = new boost::asio::deadline_timer( *ios, boost::posix_time::milliseconds(UPDATE_TIME) );
    //readTimer->async_wait( boost::bind(threadLoop, readTimer, count, this) );
}

Robot::~Robot() {
    this->close();
}

int Robot::close(void)
{
    std::cout<<"> Closing Robot ..."<<std::endl;
    ::close(fileDescriptor);
    std::cout<<"Closed Serial Connection"<<std::endl;
    return 0;
}

void Robot::warnEvent( std::queue<servoSignals>& ss, std::queue<bumper>& bmpr )
{
    std::list<RobotListener*>::iterator it;
    for(it = m_listeners.begin(); it!=m_listeners.end(); it++) //<-- shouldn't it be ++it?
    {
        //(*it)->receivedNewSerialUSBEvent( ss ); not needed and not supported by current NCSRobotLib
        //(*it)->receivedNewSerialUSBEvent( bmpr );
        //(*it)->receivedNewSerialUSBEvent( b );
    }
}

void Robot::warnEventBumper(unsigned int bumper) {
  std::list<RobotListener*>::iterator it;
  for(it = m_listeners.begin(); it!=m_listeners.end(); it++) //<-- shouldn't it be ++it?
  {
    (*it)->receivedNewBumperEvent( bumper );
  }
}

void Robot::warnEventOmniRobotPos(double xpos, double ypos){
  std::list<RobotListener*>::iterator it;
  for(it = m_listeners.begin(); it!=m_listeners.end(); it++) //<-- shouldn't it be ++it?
  {
    (*it)->receivedNewOmniRobotEvent( xpos, ypos );
  }
}


void Robot::registerListener(RobotListener* listener)
{
    m_listeners.push_back(listener);
}

void Robot::deregisterListener(RobotListener* listener)
{
    m_listeners.remove(listener);
}

int Robot::listen(void)
{
    std::cout << "Robot::listen" << std::endl;
    if(!m_toggleListening)
    {
        std::cout<<"> Activate listening"<<std::endl;
        m_readThreadStatus = Robot::RUN_THREAD;
        m_toggleListening = true;
        *count = 0;
        readThread = new boost::thread(boost::bind(&boost::asio::io_service::run, ios));
        return true;
    }
    else
        return true;
}

int Robot::stopListening(void)
{
    if(!m_toggleListening)
    {
        return true;
    }
    else
    {
        m_readThreadStatus = Robot::TERMINATE_THREAD;
        m_toggleListening = false;
    }
	  return 0;
}

void Robot::readSocket()
{
#define TO_READ    7200   // bytes

    unsigned char               data[TO_READ];             // Buffer to store the data on the USB

    // Data structures for the select
    // see also: http://man7.org/linux/man-pages/man2/select.2.html
    fd_set                      rFdSet;

    FD_ZERO(&rFdSet);
    FD_SET(fileDescriptor, &rFdSet);

    //check servos, bumper and battery
    std::string cmd = "?SA\n?Ib\n?B\n";
    if( sendCommand(cmd) )
      std::cout << "<< " << "ERROR while sending command ..." << std::endl;

    // The select() system call tells you whether there is any data to read on the file descriptors that you're interested in. Read is blocking and would wait until there is something else
    //int r = select(fileDescriptor + 1, &rFdSet, NULL, NULL, NULL);
    int r = 1;
    if(r)
    {
	// read(int fd, void *buf, size_t count) attempts to read up to count bytes from file descriptor fd into the buffer starting at buf.
	int bytesRead = read(fileDescriptor, data, TO_READ); //<-- here the data is read from the USB buffer
	parse( data, bytesRead );
    }
    else if (r == -1)
    {
	std::cout << ">> Error in the listening thread" << std::endl;
    }
    else
    {
	std::cout << ">> No data within 5secs" << std::endl;
    }
}


void Robot::threadLoop(boost::asio::deadline_timer* t, int* count, void* obj)
{
    if(reinterpret_cast<Robot*>(obj)->m_toggleListening)
    {
        std::cout << "----" << *count << "----" << std::endl;
        ++(*count);
        reinterpret_cast<Robot*>(obj)->readSocket();
        // std::cout << "Before: " << t->expires_at() << std::endl;
        t->expires_at(t->expires_at() + boost::posix_time::milliseconds(UPDATE_TIME));
        // std::cout << "After: " << t->expires_at() << std::endl;
        t->async_wait(boost::bind(threadLoop, t, count, obj));    
    }
}


int Robot::sendCommand(std::string cmd, bool verbose, std::mutex * usb_mutex)
{
      unsigned int w;
      int file_descriptor = this->fileDescriptor;
      // Data structures for the select
      fd_set wFdSet;
      FD_ZERO(&wFdSet);
      FD_SET(file_descriptor, &wFdSet);
      int r;
      //usb_mutex->lock();
      r=select(file_descriptor + 1, NULL, &wFdSet, NULL, NULL);
      //r=1;
      if (r) {
        /*unsigned int*/ w = write(fileDescriptor,
                          cmd.c_str(),
                          cmd.length());
        usleep(10000);
        //if (w==cmd.length()) {usb_mutex->unlock();}

    #ifdef ECHO_ON
		if (verbose) {
        std::string str = cmd.substr( 0, w);
        str.erase(std::remove(str.begin(), str.end(), '\n'), str.end());
        //std::cout << "<< " << str << std::endl; // w - 1 to skip the \n at the end off the command
		}
    #endif
      }
      else {
        w=0;
        std::cout<<"Write timeout\n";
        //usb_mutex->unlock();
      }
      //usb_mutex->unlock();
      return (w==cmd.length()); // Got rid of ! in return statement so on success it will return 1 and thus set m_status to 1
															// Will return 1 on connected but turned off robot too...
}


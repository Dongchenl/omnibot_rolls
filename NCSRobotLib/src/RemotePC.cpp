#include <RemotePC.h>

#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>
// provides sleep and usleep
#include <unistd.h>
#include <fcntl.h>

#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>

#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>

static void logg(std::string toLog)
{
	     std::cout << toLog << std::endl;
}

RemotePC::RemotePC(int fileDescriptor)
    : m_status(0),
    m_started(false),
    m_toggleListening(false),
    //m_eventFormat(EVT_FORMAT_E0),
    m_Fd(fileDescriptor),
    m_readThread(0),
    m_readThreadStatus(RemotePC::TERMINATE_THREAD)
{
    m_status = this->startRemotePC();
//     this->startIMU( EDVSIMUEvent::IMU_GYRO, 1000 );
}

RemotePC::~RemotePC(void)
{
    this->close();
}

int RemotePC::startRemotePC()
{
    logg("> Starting Server ...");
    if(m_started)
    {
        logg("\t Server was already started");
        return 0;
    }
    
    //m_status = this->sendCommand("0");

    //this->sendCommand("0");

    m_status = 0;

    if( m_status == 0)
    {
        logg("\t Started Server");
        m_started = true;
        return 0;
    }
    else
    {
        logg("\t Could not start Server");
        m_started = false;
        return -1;
    }
}

int RemotePC::isInit()
{
    return m_status;
}


void* readThreadPC(void* arg)
{
    struct PCArgs* args = (struct PCArgs*)(arg);

    // Not really important in fact
#define TO_READ    7200   // bytes

    // To receive the data
    int                                  ttyFd = *(args->ttyFd);    // The stream
    unsigned char                        data[TO_READ];             // Raw stream of events
    unsigned int                         bytesRead;                 // # Bytes read in the raw stream
    unsigned char                        PCinCollection[7200];     // Buffer to store current read
    int                                  inputProcessingIndex = 0;  // How many bytes have been processed for the current event
    PCEvent               e;                         // Current decoded event
    std::vector<PCEvent> events(TO_READ);           // Buffer to store the decoded events
    //std::string                          asciiData;                 // Current decoded ascii data
    std::cout << "Done initializing for thread" << std::endl;
    // Data structures for the select
    fd_set rFdSet;
    FD_ZERO(&rFdSet);
    std::cout << "Set FD to zero" << std::endl;
    std::cout << "ttyd:" << ttyFd << std::endl;
    FD_SET(ttyFd, &rFdSet);
    std::cout << "Set FD here" << std::endl;
    int r;

   // while(true)
    //{
        std::cout << "readThreadClient" << std::endl;
        r = select(ttyFd + 1, &rFdSet, NULL, NULL, NULL);

        if(r)
        {
             //std::cout << "select successful" << std::endl;
            bytesRead =
            read(ttyFd, data, TO_READ);
             //std::cout << "Bytes Read" << bytesRead << std::endl;

            events.clear();
            
            for( int n = 0; n < (int)bytesRead; n++ )
            {
                PCinCollection[inputProcessingIndex] = data[n];
                inputProcessingIndex++;

                if(((PCinCollection[0]) & 0x80)==0x80)
                {
                    if(inputProcessingIndex==2)
                    {
                        e = ((PCinCollection[0]) & 0x7F);
                        events.push_back(e);
                        inputProcessingIndex = 0;
                    }
                }
                
            }

            // Transmit the events
            args->device->warnEvent(events);

            // Check if the thread is still alive
        //    if(*(args->threadStatus)==RemotePC::TERMINATE_THREAD)
       //         break;
        }
        else if (r == -1)
        {
            std::cout << "> Error in the listening thread" << std::endl;
        }
        else
        {
            std::cout << "> No data within 5secs" << std::endl;
        }
    //}
    pthread_exit(NULL);
}

int RemotePC::listen(void)
{
    if(m_started)
    {
        if(!m_toggleListening)
        {
            m_readThreadArgs.device = this;
            m_readThreadArgs.ttyFd = &m_Fd;
            std::cout << "m_Fd:" << m_Fd << std::endl;
            m_readThreadArgs.threadStatus = &m_readThreadStatus;
            m_readThreadArgs.events = &m_events;

            logg("RemotePC: Activate listening\n");
            m_readThreadStatus = RemotePC::RUN_THREAD;
            m_toggleListening = true;
            pthread_create(&m_readThread,NULL,readThreadPC,(void*)&m_readThreadArgs);

            return 1;
        }
        else
            return 1;
    }
    else
    {   
        logg("\t Cannot activate listening to Client ...\n");
        return -1;
    }
}

int RemotePC::stopListening(void)
{
    if(!m_toggleListening)
    {
        return true;
    }
    else
    {
        m_readThreadStatus = RemotePC::TERMINATE_THREAD;
        m_toggleListening = false;
    }
    return 0;
}



int RemotePC::close(void)
{
    logg("> Exiting PC4337_Driver ... \n");

    ::close(m_Fd);
    logg("Closed PC4337_Driver \n");
    // tcsetattr(STDOUT_FILENO,TCSANOW,&m_oldStdio);
    return 0;
}

void RemotePC::registerListener(PCListener* listener)
{
    m_listeners.push_back(listener);
}

void RemotePC::deregisterListener(PCListener* listener)
{
    m_listeners.remove(listener);
}

void RemotePC::warnEvent(std::vector<PCEvent>& events)
{
    std::list<PCListener*>::iterator it;
    for(it = m_listeners.begin(); it!=m_listeners.end(); it++)
    {
        for( int i = 0; i < (int)events.size(); i++ )
            (*it)->receivedNewPCEvent(events[i]);
    }
}


int RemotePC::sendServer(int cmd)
{  
    // DEBUG Command
    std::cout << "\t Input command new: " << cmd << std::endl;
    if ( write(m_Fd, &cmd, sizeof(cmd))<0) {
        std::cout << "writing on stream socket failed"<< std::endl;
        exit(EXIT_FAILURE);
        //exit(1);
        }

}

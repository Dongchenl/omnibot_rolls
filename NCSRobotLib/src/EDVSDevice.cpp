#include <EDVSDevice.h>

#include <iostream>
#include <sstream>
#include <string.h>
#include <stdlib.h>
// provides sleep and usleep
#include <unistd.h>
#include <fcntl.h>

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

EDVSDevice::EDVSDevice(int fileDescriptor)
    : m_status(0),
    m_started(false),
    m_toggleListening(false),
    m_eventFormat(EVT_FORMAT_E0),
    m_ttyFd(fileDescriptor),
    m_readThread(0),
    m_readThreadStatus(EDVSDevice::TERMINATE_THREAD)
{
    m_status = this->startEDVS();
//     this->startIMU( EDVSIMUEvent::IMU_GYRO, 1000 );
}

EDVSDevice::~EDVSDevice(void)
{
    logg("> Cleaning up EDVS_Driver ...");
    this->stopEDVSAndIMU();
    this->close();
}

int EDVSDevice::startEDVS()
{
    logg("> Starting EDVSDevice ...");
    if(m_started)
    {
        logg("\t EDVS_Driver was already started");
        return 0;
    }

    m_status = this->sendCommand("E+\n");

    if( m_status == 0)
    {
        logg("\t Started EDVSDevice");
        m_started = true;
        return 0;
    }
    else
    {
        logg("\t Could not start EDVSDevice");
        m_started = false;
        return -1;
    }
}

int EDVSDevice::isInit()
{
    return m_status;
}

int EDVSDevice::stopEDVSAndIMU()
{
    logg("> Stopping EDVS_Driver ...");
    if(!m_started)
    {
        logg("\t EDVS_Driver was already stopped");
        return 0;
    }

    m_status = this->sendCommand("E-\n");

    this->stopIMU(EDVSIMUEvent::IMU_GYRO);
    this->stopIMU(EDVSIMUEvent::IMU_COMPASS);
    this->stopIMU(EDVSIMUEvent::IMU_ACCELEROMETER);

    if( m_status == 0)
    {
        logg("\t Stopped EDVS_Driver");
        m_started = false;
        return 0;
    }
    else
    {
        logg("\t Could not stop EDVS_Driver");
        return -1;
    }
    return 0;
}

int EDVSDevice::changeEventFormat(EventFormat fmt)
{
    if(m_started)
    {
        m_status = this->stopEDVSAndIMU();
        if(m_status == -1)
            return -1;
    }

    logg("> Changing EDVS4337_Driver event format ... \n");
    switch(fmt)
    {
        case EVT_FORMAT_E0:
            m_status = this->sendCommand("!E0\n");
            if(m_status==0)
            {
                logg("Changed EDVS4337_Driver event format = E0\n");
                m_eventFormat = EVT_FORMAT_E0;
            }
            break;

        case EVT_FORMAT_E1:
            m_eventFormat = EVT_FORMAT_E1;
            this->sendCommand("!E1\n");
            if(m_status==0)
            {
                logg("Changed EDVS4337_Driver event format = E1\n");
                m_eventFormat = EVT_FORMAT_E1;
            }
            break;

        case EVT_FORMAT_E2:
            m_eventFormat = EVT_FORMAT_E2;
            this->sendCommand("!E2\n");
            if(m_status==0)
            {
                logg("Changed EDVS4337_Driver event format = E2\n");
                m_eventFormat = EVT_FORMAT_E2;
            }
            break;

        case EVT_FORMAT_E3:
            m_eventFormat = EVT_FORMAT_E3;
            this->sendCommand("!E3\n");
            if(m_status==0)
            {
                logg("Changed EDVS4337_Driver event format = E3\n");
                m_eventFormat = EVT_FORMAT_E3;
            }
            break;

        case EVT_FORMAT_E4:
            m_eventFormat = EVT_FORMAT_E4;
            this->sendCommand("!E4\n");
            if(m_status==0)
            {
                logg("Changed EDVS4337_Driver event format = E4\n");
                m_eventFormat = EVT_FORMAT_E4;
            }
            break;

        default:
            m_eventFormat = EVT_FORMAT_E0;
            this->sendCommand("!E0\n");
            if(m_status==0)
            {
                logg("Changed EDVS4337_Driver event format = E0\n");
                m_eventFormat = EVT_FORMAT_E0;
            }
            break;
    }
    return this->startEDVS();
}

inline void parseIMUStrData(std::string imuStrData,
        EDVSIMUEvent::IMUType& type,
        int& x,
        int& y,
        int& z)
{
    int idx = 0;
    std::string num = "";
    for( int i = 0; i < (int)imuStrData.length(); i++ )
    {
        if(imuStrData.at(i) == ' ' || imuStrData.at(i) == '\n')
        {
        // we got the next blank character, therefore num contains another
        // complete output value
            if(idx == 0) {
                if (num.compare("-S7") == 0) {
                    type = EDVSIMUEvent::IMU_GYRO;
                } else if (num.compare("-S8") == 0) {
                    type = EDVSIMUEvent::IMU_ACCELEROMETER;
                } else if (num.compare("-S9") == 0) {
                    type = EDVSIMUEvent::IMU_COMPASS;
                } else if (num.compare("-S16") == 0) {
                    type = EDVSIMUEvent::IMU_HEADING;
                }
            }
            if(idx == 1  && type == EDVSIMUEvent::IMU_HEADING) {
                // HEADING gives a 6 character hex string as value
                const long heading = strtol(num.c_str(), NULL, 16);
                // we can only process signed integers through an EDVSIMUEvent
                // therefore we bitshift by 16 and get a heading from 0 to 255
                // for consistency we pass the heading through the y axis
                x = 0;
                y = heading >> 16;
                z = 0;
                return;
            }
            // in any other case, the data is given as in format x y z ,and as
            // decimal string
            switch (idx) {
                case 1: x = atoi(num.c_str());
                        break;
                case 2: y = atoi(num.c_str());
                        break;
                case 3: z = atoi(num.c_str());
                        break;
            }
            num = "";
            idx++;
        }
        num.push_back(imuStrData.at(i));
    }
}

void* readThreadEDVS(void* arg)
{
    struct EDVSArgs* args = (struct EDVSArgs*)(arg);

    // Not really important in fact
#define TO_READ    7200   // bytes

    // To receive the data
    int                                  ttyFd = *(args->ttyFd);    // The stream
    unsigned char                        data[TO_READ];             // Raw stream of events
    unsigned int                         bytesRead;                 // # Bytes read in the raw stream
    unsigned char                        eDVSinCollection[512];     // Buffer to store current read
    int                                  inputProcessingIndex = 0;  // How many bytes have been processed for the current event
    EDVSEvent               e;                         // Current decoded event
    EDVSIMUEvent            imu;                       // Current decoded imu event
    std::vector<EDVSEvent>  events(TO_READ);           // Buffer to store the decoded events
    std::vector<EDVSIMUEvent> imuEvents(TO_READ);      // Buffer to store the decoded imu events
    EventFormat                          eventFormat;               // Current event format mode
    unsigned int                         eDVSTS = 0;                // The current timestamp on the eDVS for the delta ts mode
    std::string                          asciiData;                 // Current decoded ascii data

    // Data structures for the select
    fd_set rFdSet;
    FD_ZERO(&rFdSet);
    FD_SET(ttyFd, &rFdSet);
    int r;

    while(true)
    {
        //std::cout << "readThreadEDVS4337" << std::endl;
        r = select(ttyFd + 1, &rFdSet, NULL, NULL, NULL);

        if(r)
        {
            // std::cout << "select successful" << std::endl;
            bytesRead =
                read(ttyFd,
                        data,
                        TO_READ);
            // std::cout << "Bytes Read" << bytesRead << std::endl;

            eventFormat = *(args->eventFormat);

            events.clear();
            imuEvents.clear();
            for( int n = 0; n < (int)bytesRead; n++ )
            {
                eDVSinCollection[inputProcessingIndex] = data[n];
                inputProcessingIndex++;

                if(((eDVSinCollection[0]) & 0x80)==0x80)
                {
                    if((eventFormat == EVT_FORMAT_E0) && (inputProcessingIndex==2))
                    {
                        e.m_x = ((eDVSinCollection[0]) & 0x7F);
                        e.m_y = ((eDVSinCollection[1]) & 0x7F);
                        e.m_pol = 1 - (((eDVSinCollection[1]) & 0x80) >> 7);
                        e.m_timestamp = -1;

                        events.push_back(e);
                        inputProcessingIndex = 0;
                    }
                    else if((eventFormat == EVT_FORMAT_E1) && (inputProcessingIndex>2))
                    {
                        if((data[n] & 0x80) == 0x80)
                        {
                            e.m_x = ((eDVSinCollection[0]) & 0x7F);
                            e.m_y = ((eDVSinCollection[1]) & 0x7F);
                            e.m_pol = 1 - (((eDVSinCollection[1]) & 0x80) >> 7);
                            if(inputProcessingIndex==3) eDVSTS +=       ((eDVSinCollection[2]) & 0x7F);
                            if(inputProcessingIndex==4) eDVSTS +=       (((eDVSinCollection[2]) & 0x7F) << 7)
                                |   ((eDVSinCollection[3]) & 0x7F);
                            if(inputProcessingIndex==5) eDVSTS +=       (((eDVSinCollection[2]) & 0x7F) << 14)
                                |   (((eDVSinCollection[3]) & 0x7F) << 7)
                                    |   ((eDVSinCollection[4]) & 0x7F);
                            e.m_timestamp = eDVSTS;

                            events.push_back(e);
                            inputProcessingIndex = 0;
                        }
                    }
                    else if((eventFormat == EVT_FORMAT_E2) && (inputProcessingIndex==4))
                    {
                        e.m_x = ((eDVSinCollection[0]) & 0x7F);
                        e.m_y = ((eDVSinCollection[1]) & 0x7F);
                        e.m_pol = 1 - (((eDVSinCollection[1]) & 0x80) >> 7);
                        e.m_timestamp =     (((eDVSinCollection[2]) & 0xFF) << 8)
                            |   ((eDVSinCollection[3]) & 0xFF);

                        events.push_back(e);
                        inputProcessingIndex = 0;
                    }
                    else if((eventFormat == EVT_FORMAT_E3) && (inputProcessingIndex==5))
                    {
                        e.m_x = ((eDVSinCollection[0]) & 0x7F);
                        e.m_y = ((eDVSinCollection[1]) & 0x7F);
                        e.m_pol = 1 - (((eDVSinCollection[1]) & 0x80) >> 7);
                        e.m_timestamp =     (((eDVSinCollection[2]) & 0xFF) << 16)
                            |   (((eDVSinCollection[3]) & 0xFF) << 8)
                            |   ((eDVSinCollection[4]) & 0xFF);

                        events.push_back(e);
                        inputProcessingIndex = 0;
                    }
                    else if((eventFormat == EVT_FORMAT_E4) && (inputProcessingIndex==6))
                    {
                        e.m_x = ((eDVSinCollection[0]) & 0x7F);
                        e.m_y = ((eDVSinCollection[1]) & 0x7F);
                        e.m_pol = 1 - (((eDVSinCollection[1]) & 0x80) >> 7);
                        e.m_timestamp =     (((eDVSinCollection[2]) & 0xFF) << 24)
                            |   (((eDVSinCollection[3]) & 0xFF) << 16)
                            |   (((eDVSinCollection[4]) & 0xFF) << 8)
                            |   ((eDVSinCollection[5]) & 0xFF);

                        events.push_back(e);
                        inputProcessingIndex = 0;
                    }
                    else if(inputProcessingIndex > 6)
                    {
                        //std::cerr << "Error: too long a timestamp... ignoring data" << std::endl;
                        inputProcessingIndex=0;
                    }
                }
                else
                {
                    if ((data[n] & 0x80) == 0x80)
                    {
                        eDVSinCollection[0] = data[n];
                        inputProcessingIndex = 1;
                        asciiData="";
                        //std::cerr << "Error: set high bit received in ASCII mode... ignoring data" << std::endl;
                    }
                    else
                    {
                        asciiData = asciiData + char(data[n]);
                        if (data[n] == '\n')
                        {
                            parseIMUStrData(asciiData,
                                    imu.m_type,
                                    imu.m_x,
                                    imu.m_y,
                                    imu.m_z);

                            imuEvents.push_back(imu);
                            inputProcessingIndex=0;
                            asciiData="";
                        }
                    }
                }
            }

            // Transmit the events
            args->device->warnEvent(events);
            args->device->warnIMUEvent(imuEvents);

            // Check if the thread is still alive
            if(*(args->threadStatus)==EDVSDevice::TERMINATE_THREAD)
                break;
        }
        else if (r == -1)
        {
            std::cout << "> Error in the listening thread" << std::endl;
        }
        else
        {
            std::cout << "> No data within 5secs" << std::endl;
        }
    }
    pthread_exit(NULL);
}

int EDVSDevice::listen(void)
{
    if(m_started)
    {
        if(!m_toggleListening)
        {
            m_readThreadArgs.device = this;
            m_readThreadArgs.ttyFd = &m_ttyFd;
            m_readThreadArgs.threadStatus = &m_readThreadStatus;
            m_readThreadArgs.events = &m_events;
            m_readThreadArgs.eventFormat = &m_eventFormat;

            logg("EDVSDevice: Activate listening\n");
            m_readThreadStatus = EDVSDevice::RUN_THREAD;
            m_toggleListening = true;
            pthread_create(&m_readThread,NULL,readThreadEDVS,(void*)&m_readThreadArgs);

            return 1;
        }
        else
            return 1;
    }
    else
    {
        logg("\t Cannot activate listening because EDVS4337_Driver has not been started ...\n");
        return -1;
    }
}

int EDVSDevice::stopListening(void)
{
    if(!m_toggleListening)
    {
        return true;
    }
    else
    {
        m_readThreadStatus = EDVSDevice::TERMINATE_THREAD;
        m_toggleListening = false;
    }
    return 0;
}

int EDVSDevice::sendBiases(EDVSBiases b)
{
    std::ostringstream biasTextstream;
    biasTextstream.flush();
    for(int i=0; i<12; i++)
    {
        biasTextstream << "!B" << i << "=" << b.biases[i] << "\n";
    }
    biasTextstream << "!BF\n";
    // std::cout << biasTextstream.str() << std::endl;
    return sendCommand( biasTextstream.str() );
}

EDVSBiases EDVSDevice::getBiases()
{
    return m_biases;
}

void EDVSDevice::startIMU(EDVSIMUEvent::IMUType type, int periodInMs)
{
    std::ostringstream t;
    t << periodInMs;
    int r;

    switch(type)
    {
        case EDVSIMUEvent::IMU_GYRO:
            logg("> Starting IMU Gyrometer ...\n");
            r = this->sendCommand("!S+128,"+t.str()+"\n");
            if(r != 0)
                logg("\t Cannot start IMU Gyrometer ...\n");
            else
                logg("\t Started IMU Gyrometer ...\n");
            break;

        case EDVSIMUEvent::IMU_ACCELEROMETER:
            logg("> Starting IMU Accelerometer ...\n");
            r = this->sendCommand("!S+256,"+t.str()+"\n");
            if(r != 0)
                logg("\t Cannot start IMU Accelerometer\n");
            else
                logg("\t Started IMU Accelerometer\n");
            break;

        case EDVSIMUEvent::IMU_COMPASS:
            logg("> Starting IMU Compass ...\n");
            r = this->sendCommand("!S+512,"+t.str()+"\n");
            if(r != 0)
                logg("\t Cannot start IMU Compass\n");
            else
                logg("\t Started IMU Compass\n");
            break;

        case EDVSIMUEvent::IMU_HEADING:
            logg("> Starting IMU Heading ...\n");
            r = this->sendCommand("!S+65536,"+t.str()+"\n");
            if(r != 0)
                logg("\t Cannot start IMU Heading\n");
            else
                logg("\t Started IMU Heading\n");
            break;

        default:
            logg("> Cannot recognize the option to start IMU ...\n");
            break;
    }
}

void EDVSDevice::stopIMU(EDVSIMUEvent::IMUType type)
{
    int r;

    switch(type)
    {
        case EDVSIMUEvent::IMU_GYRO:
            r = this->sendCommand("!S-128\n");
            if(r != 0)
                logg("\t Cannot stop IMU Gyrometer ...\n");
            break;

        case EDVSIMUEvent::IMU_ACCELEROMETER:
            r = this->sendCommand("!S-256\n");
            if(r != 0)
                logg("\t Cannot stop IMU Accelerometer\n");
            break;

        case EDVSIMUEvent::IMU_COMPASS:
            r = this->sendCommand("!S-512\n");
            if(r != 0)
                logg("\t Cannot stop IMU Compass\n");
            break;

        case EDVSIMUEvent::IMU_HEADING:
            r = this->sendCommand("!S-65536\n");
            if(r != 0)
                logg("\t Cannot stop IMU Heading\n");
            break;

        default:
            logg("> Cannot recognize the option to stop IMU ...\n");
            break;
    }
}

int EDVSDevice::close(void)
{
    logg("> Exiting EDVS4337_Driver ... \n");
    if(m_started)
    {
        this->stopEDVSAndIMU();
        logg("Stopped EDVS4337_Driver \n");
    }
    ::close(m_ttyFd);
    logg("Closed EDVS4337_Driver \n");
    // tcsetattr(STDOUT_FILENO,TCSANOW,&m_oldStdio);
    return 0;
}

void EDVSDevice::registerListener(EDVSListener* listener)
{
    m_listeners.push_back(listener);
}

void EDVSDevice::deregisterListener(EDVSListener* listener)
{
    m_listeners.remove(listener);
}

void EDVSDevice::warnEvent(std::vector<EDVSEvent>& events)
{
    std::list<EDVSListener*>::iterator it;
    for(it = m_listeners.begin(); it!=m_listeners.end(); it++)
    {
        for( int i = 0; i < (int)events.size(); i++ )
            (*it)->receivedNewEDVSEvent(events[i]);
    }
}

void EDVSDevice::warnIMUEvent(std::vector<EDVSIMUEvent>& events)
{
    std::list<EDVSListener*>::iterator it;
    for(it = m_listeners.begin(); it!=m_listeners.end(); it++)
    {
        for( int i = 0; i < (int)events.size(); i++ )
            (*it)->receivedNewEDVSIMUEvent(events[i]);
    }
}

int EDVSDevice::sendCommand(std::string cmd)
{
    // DEBUG Command
    //std::cout << "\t Input command: " << cmd.substr(0,cmd.length()-1) << std::endl;

    unsigned int w = write(m_ttyFd,
            cmd.c_str(),
            cmd.length());

    return !( w == cmd.length() );
}

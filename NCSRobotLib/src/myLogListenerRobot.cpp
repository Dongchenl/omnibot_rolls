#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <vector>
#include <iostream>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "myLogListenerRobot.h"

myLogListenerRobot::myLogListenerRobot(void)
    : numOfEventsServo(0),
      numOfEventsBumper(0)
{
    // setup folder structure
    struct stat st = {0};
    // timestamp for folder
    const boost::posix_time::ptime time(boost::posix_time::microsec_clock::local_time());
    // const boost::posix_time::time_duration time_of_day = time.time_of_day();  /*get current time */
    std::string directory = to_iso_string( time.date() );
    directory = "./"+directory;
    
    if (stat( directory.c_str(), &st) == -1 ) {
        std::cout << "Create File " << directory << std::endl;
        mkdir( directory.c_str(), 0700 );
    }
    // Open servo log file
    std::string filename = "/servos.log"; // was const char * 
    filename = directory + filename;
    f_servo = fopen(filename.c_str(), "a");
    if (f_servo == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    /* print header */
    fprintf(f_servo, "timestamp, S0, S1, S2\n");
    
    // Open bumper log file
    filename = "/bumper.log"; // was const char *
    filename = directory + filename;
    f_bumper = fopen(filename.c_str(), "a");
    if (f_bumper == NULL)
    {
        printf("Error opening file!\n");
        exit(1);
    }
    /* print header */
    fprintf(f_bumper, "timestamp, bumper\n");
}

myLogListenerRobot::~myLogListenerRobot(void)
{
    flush();
    fclose(f_servo);
    fclose(f_bumper);
}


void myLogListenerRobot::receivedNewSerialUSBEvent( std::queue<servoSignals>& events )
{
//     std::cout << "received servo events" << std::endl;
    servoSignals signal;
    while( !events.empty() )
    {
        if( numOfEventsServo < EVENTS_TO_LOG )
        {
            recordedServoSignalsArray[numOfEventsServo] = events.front();
            events.pop();
            numOfEventsServo++;
        }
        else if( numOfEventsServo == EVENTS_TO_LOG )
        {
            // write to file
            for( int i = 0; i < EVENTS_TO_LOG; i++ )
            {
                signal = recordedServoSignalsArray[i];
                fprintf(f_servo, "%s, %d, %d, %d\n",
                            signal.timestamp.c_str(),
                            signal.values[0],
                            signal.values[1],
                            signal.values[2]
                    );
            }
            numOfEventsServo = 0;
        }
    }
};

void myLogListenerRobot::receivedNewSerialUSBEvent( std::queue<bumper>& events )
{
//     std::cout << "received bumper events" << std::endl;
    bumper signal;
    while( !events.empty() )
    {
        if( numOfEventsBumper < EVENTS_TO_LOG )
        {
            recordedBumperSignalsArray[numOfEventsBumper] = events.front();
            events.pop();
            numOfEventsBumper++;
        }
        else if( numOfEventsBumper == EVENTS_TO_LOG )
        {
            // write to file
            for( int i = 0; i < EVENTS_TO_LOG; i++ )
            {
                signal = recordedBumperSignalsArray[i];
                fprintf(f_bumper, "%s, %d\n",
                            signal.timestamp.c_str(),
                            signal.value
                    );
            }
            numOfEventsBumper = 0;
        }
    }
};

void myLogListenerRobot::flush()
{
    std::cout << "LogListener: Flushing ..." << std::endl;
    // write remaining servo signals
    servoSignals signal;
    for(int i = 0; i < numOfEventsServo; i++)
    {
        signal = recordedServoSignalsArray[i];
                fprintf(f_servo, "%s, %d, %d, %d\n",
                            signal.timestamp.c_str(),
                            signal.values[0],
                            signal.values[1],
                            signal.values[2]
                    );
    }
    numOfEventsServo = 0;
    // write remaining servo signals
    bumper signalB;
    for(int i = 0; i < numOfEventsBumper; i++)
    {
        signalB = recordedBumperSignalsArray[i];
        fprintf(f_bumper, "%s, %d\n",
                    signalB.timestamp.c_str(),
                    signalB.value
               );
    }
    numOfEventsBumper = 0;
}

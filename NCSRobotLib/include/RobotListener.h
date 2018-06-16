#ifndef ROBOTLISTENER_H
#define ROBOTLISTENER_H

/*! \file RobotListener.h

 *  \author Michel Frising, Hermann Blum
 * Provides an abstract base class to receive the servo states and the bumber states parsed from the robot event 
 * stream. 
 */

#include <string>
#include <queue>

struct Event
 {
		//public:
		//  Event()
		//	 	: m_data()
		//		{}

		// unsigned int time_stamp;
    unsigned int event_id;
    unsigned long int timestamp;
		char m_data[32]; //!< data from Omnibot USB stream separated at ! or ?
};

struct PushbotSignals {
    int left;
    int right;
};

struct servoSignals {
    int values[3];
    std::string timestamp;
    unsigned long int d_timestamp;
    // double timestamp; // TODO operate with doubles directly (change in LogListener needed)
};

// to globally store wheel encoder's states
struct encoderStates {
		double values[3];
		unsigned long int timestamp;
		unsigned long int delta_t;
};

struct bumper {
    int value;
    std::string timestamp;
    unsigned long int d_timestamp; // TODO operate with doubles directly (change in LogListener needed)
};


//! Abstract base class providing the necessary interface to observe the servo states and bumper states of the observed robot
/*!
 * This class needs to be subclassed and the member functions need to be implemented in order to use the interface
 */
class RobotListener 
{
    public:
        RobotListener(void) {}
//        virtual void receivedNewSerialUSBEvent( std::queue<servoSignals>& ) = 0;
//				virtual void receivedNewSerialUSBEvent( std::queue<encoderStates>& ) = 0;
//        virtual void receivedNewSerialUSBEvent( std::queue<bumper>& ) = 0;
        // Pushbot listener function
        virtual void receivedNewBumperEvent(unsigned int event) = 0; // try as not purely virtual
        // Omnibot listener functions
        virtual void receivedNewOmniRobotEvent( double xposition,
                                                double yposition ) = 0; // try as not purely virtual
        virtual void receivedNewServoEvent(PushbotSignals &cmd) = 0; // try as not purely virtual
//        virtual void flush() = 0;
};
#endif

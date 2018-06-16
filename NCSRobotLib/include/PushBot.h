#ifndef PUSHBOT_H
#define PUSHBOT_H

#define MAX_COMMAND_LENGTH 50 
#define SPEED 20

#include <stdio.h>
#include <termios.h>
#include <string>
#include <vector>
#include <list>
#include <Robot.h>
#include <Enums.h>

/*! \file PushBot.h
 *  protoptypes for the OmniRob USB communiaction and low-level commands
 */

//! Mid-Level interface to the PushBot. Communnication via WiFi
class PushBot : public Robot {
public:
    //! Constructor
    /*!
     * \param fileDescriptor file descriptor to a successfully opened socket to the pushBot
     */
    PushBot(int fileDescriptor) : Robot(fileDescriptor), _forwardVelocity(0), 
                                  _angularVelocity(0) {};
		~PushBot();
		//! set the velocity of the right wheel
    bool setRightVelocity( int );
    //! set the velocity of the left wheel
    bool setLeftVelocity( int );

    // the following 3 functions should not be used together with the
    // low-level methods above
    // set forward velocity
    void setSpeed(int);
    // set angular velocity
    void setAngularVelocity(int);
    // set both above
    void move(int forwardVelocity, int angularVelocity);

    //! check if the servos are working
    bool twitch();
    //! start the motors up
    bool enableMotors();
    //! parse message 
    void parse( unsigned char *, int );
    //! process key strokes logged from the keyboard
    void processKeys( int keysPressed );
	bool switchLaser(bool on);
    // set LED ON or OFF
    bool switchLED(bool on);
private:
    // save parameters
    int _forwardVelocity;
    int _angularVelocity; // positive to turn left
    int _leftSpeed;
    int _rightSpeed;
};

#endif

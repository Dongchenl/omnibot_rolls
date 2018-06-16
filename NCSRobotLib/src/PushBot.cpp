#include "PushBot.h"
#include <iostream>

bool PushBot::enableMotors() 
{
    return sendCommand("!M+\n");
}

PushBot::~PushBot()
{
	std::cout<<"Pushbot distructor"<<std::endl;
}

bool PushBot::twitch() 
{
    bool success = true;
    success &= enableMotors();
    success &= setLeftVelocity(10);
    success &= setRightVelocity(-10);
    usleep(500000); 
    success &= setLeftVelocity(-10);
    success &= setRightVelocity(10);
    usleep(500000); 
    success &= setLeftVelocity(0);
    success &= setRightVelocity(0);

    return success;
}

bool PushBot::setLeftVelocity(int velocity) 
{
    char command[MAX_COMMAND_LENGTH];
    sprintf(command, "!MV0=%d\n", velocity);
    return sendCommand(command);
}

bool PushBot::setRightVelocity(int velocity) 
{
    char command[MAX_COMMAND_LENGTH];
    sprintf(command, "!MV1=%d\n", velocity);
    return sendCommand(command);
}

bool PushBot::switchLaser(bool on)
{
    char command[MAX_COMMAND_LENGTH];
    if (on == true)
	sprintf(command,"!PA=1000\n!PA0=%%75\n");
    else
	sprintf(command,"!PA0=0\n");
    return sendCommand(command);
}

bool PushBot::switchLED(bool on) 
{
    char command[MAX_COMMAND_LENGTH];
    sprintf(command, "!PC=10\n");
    sendCommand(command);
    if (on) 
        sprintf(command, "!PC0=%%100\n");
    else 
        sprintf(command, "!PC0=%%0\n");
    return sendCommand(command);
}


void PushBot::processKeys(int keysPressed) {
    switch( keysPressed )
    {
        case STOP :
            std::cout << "Stop" << std::endl;
            setLeftVelocity(0);
            setRightVelocity(0);
            break;
        case FORWARD :
            std::cout << "Forward" << std::endl;
            setLeftVelocity(SPEED);
            setRightVelocity(SPEED);
            break;
        case BACKWARD :
            std::cout << "Backward" << std::endl;
            setLeftVelocity(-SPEED);
            setRightVelocity(-SPEED);
            break;
        case LEFT : 
            std::cout << "Left" << std::endl;
            setLeftVelocity(0);
            setRightVelocity(SPEED);
            break;
        case RIGHT : 
            std::cout << "Right" << std::endl;
            setLeftVelocity(SPEED);
            setRightVelocity(0);
            break;
        case TURN_CCW : 
            std::cout << "Turn CCW" << std::endl;
            setLeftVelocity(-SPEED);
            setRightVelocity(SPEED);
            break;
        case TURN_CW : 
            std::cout << "Turn CW" << std::endl;
            setLeftVelocity(SPEED);
            setRightVelocity(-SPEED);
            break;
        default :
            std::cout << "ERROR: Unknown Command" << std::endl;
    }
}

void PushBot::parse(unsigned char* data, int length) 
{
   std::cout<<"Parse is not implemented for PushBot"<<std::endl;
}

void PushBot::move(int forwardVelocity, int angularVelocity) 
{
    // we don't know yet the angular wheel velocity unit
    // However: wheel distance from each other is 8.6cm
    double wheelDistance = 8.6;
    // wheel radius is 1.9cm
    double wheelRadius = 1.9;

    // so we get for v_r and v_l wheel velocities
    // and v and \omegadot state representation variables
    // v: forward velocity
    // \omega: heading direction in external system, mathematical direction
    // --> \dot{\omega}: angular velocity
    // and r wheel radius and d wheel distance
    // v = r/2 (v_r + v_l)
    // \dot{\omega} = d*r/2 (v_r - v_l)
    // < = >
    // v_r = v/r + \dot{\omega}/(d*r)
    // v_l = v/r - \dot{\omega}/(d*r)

    double rightWheelVelocity = forwardVelocity / wheelRadius + 
        angularVelocity / (wheelDistance * wheelRadius);
    double leftWheelVelocity = forwardVelocity / wheelRadius -
        angularVelocity / (wheelDistance * wheelRadius);

    // set PushBot, round velocities
    _rightSpeed = (int)(rightWheelVelocity + 0.5);
    _leftSpeed = (int)(leftWheelVelocity + 0.5);
    setRightVelocity(_rightSpeed);
    setLeftVelocity(_leftSpeed);

    // save parameters
    _forwardVelocity = forwardVelocity;
    _angularVelocity = angularVelocity;

    // notify listeners
    PushbotSignals state;
    state.left = _leftSpeed;
    state.right = _rightSpeed;
    std::list<RobotListener*>::iterator it;
    for(it = m_listeners.begin(); it!=m_listeners.end(); it++) {
        // notify listener
        (*it)->receivedNewServoEvent(state);
    }
}

void PushBot::setSpeed(int forwardVelocity) 
{
    move(forwardVelocity, _angularVelocity);
}

void PushBot::setAngularVelocity(int angularVelocity) 
{
    move(_forwardVelocity, angularVelocity);
}

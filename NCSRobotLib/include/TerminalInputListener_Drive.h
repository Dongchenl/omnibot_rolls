#ifndef TERMINALINPUT_LISTENER_DRIVE_H
#define TERMINALINPUT_LISTENER_DRIVE_H

#include "TerminalInputListener.h"
#include "Robot.h"
#include "Enums.h"

/*! \file TerminalInputListener_Drive.h
 *  \brief This file provides all the functions with respect to the keyboard listener.
 */
class TerminalInputListener_Drive : public TerminalInputListener
{
public:
    TerminalInputListener_Drive();
    void receivedNewTerminalInputEvent(char read_char);
    robotDriveParameters*   driveParams;
    void addRobot( Robot * );
    bool isEnabled();
private :
    bool    enable;
    int     keysPressed;
    Robot * robot;
};

#endif

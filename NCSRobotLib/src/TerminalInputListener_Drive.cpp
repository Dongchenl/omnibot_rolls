#include "TerminalInputListener_Drive.h"
#include <stdio.h>

TerminalInputListener_Drive::TerminalInputListener_Drive()
    :   enable(false),
        keysPressed(0)
{
}

void TerminalInputListener_Drive::addRobot( Robot * robot )
{
    this->robot = robot;
}

void TerminalInputListener_Drive::receivedNewTerminalInputEvent( char read_char )
{
    // bit order: NA NA E Q D A S W
    switch(read_char)
    {
    case 'w' : // Forward
        keysPressed = 1 << W_SHIFT;
        break;
    case 's' : // Backward
        keysPressed = 1 << S_SHIFT;
        break;
    case 'a' : // Turn CCW
        keysPressed = 1 << Q_SHIFT;
        break;
    case 'd': //Turn CW
        keysPressed = 1 << E_SHIFT;
        break;
    case 'e':  //Stop
        keysPressed = 0;
        break;
    }

    robot->processKeys(keysPressed);
}

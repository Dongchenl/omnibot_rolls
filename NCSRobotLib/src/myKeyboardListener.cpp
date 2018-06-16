// for debugging
#include <bitset>
#include <iostream>

#include "myKeyboardListener.h"
#define VERBOSE /* comment this line to suppress output */

myKeyboardListener::myKeyboardListener()
    :   enable(false),
        keysPressed(0)
{
}

void myKeyboardListener::addRobot( Robot * robot )
{
    this->robot = robot;
}

void myKeyboardListener::receivedNewKeyboardEvent( input_event* keyboard_ev )
{
    // keyboard_ev->value < 2 ensure s that the keyboard keays just have the states PRESSED and RELEASED
    if (keyboard_ev->type == EV_KEY && keyboard_ev->value >= 0 && keyboard_ev->value < 2) {
        
        // bit order: NA NA E Q D A S W
        switch(keyboard_ev->code)
        {
        case KEY_W : // Forward
            keysPressed ^= 1 << W_SHIFT;
            break;
        case KEY_S : // Backward
            keysPressed ^= 1 << S_SHIFT;
            break;
        case KEY_A : // Left
            keysPressed ^= 1 << A_SHIFT;
            break;
        case KEY_D : // Right
            keysPressed ^= 1 << D_SHIFT;
            break;
        case KEY_Q : // Turn CCW
            keysPressed ^= 1 << Q_SHIFT;
            break;
        case KEY_E : //Turn CW
            keysPressed ^= 1 << E_SHIFT;
            break;
        }

        robot->processKeys(keysPressed);
    }
}

#ifndef MY_KEYBOARD_LISTENER_H
#define MY_KEYBOARD_LISTENER_H

#include "keyboard.h"
#include "Robot.h"

#define W_SHIFT 0
#define S_SHIFT 1
#define A_SHIFT 2
#define D_SHIFT 3
#define Q_SHIFT 4
#define E_SHIFT 5

#define STOP            0
#define FORWARD         1
#define BACKWARD        2
#define LEFT            4
#define RIGHT           8
#define TURN_CCW        16
#define TURN_CW         32
#define FWR_ARC_RIGHT   9
#define FWR_ARC_LEFT    5
#define BWR_ARC_RIGHT   6
#define BWR_ARC_LEFT    10

struct robotDriveParameters {
    double  angular_vel;
    bool    CCW;
    double  linear_vel;
    double  direction;
};

class myKeyboardListener : public KeyboardListener
{
    public:
        myKeyboardListener();
        void receivedNewKeyboardEvent( input_event* );
        robotDriveParameters*   driveParams;
        void addRobot( Robot * );
        bool isEnabled();
    private :
        bool                    enable;
        int                     keysPressed;
        Robot *         robot;
};

#endif

#ifndef ENUMS_H
#define ENUMS_H

/*! \enum bitShifts
 *  \brief Mapping of the keystrokes
 * 
 * Pressing a key results in setting a bit at the corresponding position in a int variable. This allows filtering keys with no mapping and storing the state of the key and detecting combinations of keys. Key presses map to bit = 1 and key releases to bit=0 at the respective bit position of the corresponding key as defined by bitShifts.
 *  \sa mappedKeys
 */
enum bitShifts
{
    W_SHIFT = 0, //<! 
    S_SHIFT = 1, //<! 
    A_SHIFT = 2, //<! 
    D_SHIFT = 3, //<! 
    Q_SHIFT = 4, //<! 
    E_SHIFT = 5  //<! 
};

/*! \enum mappedKeys
 *  \brief Mapping of the keystrokes to a more readably form.
 */
enum mappedKeys
{
    _KEY_W = (1 << W_SHIFT), //<!
    _KEY_S = (1 << S_SHIFT), //<!
    _KEY_A = (1 << A_SHIFT), //<!
    _KEY_D = (1 << D_SHIFT), //<!
    _KEY_E = (1 << E_SHIFT), //<!
    _KEY_Q = (1 << Q_SHIFT)  //<!
};

/*! \enum robotDirectives
 *  \brief Mapping of the keystrokes to drive directives for the robot
 */
enum robotDirectives
{
    STOP     = 0,                       //<!
    FORWARD  = _KEY_W,                  //<!
    BACKWARD = _KEY_S,                  //<!
    LEFT     = _KEY_A,                  //<!
    RIGHT    = _KEY_D,                  //<!
    TURN_CCW = _KEY_Q,                  //<!
    TURN_CW  = _KEY_E,                  //<!
    FWR_ARC_RIGHT = _KEY_W + _KEY_D,    //<!
    FWR_ARC_LEFT  = _KEY_W + _KEY_A,    //<!
    BWR_ARC_RIGHT = _KEY_S + _KEY_D,    //<!
    BWR_ARC_LEFT  = _KEY_S + _KEY_A     //<!
};

struct robotDriveParameters {
    double  angular_vel;
    bool    CCW;
    double  linear_vel;
    double  direction;
};

#endif

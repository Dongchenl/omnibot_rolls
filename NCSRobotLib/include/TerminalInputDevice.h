#ifndef TERMINAL_INPUT_DEVICE_H
#define TERMINAL_INPUT_DEVICE_H

#include <list>
#include <aerctl.h>
#include "TerminalInputListener.h"

/*! \file TerminalInputDevice.h
 *  \brief This file contains all the necessary interfaces  to log keystrokes from the keyboard. One drawback is 
 * that all keystrokes are logged, even if the window is not in focus.
 */


/*! \class TerminalInput
 *  \brief Manages the communication with an attached keyboard.
 * 
 * cKeyboard manages the communication with an attached keyboard. It is assumed that the attached keyboard has the expression "kbd" in its name. This class can detect key releases and key presses.
 */
class TerminalInput
{
public:
   TerminalInput();
    ~TerminalInput();
    static void* loop(void* obj);
    void  readEv();  

    // Listener stuff
    void registerListener( TerminalInputListener * );
    void deregisterListener( TerminalInputListener * );
//    void warnEvent( input_event* ); 
    void listen();  
    void  stopListening(); 
    bool isListening(); 

private:
    bool            m_toggleListening; //!< Flag indicating if we are listening or not
    pthread_t       thread;
    char name[256];
    char read_char; 
    std::list<TerminalInputListener *> m_listeners; //!< List of the registered listeners
		std::list<TerminalInputListener *>::iterator it_list; 
};

#endif

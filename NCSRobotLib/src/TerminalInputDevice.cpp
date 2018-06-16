#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <list>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <termios.h>
#include <dirent.h>

#include "TerminalInputDevice.h"

/*! \file TerminalInputDevice.cpp
 *  \brief This file provides the implementation of the keyboard communication
 */


static struct termios old_set, new_set;


TerminalInput::TerminalInput()
{  
	m_toggleListening = false;
	int echo =1;
  tcgetattr(0, &old_set); /* grab old terminal i/o settings */
  new_set = old_set; /* make new settings same as old settings */
  new_set.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_set.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_set); /* use these new terminal i/o settings now */
}

TerminalInput::~TerminalInput() 
{
    tcsetattr(0, TCSANOW, &old_set);
}

void* TerminalInput::loop(void* obj)
{
    while (reinterpret_cast<TerminalInput*>(obj)->m_toggleListening)
    {
      reinterpret_cast<TerminalInput*>(obj)->readEv();
   //     reinterpret_cast<cKeyboard*>(obj)->warnEvent( reinterpret_cast<cKeyboard*>(obj)->keyboard_ev );
    }
    pthread_exit(NULL);
}

void TerminalInput::readEv()
{
	std::cout<<"Reading event from Terminal"<<std::endl;
  int r;
	if ((r = read(0, &read_char, sizeof(read_char))) < 0)	
	{
		std::cout<<"Didn't get any event"<<std::endl;
	}
	else
	{
		std::cout<<"Got event: "<< read_char <<std::endl; 
    for(it_list = m_listeners.begin(); it_list!=m_listeners.end(); it_list++)
    {
       (*it_list)->receivedNewTerminalInputEvent(read_char);
    }
		if(read_char == 'q')
		{
			std::cout<<"Stop listening in TerminalInput!"<<std::endl;
			m_toggleListening = false;
      tcsetattr(0, TCSANOW, &old_set);
		std::terminate();
		}
	}
}

void TerminalInput::registerListener(TerminalInputListener* listener) {
    m_listeners.push_back(listener);
}

void TerminalInput::deregisterListener(TerminalInputListener* listener) {
    m_listeners.remove(listener);
}

//void cKeyboard::warnEvent( input_event* event ) {
//    std::list<KeyboardListener*>::iterator it;
//    for(it = m_listeners.begin(); it!=m_listeners.end(); it++)
//    {
//            (*it)->receivedNewKeyboardEvent( event );
//    }
//}

void  TerminalInput::listen() 
{
     m_toggleListening = true;
     pthread_create(&thread, NULL, &TerminalInput::loop, this);
}

bool TerminalInput::isListening()
{
    return m_toggleListening;
}

void  TerminalInput::stopListening( void ) 
{
    m_toggleListening = false;
    pthread_join(thread, 0);
  	tcsetattr(0, TCSANOW, &old_set);
    std::cout << "Stop listening" << std::endl;
}

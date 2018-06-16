#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <linux/input.h>
#include <unistd.h>
#include <list>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstring>
#include <termios.h>

#include "keyboard.h"

#define KEYBOARD_DEV "/dev/input/by-path/platform-i8042-serio-0-event-kbd"

static struct termios old_set, new_set;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old_set); /* grab old terminal i/o settings */
  new_set = old_set; /* make new settings same as old settings */
  new_set.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_set.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_set); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old_set);
}

cKeyboard::cKeyboard() :
    m_toggleListening(false),
    keyboard_fd(0)
{
    initTermios(0);
    keyboard_ev = new input_event();
    keyboard_st = new keyboard_state();
    
    keyboard_fd = open(KEYBOARD_DEV, O_RDONLY); //  | O_NONBLOCK <-- doesn't seem to work ...
    if (keyboard_fd == -1) {
        fprintf(stderr, "Cannot open %s: %s.\n", KEYBOARD_DEV, strerror(errno));
    } else 
        std::cout << "Successfully opened keyboard device" << std::endl;
}

cKeyboard::~cKeyboard() {
    resetTermios();
    if (keyboard_fd > 0) {
            close(keyboard_fd);
    }
    delete keyboard_st;
    delete keyboard_ev;
    keyboard_fd = 0;
    m_toggleListening = false;
}

void* cKeyboard::loop(void *obj) {
    while (reinterpret_cast<cKeyboard *>(obj)->m_toggleListening){
        reinterpret_cast<cKeyboard *>(obj)->readEv();
        reinterpret_cast<cKeyboard *>(obj)->warnEvent( reinterpret_cast<cKeyboard *>(obj)->keyboard_ev );
    }
}

void cKeyboard::readEv()
{
    int bytes = read(keyboard_fd, keyboard_ev, sizeof(input_event) );
    if (bytes > 0) {
        if (keyboard_ev->type == EV_KEY && keyboard_ev->value >= 0 && keyboard_ev->value <= 2  ) {
            keyboard_st->key_states[keyboard_ev->code] = keyboard_ev->value;
        }
    }
    
}

short cKeyboard::getKeyState(short key) {
    return keyboard_st->key_states[key];
}

void cKeyboard::registerListener(KeyboardListener* listener) {
    m_listeners.push_back(listener);
}

void cKeyboard::deregisterListener(KeyboardListener* listener) {
    m_listeners.remove(listener);
}

void cKeyboard::warnEvent( input_event* event ) {
    std::list<KeyboardListener*>::iterator it;
    for(it = m_listeners.begin(); it!=m_listeners.end(); it++)
    {
            (*it)->receivedNewKeyboardEvent( event );
    }
}

int  cKeyboard::listen( void ) {
    if (keyboard_fd > 0) {
            m_toggleListening = true;
            
            if( ioctl(keyboard_fd, EVIOCGNAME(256), name) ){
                std::cout << "   Name: " << name << std::endl;
                pthread_create(&thread, 0, &cKeyboard::loop, this);
            }
            else
                fprintf(stderr, "IOCTL fail: %d %s.\n", errno, strerror(errno));
    }
}

bool cKeyboard::isListening()
{
    return m_toggleListening;
}

int  cKeyboard::stopListening( void ) {
    if (keyboard_fd > 0) {
            m_toggleListening = false;
            pthread_join(thread, 0);
            resetTermios();
            std::cout << "Stop listening" << std::endl;
    }
}

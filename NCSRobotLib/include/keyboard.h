#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <linux/input.h>
#include <list>

class KeyboardListener
{
    public:
        virtual void receivedNewKeyboardEvent( input_event* ) = 0;
};

struct keyboard_state {
	unsigned short key_states[KEY_CNT];
};

class cKeyboard {
    private:
        bool            m_toggleListening;
        pthread_t       thread;
        int             keyboard_fd;
        input_event*    keyboard_ev;
        keyboard_state* keyboard_st;
        char name[256];
        
        // Registered listeners
        std::list<KeyboardListener *> m_listeners;
        

    protected:
    public:
        cKeyboard();
        ~cKeyboard();
        static void* loop(void* obj);
        void  readEv();
        short getKeyState(short key);

        // Listener stuff
        void registerListener( KeyboardListener * );
        void deregisterListener( KeyboardListener * );
        void warnEvent( input_event* ); 
        int  listen();
        int  stopListening();
        bool isListening();
};

#endif

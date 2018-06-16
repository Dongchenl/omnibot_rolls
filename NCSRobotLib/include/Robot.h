#ifndef ROBOT_H
#define ROBOT_H

#define UPDATE_TIME 1000

#include <queue>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <mutex>
#include <pthread.h>

#include <RobotListener.h>

// x, y are m/s
// theta is deg/s
// CCW means counter clock-wise
struct RobotTwist {
    float x, y, theta;
    RobotTwist(float x, float y, float theta)
        : x(x),
          y(y),
          theta(theta)
    {};
};

class Robot {
    public:
        Robot(int fileDescriptor);
        ~Robot();

        void registerListener( RobotListener * );
        void deregisterListener( RobotListener * );
        // Listener to receive the new events
        int listen(void);
        int stopListening(void);
        int close();

        virtual bool move(RobotTwist twist) {return true;};
        virtual bool twitch() {return true;};
        virtual void processKeys(int keysPressed) {};
				virtual void parse(unsigned char* data, int bytesRead) {};

				pthread_mutex_t p_usb_mutex;
    protected:
        // Warn all the listeners that an event has occurred
        void warnEvent( std::queue<servoSignals>&, std::queue<bumper>& );
        void warnEventBumper(unsigned int bumper);
        void warnEventOmniRobotPos(double xpos, double ypos);

        // Threading for listening
        boost::thread*               readThread;
        int                          m_readThreadStatus;
        boost::asio::io_service*     ios;
        boost::asio::deadline_timer* readTimer;
        static void                  threadLoop(boost::asio::deadline_timer* , int*, void*);
        static const int TERMINATE_THREAD = -1;
        static const int RUN_THREAD = 1;
        bool   m_toggleListening;
        std::list<RobotListener*> m_listeners; /* Registered listeners */

	int sendCommand(std::string cmd, bool verbose=true, std::mutex * usb_mutex=NULL);

        int m_status;
				int fileDescriptor;

	void readSocket();

        // TODO try to get rid of
        int* count;
};

#endif

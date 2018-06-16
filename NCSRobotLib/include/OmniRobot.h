// protoptypes for the OmniRobot communication and low-level commands

#ifndef OMNIROBOT_H
#define OMNIROBOT_H

#include <stdio.h>
#include <termios.h>
#include <string>
#include <vector>
#include <queue>
#include <list>
#include <mutex>
//#include "/home/mario/Semester_Project/src/NCSRobotExamples/omnibot_parallella_mario/include/Robot.h"
#include <Robot.h>
//#include "/home/mario/Semester_Project/src/NCSRobotExamples/omnibot_parallella_mario/include/RobotListener.h"
#include <RobotListener.h>
#include <myKeyboardListener.h>
//#include <QUdpSocket>
#include <fstream>

#include <cmath>

class OmniRobot;

struct robotDriveState
{
    double linear_vel;
    bool   CCW;
    double angular_vel;
		double direction;
    robotDriveState()
        : linear_vel(0),
 					CCW(true),
          angular_vel(0),
	        direction(0)
    	{};
    robotDriveState(double anAngularVel, bool aCCW, double aLinVel, double aDirection)
        : linear_vel(aLinVel),
          CCW(aCCW),
          angular_vel(anAngularVel),
          direction(aDirection)
    	{};
};

struct robotPosition
{
  float posX;
  float posY;
  float heading;
};
/*! \struct OmnibotArgs
 *   \brief encapsulates the necessary arguments to start reading from the serial port in a new Threading
 */
struct OmnibotArgs
{
    OmniRobot*                              omnirobot;
    int*                                    thread_status;
    int*                                    file_descriptor;
    std::vector<Event>*       	            events;
};

class OmniRobot : public Robot {
public:
    OmniRobot(int fileDescriptor) : Robot(fileDescriptor) {
    encoderDeltaBuffer.push_back({{0,0,0},0,0});
    encoderSpeedsBuffer.push_back({{0,0,0},0,0});
    encoderBuffer.push_back({{0,0,0},0,0});
    currentEncoderState={{0,0,0},0,0};
    };

    // test OmniRob when first connecting
    bool twitch();

    void processKeys(int keysPressed);

    //! Make the robot move
    /*!
    * The drive function sets the servo speeds of the robot according to the
    * specified input.
    * \param angular_vel can take values from 0 to 100, indicating the speed of the servo (0 = off, 100 = maximum). Determines the turning rate of the robot.
    * \param CCW determines if the servo is turning CCW or CW
    * \param linear_vel can take values from 0 to 100, indicating the speed of
    * the servo (0 = off, 100 = maximum). Determines the linear velocity of
    * the robot.
    * \param direction in degrees. Sets the direction where the robot is
    * headed. NB: The driving vector is in polar coordinates.
    * \param delta_t in milliseconds. Sets the time the servos are on.
    * Internally boost::asio::deadline_timer is used.
    */
		void drive( double angular_vel, bool CCW,
		            double linear_vel,
		            double direction, double delta_t);
    void drive(double angular_vel, bool CCW, double linear_vel, double direction);
		void drive();
		void drive_random();
		void drive_random(int bumper_signal); // was const char*
		void drive_dist(double angular_vel, bool CCW, double linear_vel,
		                double direction, double distance);
		void drive_global(double angular_vel, bool CCW, double linear_vel,
		                  double direction, double heading);
		void drive_global_rot(double angular_vel, bool CCW, double linear_vel,
		                      double direction, double heading_start);
    void emergencyStop();
    // Uncommented because Qt not yet installed on Raphaelas parallella
//    void visualize( QUdpSocket* socket, struct encoderStates deltaEnc,
//                    struct bumper bumperValue, int pointnumber );
//    void visualize(QUdpSocket* socket, int i);
    void pathIntegration();
    void auxPathIntegration();
    void log();
    void logInit();

    /* getter functions */
    void getServoPosition(int servoID, int * signal);
    void getServoSpeed(int servoID, int * signal);
    bool isBumperHit();
		void getBumperSignal();
    int  getbatteryVoltage();
		void getEncoderSignals();

    /* setter functions */
    void setRobotDriveState( const robotDriveState& ); 
    void setServoSpeed( int servoID, int signal );
    void setServoSpeeds( double * servosignals );
    void setServoTorque( int servoID, bool enable, int signal );
    
    void parseEncAndBumperEvents();
    void computeEncoderDeltas();
		int  listen( void );
		int  stopListening(void);

		bool isNumber(std::string& str);
		void notifyListeners(unsigned int bumper);
		void warnPosTest( double xpos, double ypos);

		std::vector<Event>  m_events; // A vector with events
		                              // (to select encoder values)
		std::list<Event>	m_event_list; // A list with events
		                                // (allows accessing and pushing from both
		                                // ends efficiently)
		std::mutex	event_mutex; // mutex object to lock simultaneous access
		                         // to event_list
		std::mutex	usb_mutex;
		struct servoSignals currentDriveCommands;
		struct bumper currentBumperVal;
		robotPosition currentGlobalPosition;
		robotPosition auxCurrentGlobalPosition;
    robotPosition currentPosition;
    robotPosition auxCurrentPosition;
    bool lagFlag;

private:
    int logNumber;
    int batteryVoltage;
    int servoSpeeds[3];
		int encoderSignals[3];
    robotDriveState state;

    struct encoderStates currentEncoderState;
    std::list<encoderStates> encoderBuffer;
    std::vector<encoderStates> encoderDeltaBuffer;
    std::vector<encoderStates> encoderSpeedsBuffer;

    std::list<RobotListener*> m_listeners;

		// Device state
    //int         m_status;  //!< initialization status of the serial Port, not yet implemented
    bool        m_started_omnibot; //!< holds true if the omnirobot was successfully started, else false
    //bool        m_toggle_listening_omnibot; //!< holds true if listening is enabled, else false

    // Device
    int m_file_descriptor; //!< file descriptor of the serial port

    // Threading
    pthread_t                    m_readthread[3]; //!< three threads to outsource the reading, querying and parsing/controlling operations
    int                          m_readthreads_status; //!< flag to indicating the status of the thread, holds the values -1 when stopped and 1 when running
		int													 m_thread_id;

		struct OmnibotArgs 	m_readthread_args; //needed to have args available from outside thread
		//std::vector<Event>  m_events; // A vector with events (to select encoder values)

		// Files for logging
		std::ofstream visLogFile;
		std::ofstream readLogFile;
		std::ofstream deltaLogFile;
};

static speed_t getBaudrate(unsigned int);

#endif

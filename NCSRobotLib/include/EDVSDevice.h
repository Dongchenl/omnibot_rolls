#ifndef EDVSDEVICE_H
#define EDVSDEVICE_H

#include <stdio.h>
#include <termios.h>
#include <string>
#include <vector>
#include <list>
#include <pthread.h>
#include "EDVSListener.h"

/*! \file EDVS_DEVICE.h
 *  \brief provides the necessary interface for communication and listening to the eDVS camera and the embedded IMU
 * 
 */

class EDVSDevice;


/*! \enum biases_t
 *  \brief defines the biases necessary for proper eDVS initialization
 * 
 *  \sa here for more information: http://inilabs.com/support/hardware/biasing/#h.khgrv1ecqm5q
 */
enum biases_t{
    CAS,        //!< First stage cascode
    INJ_GND,    //!< Injected ground
    REQ_PD,     //!< Pull down on chip request
    PU_X,       //!< Pull up on request from X arbiter
    DIFF_OFF,   //!< Threshold for Off events
    REQ,        //!< Pull down for passive load inverters in digital AER pixel circuitry
    REFR,       //!< Refractory period
    PU_Y,       //!< Pull up on request from X arbiter
    DIFF_ON,    //!< Threshold for On events
    DIFF,       //!< Second stage (“Differential”)
    FOLL,       //!< Source follower separating first and second stages
    PR          //!< First stage (“Photoreceptor”)
};

/*! \struct EDVSBiases
 *  \brief  encapsulates the values for the biases and stores them in an array with the help of the biases_t enum for better readability.
 * 
 *  \sa biases_t
 */

struct EDVSBiases
{
    EDVSBiases()
    {
        biases[CAS]      = 1067;
        biases[INJ_GND]  = 12315;
        biases[REQ_PD]   = 16777215;
        biases[PU_X]     = 5579731;
        biases[DIFF_OFF] = 999;
        biases[REQ]      = 427594;
        biases[REFR]     = 0;
        biases[PU_Y]     = 16777215;
        biases[DIFF_ON]  = 567391;
        biases[DIFF]     = 19187;
        biases[FOLL]     = 39;
        biases[PR]       = 4;
    }
    
    int biases[12]; //!< array with the biases, can be convientently accessed with the biases_t enum.
};



/*! \enum EventFormat
 *  \brief provides all the event formats that are supported by this camers.
 * 
 *  \sa here for a more detailed explanation: http://inilabs.com/support/hardware/edvs/#h.az1ycqjvj6jm
 */
enum EventFormat
{
    EVT_FORMAT_E0, //!< 2 Bytes per evt (no timestamp)
    EVT_FORMAT_E1, //!< 3..5 bytes per event, 1..3 bytes delta-timestamp (7bits each)
    EVT_FORMAT_E2, //!< 4 Bytes per evt (16 bits ts, 1us res)
    EVT_FORMAT_E3, //!< 5 Bytes per evt (24 bits ts, 1us res)
    EVT_FORMAT_E4, //!< 6 Bytes per evt (32 bits ts, 1us res)
};

/*! \struct EDVSArgs
 *   \brief encapsulates the necessary arguments to start reading from the serial port in a new Threading
 * 
 *   \sa readThreadEDVS4337
 */
struct EDVSArgs
{
    EDVSDevice*                             device;
    int*                                    threadStatus;
    int*                                    ttyFd;
    std::list<EDVSEvent>*                   events;
    EventFormat*                            eventFormat;
};

 //! Class providing the necessary interfaces for communication and event streaming from the eDVS camera
 /*! 
 * The eDVS camera is fitted with a FTDI chip, such that the camera can be opened and handeled like a serial port, despite the connection is established via USB.
 */
class EDVSDevice
{

public:
    static const int TERMINATE_THREAD = -1; //!< Flag indicating when to stop the reading thread
    static const int RUN_THREAD       =  1; //!< Flag indicating when to start the reading thread

    //! constructor of the EDVS4337SerialUSB
    /*!
     * \param int valid file descriptor to the eDVS camera
     */
    EDVSDevice( int );
    ~EDVSDevice();

    // Step by step life of a DVS128USB object
    //! starts the eDVS by setting the necessary biases
    int startEDVS();        
    //! stops the started eDVS and the IMU
    int stopEDVSAndIMU();   
    //! start listening to the events parsed from the event stream
    int listen();   
    //! stop listening to the events parsed from the event stream
    int stopListening();
    //! checks if the camera is correctly initialized
    /*!
     * \returns 1 if the camera has been corectly initialized, else -1
     */
    int isInit();           
    //! closes the serial Port to the eDVS camera
    int close();            
    //! changes the event format of the events
    /*!
     * \sa EventFormat for the supported events
     */
    int changeEventFormat( EventFormat ); 

    //! send biases to the camera
    /*!
     * \param EDVSBiases b is struct with the values of the biases to be set.
     * \sa biases_t for all the biases that can be manipulated
     */
    int sendBiases( EDVSBiases b = EDVSBiases() ); 
    EDVSBiases getBiases(); //!< get the current biases

    //! Register an event listener
    void registerListener( EDVSListener* listener ); 
    //! deregister a specific event listener.
    /*! 
     * \param listener specifies the listener to be removed
     */
    void deregisterListener( EDVSListener* listener ); 

    // Register an IMU event listener
    // deregister a specific IMU event listener.
    /* 
     * \param listener specifies the listener to be removed
     */
    //void registerIMUListener( EDVS4337SerialUSBIMUListener* listener );
    //void deregisterIMUListener( EDVS4337SerialUSBIMUListener* listener );

    //! Notify all the event listeners that an event as occured
    void warnEvent( std::vector<EDVSEvent>& events );

    //! Notify all the IMU event listeners that an event as occured
    void warnIMUEvent( std::vector<EDVSIMUEvent>& events );

    //! starts the IMU by sending the necessary commands
    void startIMU( EDVSIMUEvent::IMUType , int );
    //! starts the IMU by sending the necessary commands
    void stopIMU( EDVSIMUEvent::IMUType );

private:

    // Device state
    int         m_status;  //!< initialization status of the serial Port
    bool        m_started; //!< holds true if the eDVS was successfully started, else false
    bool        m_toggleListening; //!< holds true if listening is enabled, else flase
    EventFormat m_eventFormat;     //!< holds the current event format
    EDVSBiases m_biases; //!< holds the last used bias settings

    // Device
    int m_ttyFd; //!< file descriptor of the serial port
//     struct termios m_oldStdio;

    // Threading
    pthread_t                    m_readThread; //!< a thread to outsource the reading operations
    int                          m_readThreadStatus; //!< flag to indicating the status of the thread, holds the values -1 when stopped and 1 when running
    struct EDVSArgs m_readThreadArgs; //!< arguments necessary to outsource the reading to a new thread.

    std::list<EDVSEvent>   m_events; //!< a list with the last parsed events

    std::list<EDVSListener*> m_listeners; //!< list with the registered event listeners
  //  std::list<EDVS4337SerialUSBIMUListener*> m_imuListeners; //!< list with the registered IMU event listeners

    /*!
     * \param cmd has to be a valid command in form of a string.
     * \sa http://inilabs.com/support/hardware/edvs/#h.bctg2sgwitln
     */
    int sendCommand( std::string cmd );

};

#endif

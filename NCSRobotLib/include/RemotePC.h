#ifndef REMOTEPC_H
#define REMOTEPC_H

#include <stdio.h>
#include <termios.h>
#include <string>
#include <vector>
#include <list>
#include <pthread.h>
#include "PCListener.h"

/*! \file REMOTEPC.h
 *  \brief provides the necessary interface for communication and listening to the remote PC
 * 
 */

class RemotePC;


/*! \struct EDVSArgs
 *   \brief encapsulates the necessary arguments to start reading from the serial port in a new Threading
 * 
 *   \sa readThreadEDVS4337
 */
struct PCArgs
{
    RemotePC*                             device;
    int*                                    threadStatus;
    int*                                    ttyFd;
    std::vector<PCEvent>*                   events;
    //EventFormat*                            eventFormat;
};

 //! Class providing the necessary interfaces for communication and event streaming from the eDVS camera
 /*! 
 * The eDVS camera is fitted with a FTDI chip, such that the camera can be opened and handeled like a serial port, despite the connection is established via USB.
 */
class RemotePC
{

public:
    static const int TERMINATE_THREAD = -1; //!< Flag indicating when to stop the reading thread
    static const int RUN_THREAD       =  1; //!< Flag indicating when to start the reading thread

    //! constructor of the EDVS4337SerialUSB
    /*!
     * \param int valid file descriptor to the eDVS camera
     */
    RemotePC( int );
    ~RemotePC();
    
    int startRemotePC();
    //! start listening to the PC parsed from the stream
    int listen();   
    //! stop listening to the pC parsed from the stream
    int stopListening();        
    int isInit();           
    //! closes the serial Port to the remote PC
    int close();            

    //! Register an event listener
    void registerListener( PCListener* listener ); 
    //! deregister a specific event listener.
    /*! 
     * \param listener specifies the listener to be removed
     */
    void deregisterListener( PCListener* listener ); 

    //! Notify all the event listeners that an event as occured
    void warnEvent( std::vector<PCEvent>& events );

    int sendServer( int cmd );

private:

    // Device state
    int         m_status;  //!< initialization status of the serial Port
    bool        m_started; //!< holds true if the eDVS was successfully started, else false
    bool        m_toggleListening; //!< holds true if listening is enabled, else flase
    //EventFormat m_eventFormat;     //!< holds the current event format
    // Device
    int m_Fd; //!< file descriptor of the serial port
//     struct termios m_oldStdio;

    // Threading
    pthread_t                    m_readThread; //!< a thread to outsource the reading operations
    int                          m_readThreadStatus; //!< flag to indicating the status of the thread, holds the values -1 when stopped and 1 when running
    struct PCArgs m_readThreadArgs; //!< arguments necessary to outsource the reading to a new thread.

    std::vector<PCEvent>   m_events; //!< a list with the last parsed events

    std::list<PCListener*> m_listeners; //!< list with the registered event listeners
  //  std::list<EDVS4337SerialUSBIMUListener*> m_imuListeners; //!< list with the registered IMU event listeners

    /*!
     * \param cmd has to be a valid command in form of a string.
     * \sa http://inilabs.com/support/hardware/edvs/#h.bctg2sgwitln
     */
    

};

#endif

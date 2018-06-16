#ifndef EDVSLISTENER_H
#define EDVSLISTENER_H

/*! \stuct EDVSEvent
 *  \brief encapsulates the camera events parsed from the event stream coming from the eDVS.
 */
 struct EDVSEvent
 {
		 public:
			  EDVSEvent()
				 	: m_x(0),
						m_y(0),
						m_pol(0),
						m_timestamp(0),
						m_rawdata()
						{}

				 EDVSEvent( const unsigned int  x,
						  			const unsigned int  y,
							 			const int           pol,
							  		const unsigned long timestamp)
				  						: m_x(x),
										  	m_y(y),
											 	m_pol(pol),
											  m_timestamp(timestamp)
												{}
			 
				  unsigned int    m_x; //!< x position of the pixel that fired
			  	unsigned int    m_y; //!< y position of the pixel that fired
				 	int             m_pol; //!< polarity of the pixel that fired, assumes the values 0 or 1
				  unsigned long   m_timestamp; //!< timestamp set by the eDVS camera
					char            m_rawdata[8]; //!< raw data of the event as from the eDVS event stream
};

/*! \struct EDVSIMUEvent
 *  \brief encapsulates the IMU events parsed from the event stream coming from the eDVS.
 */ 
struct EDVSIMUEvent
{
public:
    enum IMUType
    {
        IMU_COMPASS,
        IMU_ACCELEROMETER,
        IMU_GYRO,
        IMU_HEADING,
        IMU_UNDEFINED
    };

    EDVSIMUEvent()
        : m_type(IMU_UNDEFINED),
          m_x(0),
          m_y(0),
          m_z(0),
          m_timestamp(0)
    {}

    EDVSIMUEvent(IMUType type,
                              int x, int y, int z,
                              unsigned int timestamp)
        : m_type(type),
          m_x(x),
          m_y(y),
          m_z(z),
          m_timestamp(timestamp)
    {}

   IMUType      m_type;
   int          m_x;
   int          m_y;
   int          m_z;
   unsigned int m_timestamp;
};

/*! \class EDVSListener
 *  \brief abstract base class for a listener that observers eDVS events
 *
 *  needs to be subclassed and the reimplement the abstract function receivedNewEDVS4337SerialUSBEven    t
 */
class EDVSListener
{
	public:
		EDVSListener(void) {}
	  virtual void receivedNewEDVSEvent(EDVSEvent& event) = 0; //!< Is invoked when a new event is     parsed from the event stream.
	  virtual void receivedNewEDVSIMUEvent(EDVSIMUEvent& event) = 0; //!< Is invoked when a new ev    ent is parsed from the event stream.
};


#endif

#ifndef PCLISTENER_H
#define PCLISTENER_H

/*! \stuct PCEvent
 *  \brief encapsulates the camera events parsed from the event stream coming from the PC
 */
 struct PCEvent
 {
		 public:
			  PCEvent()
				 	: m_e(0),
						m_rawdata()
						{}

				 PCEvent( const int  e)
				  						: m_e(e)
				  						{}

			 
				  int    m_e; //!< integer corresponding to drive
					char            m_rawdata[8]; //!< raw data of the event as from the eDVS event stream
};

/*! \class PCListener
 *  \brief abstract base class for a listener that observers PC events
 *
 */
class PCListener
{
	public:
		PCListener(void) {}
	  virtual void receivedNewPCEvent(PCEvent& event) = 0; //!< Is invoked when a new event is     parsed from the event stream.
};


#endif
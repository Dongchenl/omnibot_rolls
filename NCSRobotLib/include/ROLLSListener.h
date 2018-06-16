#ifndef ROLLS_LISTENER_H
#define ROLLS_LISTENER_H

/*! \class ROLLSListener
 *  \brief Provides the necessary interface to Listen to keyboard events
 */
class ROLLSListener {
	public:
		virtual void receivedNewROLLSEvent( unsigned int event ) = 0;
};

#endif

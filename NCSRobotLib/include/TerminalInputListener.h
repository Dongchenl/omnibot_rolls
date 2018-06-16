#ifndef TERMINALINPUT_LISTENER_H
#define TERMINALINPUT_LISTENER_H


/*! \class TerminalInputListener
 *  \brief Provides the necessary interface to Listen to keyboard events
 */
class TerminalInputListener
{
	 public:
			virtual void receivedNewTerminalInputEvent(char read_char) = 0; //!< is emitted from a cKeyboard o   bject when a new keystroke has been registered.
};

#endif

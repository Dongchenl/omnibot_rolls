#ifndef INCLUDE_OMNIROBOTROLLSCONTROLLER_H_
#define INCLUDE_OMNIROBOTROLLSCONTROLLER_H_

#include <ROLLSArchitecture.h>
#include <ROLLSDevice.h>
#include <ROLLSListener.h>
#include <RobotListener.h>
#include <TerminalInputListener.h>
#include <OmniRobot.h>
// To simulate bumper events by pressing a key


namespace pt = boost::posix_time;

class OmniRobotRollsController : public ROLLSListener,
                                 public ROLLSArchitecture,
                                 //public OmniRobot,
                                 public RobotListener,
                                 public TerminalInputListener {
	public:
	
  OmniRobotRollsController(ROLLSDevice *_rolls, OmniRobot *_omnirobot);
	OmniRobotRollsController();
  //~OmniRobotRollsController(){};

  void receivedNewROLLSEvent(unsigned int neuron);
  void receivedNewBumperEvent(unsigned int event_bumper);
  void receivedNewOmniRobotEvent(double xpos, double ypos);
  void receivedNewTerminalInputEvent(char read_char);
  void receivedNewServoEvent(PushbotSignals &cmd);

  void start();
  void stop();

private:

  ROLLSDevice *rolls; //=0;
  OmniRobot *omnirobot; //t=0;
  void create_architecture();

  bool m_status=0;
  int m_currentNeuron;
  // Neuron populations created on the ROLLS chip
  NeuronGroup drive_right, drive_left, drive_f, drive_b, s1, s2, s3, s4, stop_g, col, inh_col, const_exc,s1_2, s2_2, s3_2, s4_2;

  std::ofstream rollsLogFile; // For ROLLS event logging
  std::ofstream terminalLogFile;
  std::string logName;
  std::string terminalLogName;
  pt::ptime startTime;
  bool go;

};

#endif

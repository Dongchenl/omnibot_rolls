#include <unistd.h>  
#include <signal.h> 
#include <termios.h>
#include <pthread.h>
#include <string.h>
#include <iostream>

#include "OmniRobot.h"
#include "ROLLSDevice.h"
#include "ROLLSListener.h"
#include "ROLLSListener_Log.h"

#include "USBConnector.h"
#include <OmniRobotRollsController.h>
#include <TerminalInputDevice.h>
#include <boost/program_options.hpp>
#include <boost/date_time.hpp>
#include <boost/filesystem.hpp>  // for logging


#define PI 3.14159265359

// Set true if to be run on parallella, false if on laptop without ROLLS
#define PARALLELLA true

volatile sig_atomic_t abort_flag = 0;
void aborter(int sig) {  // can be called asynchronously
  abort_flag = 1;  // set flag
}

 struct drive_thread_args {
     bool TERMINATE;
     OmniRobot* omnirobot;
     int file_descriptor;
   };

void *driveSequence(void *arg) {
  struct drive_thread_args *args = (struct drive_thread_args *)(arg);

  int x=0; // Initial heading direction
  int bumperBuffer=0;

  //don't drive for now
  args->omnirobot->drive(0,0,50,x);

  while(true) {
    if (args->TERMINATE) {
      break;
    }
    // report bumper value
    if(args->omnirobot->currentBumperVal.value!=bumperBuffer
        && args->omnirobot->currentBumperVal.value!=0) {
      bumperBuffer=args->omnirobot->currentBumperVal.value;
      std::cout<<"BVal " << args->omnirobot->currentBumperVal.value<<std::endl;

    }

    usleep(50000);
  }
  std::cout<<"Terminating drive sequence"<<std::endl;
  pthread_exit(NULL);
}

namespace pt = boost::posix_time;
namespace po = boost::program_options;


int loopParallella(bool enable_logging,int robotFileDescriptor,bool setup,std::string ROLLS_log)
{
    //Blacklist broken neurons
    std::list<int> blacklist;
    blacklist.push_back(150);
    blacklist.push_back(151);
    blacklist.push_back(32);
  // Initialize and test omnirobot
    OmniRobot omniRobot(robotFileDescriptor);
    //Robot* robot = &omniRobot;
    
    OmniRobot* omnirobot = &omniRobot;

    //Keyboard Listener
    TerminalInput terminput;
    
    omnirobot->currentBumperVal.value=0;
    omnirobot->twitch();
    std::cout << "omnirobot initialized" << std::endl;
    usleep(50000);

    ROLLSDevice *rolls = new ROLLSDevice(blacklist); // with bad neuron list
    std::cout << "ROLLS initialized" << std::endl;

    OmniRobotRollsController rollsController(rolls, &omniRobot); //omnirobot

    // If needed, apply the controllers ROLLS architecture
    if (setup) {
        std::cout<<"Synapse setup started."<<std::endl;
        rollsController.apply(*rolls);
        std::cout<<"Synapse setup done."<<std::endl;
    }
    
    omnirobot->registerListener(&rollsController);
    rolls->registerListener(&rollsController);
    terminput.registerListener(&rollsController);

    omnirobot->listen();
    rolls->listen();
    terminput.listen();
    rollsController.start();

    pt::ptime start = pt::microsec_clock::local_time(); 
    
    ROLLSListener_Log *rolls_logger;    
    if (enable_logging) 
    {
    	rolls_logger = new ROLLSListener_Log(ROLLS_log, start);
	    rolls->registerListener(rolls_logger);
  	}

    // Setting up and starting PID controller and path integration
    struct drive_thread_args args;
    args.TERMINATE = false;
    args.omnirobot = omnirobot; //
    args.file_descriptor = robotFileDescriptor; //

    pthread_t drive_thread[1];
    pthread_create(&drive_thread[0], NULL, driveSequence, (void*)&args);

    std::cout<<"Starting loop"<<std::endl;

    while (true) {
        usleep(1);
      // React to abort signal
        if (abort_flag) {
          std::cout << "abort signal caught!" << std::endl;
          break;
        }
      }

    // On abort do the following:
    terminput.stopListening();
    args.TERMINATE = true;
    pthread_join(drive_thread[0], NULL);
    omnirobot->emergencyStop(); // First, stop the robot
    omnirobot->stopListening(); // Then, stop all listening threads
    rolls->stopListening();
    
    if (enable_logging) {
	    delete rolls_logger;
	}

    // Wait 100 ms so the commands are actually completed
    usleep(100 * 1000);
    pthread_exit(NULL);
    return 0;
}

int main(int argc, char *argv[]) 
{
  // Variables that will store parsed values.
  const char* robot_file = "/dev/ttyUSB0";
  unsigned int robot_baudrate = 2000000;

  std::string log_dir = "../logs";
  std::string exp_name = "";
  bool no_timestamp = false;
  bool setup_needed = false;

  // register signal handler for program abort
  signal(SIGINT, aborter);

  // Robot file descriptor
  std::cout << "Connecting to robot over USB using default values for ports and baudrates" << robot_file << std::endl;

  int file_descriptor = getUSBFileDescriptor(robot_file, robot_baudrate);

  // Command line args
  po::options_description desc("Omnibot simulation.\n\nOptions");
  desc.add_options()
  ("help", "produce help message")
  ("logdir",
   po::value<std::string>(&log_dir)->default_value(log_dir),
   "directory to store logs")
  ("exp",
   po::value<std::string>(&exp_name)->default_value(exp_name),
   "name of experiment (used for logfiles)")
  ("no-timestamp",
   po::bool_switch(&no_timestamp),
   "disable automatic adding of timestamp to experiment name")
  ("setup", po::bool_switch(&setup_needed),
     "set up rolls architecture");


  // Parse input
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  // Help message
  //if (vm.count("help")) {std::cout << desc << "\n"; return 1;}

  // Only enable logging if a experiment name is given
  bool enable_logging = !exp_name.empty();

  // Prepare Log directory and log filenames
  if (log_dir.back() != '/') log_dir += '/';
  boost::filesystem::path dir(log_dir);
  if (!(boost::filesystem::exists(dir))) {
    std::cout << "Creating directory '" << log_dir << "'..." << std::endl;;
    if (boost::filesystem::create_directory(dir))
      std::cout << "Done." << std::endl;
  }
  if (!no_timestamp)
    exp_name += "_" + to_iso_string(pt::second_clock::local_time());
  std::string rolls_log = log_dir + exp_name + "_rolls.csv";


  return loopParallella(enable_logging,file_descriptor,setup_needed,rolls_log );

}

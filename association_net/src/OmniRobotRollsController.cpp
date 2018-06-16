#include <OmniRobotRollsController.h>
#include <OmniRobot.h>
#include <ROLLSDevice.h>
#include <boost/date_time.hpp>
#include <keyboard.h>
#include <RobotListener.h>
#include <exception>
#include <stdlib.h>

#define ARENA_SIZE 50




OmniRobotRollsController::OmniRobotRollsController( ROLLSDevice *rolls, OmniRobot *omnirobot)
                                                    : rolls(rolls),
                                                      omnirobot(omnirobot)
{
  std::cout<<"Creating neuron architecture\n";
  create_architecture();
}

void OmniRobotRollsController::create_architecture(){
  /* Define and connect the desired neuron groups:
  s1 and s1_2: Pattern one
  s2 and s2_2: Pattern two
  s3 and s3_2: Pattern three
  s4 and s4_2: Pattern four 
  
  Action groups:  drive_f, drive_right, drive_left, drive_b 
  
  */

  s1 = NeuronGroup(200,201); 
  s1_2 = NeuronGroup(216,217);
  s2 = NeuronGroup(202,203);
  s2_2 = NeuronGroup(218,219);
  s3 = NeuronGroup(204,205);
  s3_2 = NeuronGroup(220,221);
  s4 = NeuronGroup(206,209);
  s4_2 = NeuronGroup(222,223);
  
  drive_f = NeuronGroup(116,120);
  drive_right = NeuronGroup(121,125);
  drive_left = NeuronGroup(126,130);
  drive_b = NeuronGroup(135,140);
  
  stop_g = NeuronGroup(246,250);
  const_exc = NeuronGroup(251,253);
  col = NeuronGroup(245,255);
  inh_col = NeuronGroup(254,255);
	
  // connect sensory (pattern) groups to motor (action) groups via plastic synapses
  connectPlastic(s1, drive_right, true);
  connectPlastic(s2, drive_right, true);
  connectPlastic(s3, drive_right, true);
  connectPlastic(s4, drive_right, true);
  
  connectPlastic(s1, drive_left, true);
  connectPlastic(s2, drive_left, true);
  connectPlastic(s3, drive_left, true);
  connectPlastic(s4, drive_left, true);
  
  connectPlastic(s1, drive_f, true);
  connectPlastic(s2, drive_f, true);
  connectPlastic(s3, drive_f, true);
  connectPlastic(s4, drive_f, true);
  
  connectPlastic(s1, drive_b, true);
  connectPlastic(s2, drive_b, true);
  connectPlastic(s3, drive_b, true);
  connectPlastic(s4, drive_b, true);
  
  connectPlastic(s1_2, drive_right, true);
  connectPlastic(s2_2, drive_right, true);
  connectPlastic(s3_2, drive_right, true);
  connectPlastic(s4_2, drive_right, true);
  
  connectPlastic(s1_2, drive_left, true);
  connectPlastic(s2_2, drive_left, true);
  connectPlastic(s3_2, drive_left, true);
  connectPlastic(s4_2, drive_left, true);
  
  connectPlastic(s1_2, drive_f, true);
  connectPlastic(s2_2, drive_f, true);
  connectPlastic(s3_2, drive_f, true);
  connectPlastic(s4_2, drive_f, true);
  
  connectPlastic(s1_2, drive_b, true);
  connectPlastic(s2_2, drive_b, true);
  connectPlastic(s3_2, drive_b, true);
  connectPlastic(s4_2, drive_b, true);
  
  
  // Winner-take-all connectivity between action groups
  // connect these groups via inhibitory weight 4
  connectNonplastic(drive_right,drive_left,-4);
  connectNonplastic(drive_right,drive_b,-4);
  connectNonplastic(drive_right,drive_f,-4);
  
  connectNonplastic(drive_left,drive_right,-4);
  connectNonplastic(drive_left,drive_b,-3);
  connectNonplastic(drive_left,drive_f,-3);
  
  connectNonplastic(drive_b,drive_left,-4);
  connectNonplastic(drive_b,drive_right,-4);
  connectNonplastic(drive_b,drive_f,-4);
  
  connectNonplastic(drive_f,drive_right,-4);
  connectNonplastic(drive_f,drive_b,-4);
  connectNonplastic(drive_f,drive_left,-4);
  
  // const_exc is always active and excites stop_g which is 
  // active when there is no other driving signal
  // -> it makes sure the driving command is only send once
  connectNonplastic(const_exc,const_exc,4);
  connectNonplastic(const_exc,stop_g,4);
  
  connectNonplastic(drive_right,stop_g,-4);
  connectNonplastic(drive_left,stop_g,-4);
  connectNonplastic(drive_f,stop_g,-4);
  connectNonplastic(drive_b,stop_g,-4);

  
  
}

// Listener functions

void OmniRobotRollsController::receivedNewROLLSEvent(unsigned int neuron){
        // corresponds to drive forward neurons
       if ((neuron>=116) && (neuron<=120)&& (go==false))
       { go = true;
         omnirobot->drive(0,0,30,180); 
         }
         // corresponds to drive right neurons
     else if ((neuron>=121) && (neuron<=125)&& (go==false)) 
      { 
      go = true;
        omnirobot->drive(0,0,30,270); 
        }
        // corresponds to drive left neurons
     else if ((neuron>=126) && (neuron<=130)&& (go==false)) 
     { 
     go = true;
       omnirobot->drive(0,0,30,90); 
      }
      // corresponds to drive forward neurons
      else if ((neuron>=135) && (neuron<=140)&& (go==false)) 
      { go = true;
        omnirobot->drive(0,0,30,0); 
        }
     // set go to false so that if a new motor group is active,it will send a new drive command   
     else if ((neuron>=246) && (neuron<=250) && (go==true))  
     { go = false;
       }
    
}

void OmniRobotRollsController::receivedNewBumperEvent(unsigned int event_bumper){
    
  /* if one of the four specific bumper sensors is actived:
        log the bumper event and
        stimulate 3 of the virtual synapses of the corresponding drive population
        20 times to ensure the neuron crosses the membrane threshold 
    */
    
   if (event_bumper==6) {
    std::cout<<"Bumper event recognized\n"<<event_bumper<<std::endl;
    pt::time_duration duration = pt::microsec_clock::local_time() - startTime;
    double timestamp = duration.total_milliseconds();
    terminalLogFile.open(terminalLogName, std::ios_base::app);
    terminalLogFile
    << timestamp << "\t"
    << event_bumper << "\n";
    terminalLogFile.close();
    
    for (int i=0;i<20;i++){
        omnirobot->drive(0,0,3,0);
        rolls->stimulate(drive_f,3);
        rolls->stimulate(drive_f,1);
        rolls->stimulate(drive_f,2);
        }

  }
    if (event_bumper==8) {
    std::cout<<"Bumper event recognized\n"<<event_bumper<<std::endl;
    pt::time_duration duration = pt::microsec_clock::local_time() - startTime;
    double timestamp = duration.total_milliseconds();
    terminalLogFile.open(terminalLogName, std::ios_base::app);
    terminalLogFile
    << timestamp << "\t"
    << event_bumper << "\n";
    terminalLogFile.close();
    
    for (int i=0;i<20;i++){
        omnirobot->drive(0,0,3,180);
        rolls->stimulate(drive_right,3);
        rolls->stimulate(drive_right,1);
        rolls->stimulate(drive_right,2);
        }

  }
    if (event_bumper==48) {
    std::cout<<"Bumper event recognized\n"<<event_bumper<<std::endl;
    pt::time_duration duration = pt::microsec_clock::local_time() - startTime;
    double timestamp = duration.total_milliseconds();
    terminalLogFile.open(terminalLogName, std::ios_base::app);
    terminalLogFile
    << timestamp << "\t"
    << event_bumper << "\n";
    terminalLogFile.close();
 
    for (int i=0;i<20;i++){
        rolls->stimulate(drive_b,3);
        rolls->stimulate(drive_b,1);
        rolls->stimulate(drive_b,2);
        }

  }
  
    if (event_bumper==3) {
    std::cout<<"Bumper event recognized\n"<<event_bumper<<std::endl;
    pt::time_duration duration = pt::microsec_clock::local_time() - startTime;
    double timestamp = duration.total_milliseconds();
    terminalLogFile.open(terminalLogName, std::ios_base::app);
    terminalLogFile
    << timestamp << "\t"
    << event_bumper << "\n";
    terminalLogFile.close();
    
    for (int i=0;i<20;i++){
        rolls->stimulate(drive_left,3);
        rolls->stimulate(drive_left,1);
        rolls->stimulate(drive_left,2);
        }

  }

}

void OmniRobotRollsController::receivedNewOmniRobotEvent( double xpos,
                                                          double ypos )
{
  
}

void OmniRobotRollsController::receivedNewTerminalInputEvent(char read_char){

  // Reacts to register key strokes in active terminal
  
  // logs key events
  pt::time_duration duration = pt::microsec_clock::local_time() - startTime;
  double timestamp = duration.total_milliseconds();
  terminalLogFile.open(terminalLogName, std::ios_base::app);
  terminalLogFile
  << timestamp << "\t"
  << 1 << "\n";
  terminalLogFile.close();

  switch (read_char){
  
    // this was for validating the recognition network, 
    // making the robot move left and right in front of the pattern
  	case 'a':
  	for (int i=0;i<4;i++){
        omnirobot->drive(0, 0, 70, 270); //left 
        usleep(1000000);
        omnirobot->drive(0, 0, 70, 90); //right
        usleep(1000000);
        }
    	break;

    // simply let the robot move forward with speed 30cm/s
    case 'f':
    	std::cout<<"Drive.\n";
        omnirobot->drive(0,0,30,180);
    	break;
    // simply let the robot move backward
    case 'm':
    	std::cout<<"Drive.\n";
        omnirobot->drive(0,0,30,270);
    	break;
     // stop the robot
    case 's':
    	std::cout<<"Stop driving.\n";
		omnirobot->drive(0, 0, 0, 0);
    	break;
    // stimulate the const_exc at the start
    case 'i':
        go = false;
        for (int i=0;i<20;i++){
            rolls->stimulate(const_exc,1);
            rolls->stimulate(const_exc,2); 
            rolls->stimulate(const_exc,3); 
    	    }
        break;
    // quit the whole program
    case 'q':
    	std::cout<<"Terminate.\n";
    	omnirobot->emergencyStop();
    	omnirobot->stopListening(); // Then, stop all listening threads
   		rolls->stopListening();
   		usleep(100 * 1000);
		std::terminate();
    	break;
	}
}


void OmniRobotRollsController::receivedNewServoEvent(PushbotSignals &cmd){

}

void OmniRobotRollsController::start(){
 startTime = pt::microsec_clock::local_time(); 
}

void OmniRobotRollsController::stop(){
}

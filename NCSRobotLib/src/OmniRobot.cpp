#include <stdio.h>
#include <termios.h>
#include <string>
#include <cstring>
#include <cmath>
#include <chrono>
#include <time.h>
#include <iostream>
#include <stdlib.h>
#include <sstream> //includes osstringstream
#include <unistd.h> //provides the interface to the POSIX API
#include <fcntl.h>
#include <sys/time.h>
#include <errno.h>
#include <stdint.h>
#include <fstream>
#include <ctype.h>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

//#include <QUdpSocket> // Uncommented because Qt not yet installed on Raphaelas parallella
//#include <QDataStream>

//for debugging
#include <algorithm>
#include "OmniRobot.h"

// #define VEL_TO_SIGNAL   
#define GLOBAL_REP        true // for global  of path integration
#define PARALLELLA        false  // true if should run on parallella
#define LOG               true
#define WHEEL_RADIUS_CM   2.540 //2 inch wheel
#define MAX_ENCODER_VALUE 4096
#define PI_OVER_180       0.0174532925199433
#define SAMPLING_TIME     50000
#define ARENA_SIZE        50 // to normalize data for visualizer

// turn command echo on and off by toggling this variable
#define ECHO_ON
// turn readings from robot on and OFF
#define VERBOSE

/** Serial USB specific **/

bool OmniRobot::twitch(void)
{
  // Inital twitch (short rotation of all wheels) to check whether robot works
    std::cout << "Checking servos..." << std::endl;
    for(int i=0;i<3;i++){
        // make sure all servos are on and on full speed
        setServoTorque(i, 1, 1023);
    }
    // ping servos
    double driveSignals[3] = {200.0,200.0,200.0};
    setServoSpeeds(driveSignals);
    usleep(500000);
    driveSignals[0] = 0.0; driveSignals[1] = 0.0; driveSignals[2] = 0.0;
    setServoSpeeds(driveSignals);
    return true;
};

int OmniRobot::getbatteryVoltage()
{
    return batteryVoltage;
};

void OmniRobot::parseEncAndBumperEvents() {
  // Parses for encoder values (?PA\n) and bumper values (?Ib\n)
  // Saves parsed event with timestamps in private structs:
  // currentBumperVal, currentEncoderValues resp.

  // Number of samples to average for calculation of rotational speed
  const int SAMPLES = 3;

  Event event; // To store current event

  struct servoSignals currentServoCommand;
  int servoID;
  std::string servoVal;
  std::list<servoSignals> servoSignalBuffer;
  int buffersize;

  struct encoderStates encoders;
  struct encoderStates encoderDelta;
  struct encoderStates encoderSpeeds;
  std::string string;
  int index; // indicates end of current encoder value

  // Uncommented because Qt not yet installed on Raphaelas parallella
  //QUdpSocket socket;
  int pointnumber = 0;

  struct encoderStates avgDiff = {{0,0,0},0};
  struct encoderStates Diff = {{0,0,0},0,0};
  double correction_command;

  struct bumper currentBumperValue;
  std::string bumper_string;

  event_mutex.lock();
  event = m_event_list.front(); // Read in first entry
  m_event_list.pop_front();     // Pop first list element
  event_mutex.unlock();

  // Parse Encoder Signals (starts with ?PA\n-S16 )
  if (event.m_data[0] == '?' &&
      event.m_data[1] == 'P' &&
      event.m_data[2] == 'A' &&
      event.m_data[4] == '-' &&
      event.m_data[5] == 'S' &&
      event.m_data[6] == '1' &&
      event.m_data[7] == '6'){
    encoders.timestamp = event.timestamp;
    for(int i=9; i<13; i++) {
      if(event.m_data[i]==' ') {
        index = i-1;
        break;
      }
      else {
        string += event.m_data[i];
        index = i;
      }
    }
    // Check for invalid (non-int) encoder value
    // Due to robot returning "battery voltage too low" or similar
    if (isNumber(string)) {
      encoders.values[0] = stoi(string);
    }
    else {
      std::cout << "Non-int encoder 0 value" << string << "." << std::endl;
      encoders.values[0] = encoderBuffer.back().values[0];
    }
    string="";
    index +=2;

    for(int i=index; i<index+4; i++) {
      if(event.m_data[i]==' ') {
        index = i-1;
        break;
      }
      else {
        string += event.m_data[i];
        index = i;
      }
    }
    if (isNumber(string)) {
      encoders.values[1] = stoi(string);
    }
    else {
      std::cout << "Non-int encoder 1 value" << string << "." << std::endl;
      encoders.values[1] = encoderBuffer.back().values[1];
    }
    string="";
    index += 2;

    for(int i=index; i<index+4; i++) {
      if(event.m_data[i]=='\n')
        break;
      else {
        string += event.m_data[i];
      }
    }

    if (isNumber(string)) {
      encoders.values[2] = stoi(string);
    }
    else {
      std::cout << "Non-int encoder 2 value" << string << "." << std::endl;
      encoders.values[2] = encoderBuffer.back().values[2];
    }
    string="";
    currentEncoderState = encoders;
  } // Parsing for encoder values is finished here --------------------

  // Parsing bumper values (starts with "?Ib")
  else if (event.m_data[0] == '?' &&
           event.m_data[1] == 'I' &&
           event.m_data[2] == 'b') {
    bumper_string += event.m_data[9];
    bumper_string += +event.m_data[10];

    // Check if bumper_string is a number
    // If not store back last recorded value
    if ( isNumber(bumper_string) ) {
      currentBumperValue.value = stoi(bumper_string);
    }
    else {
      std::cout << "Non-int bumper value\n";
      currentBumperValue.value = currentBumperVal.value;
    }
    currentBumperValue.d_timestamp = event.timestamp;
    notifyListeners(currentBumperValue.value);
    currentBumperVal.value=currentBumperValue.value;
    bumper_string = "";
  } // End Parsing bumper values
}

void OmniRobot::computeEncoderDeltas() {
  // Calculates encoder speeds by forming delta with previous encoder value
  // stored in encoderBuffer.
  // Then it computes the difference of each encoder speed value of the current event to
  // the current drive command (in °/s) and stores them back in vector
  // Lastly, implements a PI controller which sends a correction command
  struct encoderStates encoderDelta;
  struct encoderStates encoderSpeeds;
  struct encoderStates encoders = currentEncoderState;
  const int SAMPLES = 3; //number of samples over for integration
  struct encoderStates Diff;
  struct encoderStates avgDiff;
  int correction_command;

  if (encoderBuffer.size() == 0) {
    encoderDelta = {{0,0,0},0,0}; // First one needs to be zero
    encoderSpeeds = {{0,0,0},0,0}; // First one needs to be zero
  }
  else {
    for (int i=0;i<3;i++) {
      // Calculate time difference
      // Note: Here, timestamp denotes the timedelta between two events
      // Delta t in us
      encoderDelta.timestamp = encoders.timestamp;
      encoderDelta.delta_t = encoders.timestamp -
                             encoderBuffer.back().timestamp;
      // Handle errors coming from the communication with the robot
      if ( encoderDelta.delta_t < 0.95 * SAMPLING_TIME ) {
        encoderDelta.delta_t = 0; // So it will be disregarded later
      }

      // Faulty encoder value
      if (encoders.values[i] == -128) {
        // Assume unchanged delta and speed if faulty encoder value
        // But also store last encoder value for this event
        encoders.values[i]=encoderBuffer.back().values[i] +
                          (encoderDeltaBuffer.back().values[i] *
                           MAX_ENCODER_VALUE/360);
        if ( encoders.values[i]<MAX_ENCODER_VALUE) {
          encoders.values[i]+=MAX_ENCODER_VALUE;
        }
        else if ( encoders.values[i]>=MAX_ENCODER_VALUE) {
          encoders.values[i]-=MAX_ENCODER_VALUE;
        }

        encoderSpeeds.values[i] = encoderSpeedsBuffer.back().values[i];
        encoderDelta.values[i]=encoderDeltaBuffer.back().values[i];
      }
      else {
        // Wheel encoder delta in °
        encoderDelta.values[i]=( encoders.values[i] -
                                 encoderBuffer.back().values[i]) /
                                 MAX_ENCODER_VALUE*360;

        // Handle transition over MAX_ENCODER_VALUE
        // which are essentially too high negative deltas
        // Assumption: No wheel will have more than 180° revolution
        //             in one sampling period (in any direction)
        //             Otherwise increase sampling rate
        if ( encoderDelta.values[i]<=-180) {
          encoderDelta.values[i]+=360;
        }
        else if ( encoderDelta.values[i]>=180) {
          encoderDelta.values[i]-=360;
        }

        // Computed speed °/s for single event
        if ( encoderDelta.delta_t > 0.95*SAMPLING_TIME) {
          encoderSpeeds.values[i]=encoderDelta.values[i] /
                                  encoderDelta.delta_t;
          encoderSpeeds.values[i]*=1000000;
          encoderSpeeds.delta_t=encoderDelta.delta_t;
        }
        else { // set to set value so no error results
          encoderSpeeds.values[i] = 0.666*currentDriveCommands.values[i];
          encoderSpeeds.delta_t=encoderDelta.delta_t;
        }
      }
    }
  }

  // Store values
  encoderDeltaBuffer.push_back(encoderDelta);
  encoderSpeedsBuffer.push_back(encoderSpeeds);
  encoderBuffer.push_back(encoders);

  // Compute average speeds (average over last SAMPLES samples)
  // Iterate over servos and number of samples used to average speed
  double speeds[3] = {0,0,0};
  int buffersize = encoderSpeedsBuffer.size();
  if (buffersize >= SAMPLES) {
    for (int j=0;j<3;j++) {
      for (int i=buffersize-1;i>=(buffersize-SAMPLES);i--) {
        speeds[j] += encoderSpeedsBuffer[i].values[j];
      }
      speeds[j]=speeds[j]/SAMPLES;
    }
  }

  // Compute differences
  if (encoderSpeedsBuffer.empty() == false) {
    for (int i=0;i<3;i++) {

      // Compute differences of measured value to the one set
      // via setServoCommands (in °/s)
      Diff.values[i] = encoderSpeedsBuffer.back().values[i] -
          currentDriveCommands.values[i] *
          0.111*360/60;

      // Assume 0 error when timestamp is below a certain threshold
      // Which is the case for lagging robot data
      if ( encoderSpeedsBuffer.back().delta_t < 10000 ) {
        Diff.values[i]=0;
      }
      // Assume 0 error on too high (faulty) absolute Diff value
      if (Diff.values[i] > (currentDriveCommands.values[i]
                            *0.111*360/60*.3)) {
        Diff.values[i]=0;
      }
      if (Diff.values[i] < -(currentDriveCommands.values[i]
                           *0.111*360/60*.3)) {
        Diff.values[i]=0;
      }

      avgDiff.values[i] +=
          Diff.values[i]*encoderSpeedsBuffer.back().timestamp/1000000;

      // Controller setup
      double p_error_corr = 0.5; // PID parameters
      double i_error_corr = 0.1;

      correction_command = currentDriveCommands.values[i]
                        -( p_error_corr*Diff.values[i]/0.666
                        +  i_error_corr*avgDiff.values[i]/0.666 );
      // Send control command
      setServoSpeed(i, (int)correction_command);
    }
  }

  // Crop data structures if exceeding certain size
  if (encoderBuffer.size()>10) {
    encoderBuffer.pop_front();
  }

  for (int i=0;i<3;i++) {
    Diff.values[i] = 0;
  }
  if (encoderSpeedsBuffer.size() > SAMPLES) {
    encoderSpeedsBuffer.erase(encoderSpeedsBuffer.begin());
  }
  if (encoderDeltaBuffer.size() > SAMPLES) {
    encoderDeltaBuffer.erase(encoderDeltaBuffer.begin());
  }// End Compute differences
}

// drive forward
// angular_vel [deg/s]
// linear_vel [cm/s]
// direction [deg]
// delta_t [s]
void OmniRobot::drive(double angular_vel,
                      bool CCW,
                      double linear_vel,
                      double direction) {
  #define MAX_SERVO_SIGNAL 800 // do not set above 1000 !!!
  #define SIGNAL_TO_RPM   0.111
  #define WHEEL_RADIUS_CM 2.540 //2 inch wheel

    // convert the input velocities in signals
    // check for signal boundaries 
    // Idea: saturation is avoided by limiting the
    // maximum sum between linear and angular velocity, in which
    // case, angular velocity is given priority over linear velocity.
    // we have to map the driving signal to the allowed range
    // If a value is in the range of 0~1023, it means that the motor rotates to the CCW direction.
    // If a value is in the range of 1024~2047, it means that the motor rotates to the CW direction.
    // the signal conversion is 0.111 rpm/unit signal
    // linear and angular velocity have to range from 0 ... 100
    
  if( linear_vel > 100 )
      linear_vel = 100;
  else if( linear_vel > 100 )
      linear_vel = 100;

  if( angular_vel > 100 )
      angular_vel = 100;
  else if( angular_vel > 100 )
      angular_vel = 100;
  // their sum has also to be < 100
  if( linear_vel + angular_vel > 100 ){
    std::cout<<"Maximum servo speed exceeded. Limiting angular velocity\n";
    angular_vel = 100 - linear_vel;
    }
    // now that all ranges are checked we can proceed to calculate directions

    // linear movement
    double linear_components[3];
    
    linear_components[0] = MAX_SERVO_SIGNAL*linear_vel/100 *
                           sin( (direction - 240)*PI_OVER_180 );
    linear_components[1] = MAX_SERVO_SIGNAL*linear_vel/100 *
                           sin( (direction)*PI_OVER_180 );
    linear_components[2] = MAX_SERVO_SIGNAL*linear_vel/100 *
                           sin( (direction - 120)*PI_OVER_180 );

    // angular movement
    double angular_components[3];
    
    for(int i=0; i<3; i++) {   
        if(CCW){
            angular_components[i] = - MAX_SERVO_SIGNAL*angular_vel/100;
        } else {
            angular_components[i] = MAX_SERVO_SIGNAL*angular_vel/100;
        };
    };
        
    // superposition of angular the braces and linear movement
    double drive_components[3];
    for(int i=0; i<3; i++){
        drive_components[i] = linear_components[i] + angular_components[i];
    };
    robotDriveState drivestate = robotDriveState(angular_vel,
                                                 CCW,
                                                 linear_vel,
                                                 direction);
    this->setRobotDriveState(drivestate);
    setServoSpeeds(drive_components);
};

// delta_t time in s
void OmniRobot::drive(double angular_vel,
                      bool CCW,
                      double linear_vel,
                      double direction,
                      double delta_t)
{
    drive(angular_vel, CCW, linear_vel, direction);
    usleep(delta_t*1000000);
    double stop_components[3] = {0,0,0};
    robotDriveState drivestate = robotDriveState(angular_vel,
                                                 CCW,
                                                 linear_vel,
                                                 direction);
    setServoSpeeds(stop_components);
};

void OmniRobot::drive()
{
    drive( state.angular_vel, state.CCW, state.linear_vel, state.direction);
}

//move into random direction with same speeds
void OmniRobot::drive_random()
{
  int rand_orient = rand() % 360 + 1;
  drive(state.angular_vel, state.CCW, state.linear_vel, rand_orient);
  srand (time(NULL));
};

void OmniRobot::drive_random(int bumper_signal){
  // Compute where wall is and set range of direction accordingly
  // (+- 70° in opposite 180° of the normal)
  srand (time(NULL));
  int rand_int = rand() % 180 + 1;
  rand_int-=70;
  int rand_orient=rand() % 360 + 1;

  switch(bumper_signal) {
    case 16: rand_orient=330+rand_int; break;//bumper 1 (at 150°)
    case 32: rand_orient=270+rand_int; break;
    case 01: rand_orient=210+rand_int; break;
    case 02: rand_orient=150+rand_int; break;
    //case xy: rand_orient=90+rand_int; break; to be defined after fixing of bumper 5
    case 8: rand_orient=30+rand_int; break; //bumper 6

    case 48: rand_orient=330+rand_int; break;  //bumpers 1&2
    case 33: rand_orient=240+rand_int; break; //bumpers 2&3
    case 03: rand_orient=180+rand_int; break;
    //case xyz: rand_orient=120+rand_int; break; to be defined after fixing of bumper 5
    //case zy: rand_orient=60+rand_int; break; to be defined after fixing of bumper 5
    case 24: rand_orient=rand_int; break; //bumpers 6&1

    case 49: rand_orient=270+rand_int; break;  //bumpers 1&2&3
    case 35: rand_orient=210+rand_int; break; //bumpers 2&3&4
    //case 03: rand_orient=150+rand_int; break; to be defined after fixing of bumper 5
    //case a: rand_orient=90+rand_int; break; to be defined after fixing of bumper 5
    //case b: rand_orient=30+rand_int; break; to be defined after fixing of bumper 5
    case 56: rand_orient=330+rand_int; break; //bumpers 6&1&2
    }

    if (rand_orient>360)
      rand_orient-=360;

  drive(state.angular_vel, state.CCW, state.linear_vel, rand_orient);
}

//distance [cm]
void OmniRobot::drive_dist(double angular_vel,
                           bool CCW,
                           double linear_vel,
                           double direction,
                           double distance) {
  double delta_t=distance/linear_vel;
  drive(angular_vel, CCW, linear_vel, direction, delta_t);
}

// heading [deg] --> angle referring to global frame
// (y-axis marked with double arrow), range 0 ... 360
void OmniRobot::drive_global(double angular_vel,
                             bool CCW,
                             double linear_vel,
                             double direction,
                             double heading) {
  direction -= heading;
  if (direction > 360) {
    direction -= 360;
  }
  drive(angular_vel, CCW, linear_vel, direction);
};

void OmniRobot::drive_global_rot(double angular_vel,
                                 bool CCW,
                                 double linear_vel,
                                 double direction,
                                 double heading_start) {
  // TODO to be implemented
};

void OmniRobot::emergencyStop()
{   
    state.linear_vel  = 0;
    state.direction   = 0;
    state.angular_vel = 0;
    state.CCW         = true;
    drive();
};

void OmniRobot::auxPathIntegration(){
  // Hack: Used to generate data in the absence of regular robot data
  // Might not be needed anymore in the future
  float dx = PI_OVER_180*WHEEL_RADIUS_CM
           *(cos(30*PI_OVER_180)*encoderDeltaBuffer.back().values[0]
           - cos(30*PI_OVER_180)*encoderDeltaBuffer.back().values[2]);
  float dy = PI_OVER_180*WHEEL_RADIUS_CM
          *(-cos(60*PI_OVER_180)*encoderDeltaBuffer.back().values[0]
          +  encoderDeltaBuffer.back().values[1]
          -  cos(60*PI_OVER_180)*encoderDeltaBuffer.back().values[2]);

  auxCurrentPosition.posX += dx;
  auxCurrentPosition.posY += dy;

  float dx_global = dx*cos(auxCurrentGlobalPosition.heading*PI_OVER_180)
                   -dy*sin(auxCurrentGlobalPosition.heading*PI_OVER_180);
  float dy_global = dx*sin(auxCurrentGlobalPosition.heading*PI_OVER_180)
                   +dy*cos(auxCurrentGlobalPosition.heading*PI_OVER_180);

  auxCurrentGlobalPosition.posX += dx_global;
  auxCurrentGlobalPosition.posY += dy_global;

  float head = encoderDeltaBuffer.back().values[0]*1.
             + encoderDeltaBuffer.back().values[1]*1.
             + encoderDeltaBuffer.back().values[2]*1.;
  auxCurrentGlobalPosition.heading += head;

  if( GLOBAL_REP ){
    warnEventOmniRobotPos(auxCurrentGlobalPosition.posX,
                          auxCurrentGlobalPosition.posY);
  }
  else {
    warnEventOmniRobotPos(auxCurrentPosition.posX,
                          auxCurrentPosition.posY);
  }
}

void OmniRobot::pathIntegration(){
  // Do path integration in order to keep track of robot's current position
  // Uses parsed and computed encoder deltas
  struct encoderStates deltaEnc = encoderDeltaBuffer.back();

  // Calculates x and y deltas within robot's reference frame
  // See marks on Omnibot
  float dx = PI_OVER_180*WHEEL_RADIUS_CM
           *(cos(30*PI_OVER_180)*deltaEnc.values[0]
           - cos(30*PI_OVER_180)*deltaEnc.values[2]);
  float dy = PI_OVER_180*WHEEL_RADIUS_CM
          *(-cos(60*PI_OVER_180)*deltaEnc.values[0]
          +  deltaEnc.values[1]
          -  cos(60*PI_OVER_180)*deltaEnc.values[2]);

  if (dx > 2) {dx = 0;} // Ignore initial value which is an offset
  if (dy > 2) {dy = 0;} // Ignore initial value which is an offset

  currentPosition.posX += dx;
  currentPosition.posY += dy;

  // For "global" representation
  // Takes into account changing heading
  // Need to set some sort of starting heading which defines global frame to
  // Robot's frame in the beginning
  float dx_global = dx*cos(this->currentGlobalPosition.heading*PI_OVER_180)
                   -dy*sin(this->currentGlobalPosition.heading*PI_OVER_180);
  float dy_global = dx*sin(this->currentGlobalPosition.heading*PI_OVER_180)
                   +dy*cos(this->currentGlobalPosition.heading*PI_OVER_180);

  currentGlobalPosition.posX += dx_global;
  currentGlobalPosition.posY += dy_global;

  float head = deltaEnc.values[0]*1.
             + deltaEnc.values[1]*1.
             + deltaEnc.values[2]*1.;
  currentGlobalPosition.heading += head;

  auxCurrentPosition = currentPosition;
  auxCurrentGlobalPosition = currentGlobalPosition;

  if( GLOBAL_REP ){
    warnEventOmniRobotPos(currentGlobalPosition.posX,
                          currentGlobalPosition.posY);
  }
  else {
    warnEventOmniRobotPos(currentPosition.posX,
                          currentPosition.posY);
  }
  }

// Uncommented because Qt not yet installed on Raphaelas parallella

//void OmniRobot::visualize(QUdpSocket* socket, int i){
//  float tx, ty, head, b;
//
//  if (currentBumperVal.value != 0)
//    b = 1.;
//  else
//    b = 0.;
//
//  if (!lagFlag) { // Hack due to lagging robot values
//    if (GLOBAL_REP) {
//      head = currentGlobalPosition.heading;
//      tx = - currentGlobalPosition.posX/50; // needs -, not sure why
//      ty = - currentGlobalPosition.posY/50; // needs -, not sure why
//     }
//    else {
//      head = currentPosition.heading;
//      tx = - currentPosition.posX/50; // needs -, not sure why
//      ty = - currentPosition.posY/50; // needs -, not sure why
//    }
//  }
//  else {
//    if (GLOBAL_REP) {
//      head = auxCurrentGlobalPosition.heading;
//      tx = - auxCurrentGlobalPosition.posX/50; // needs -, not sure why
//      ty = - auxCurrentGlobalPosition.posY/50; // needs -, not sure why
//     }
//    else {
//      head = auxCurrentPosition.heading;
//      tx = - auxCurrentPosition.posX/50; // needs -, not sure why
//      ty = - auxCurrentPosition.posY/50; // needs -, not sure why
//    }
//  }
//
//  // Send data
//  QByteArray datagram;
//  QDataStream out(&datagram, QIODevice::WriteOnly);
//  //out.setVersion(QDataStream::Qt_5_8); // Different version on parallella
//  out << i << tx << ty << head << b;
//  if (PARALLELLA){
//    socket->writeDatagram(datagram,QHostAddress("172.19.9.206"),12345);
//  }
//  else {
//    socket->writeDatagram(datagram,QHostAddress::LocalHost,12345);
//  }
//}
//

void OmniRobot::logInit(){
  // Sets up logfiles with headers
  deltaLogFile.open("deltaLog.csv", std::ios_base::app);
  deltaLogFile
    << "Timestamp" << "\t"
    << "Encoder Val 0" << "\t"
    << "Encoder Val 1" << "\t"
    << "Encoder Val 2" << "\t"
    << "EncoderDelta dt" << "\t"
    << "EncoderDelta Val 0" << "\t"
    << "EncoderDelta Val 1" << "\t"
    << "EncoderDelta Val 2" << "\t"
    << "Encoder Speed 0" << "\t"
    << "Encoder Speed 1" << "\t"
    << "Encoder Speed 2" << "\t"
    << "Drivecommand Servo 0" << "\t"
    << "Drivecommand Servo 1" << "\t"
    << "Drivecommand Servo 2" << "\n";
  deltaLogFile.close();

  if (LOG) {
    visLogFile.open("visualizerLog.csv");
    logNumber = 1;
    visLogFile.close();
  }
}

void OmniRobot::log(){
  // Used to log data
  deltaLogFile.open("deltaLog.csv", std::ios_base::app);
  deltaLogFile
    << currentEncoderState.timestamp << "\t"
    << currentEncoderState.values[0] << "\t"
    << currentEncoderState.values[1] << "\t"
    << currentEncoderState.values[2] << "\t"
    << encoderDeltaBuffer.back().delta_t << "\t"
    << encoderDeltaBuffer.back().values[0] << "\t"
    << encoderDeltaBuffer.back().values[1] << "\t"
    << encoderDeltaBuffer.back().values[2] << "\t"
    << encoderSpeedsBuffer.back().values[0] << "\t"
    << encoderSpeedsBuffer.back().values[1] << "\t"
    << encoderSpeedsBuffer.back().values[2] << "\t"
    << currentDriveCommands.values[0]*0.111*360/60 << "\t"
    << currentDriveCommands.values[1]*0.111*360/60 << "\t"
    << currentDriveCommands.values[2]*0.111*360/60 << "\t";
  deltaLogFile.close();

  visLogFile.open("visualizerLog.csv", std::ios::app);
    visLogFile << logNumber << " " << currentPosition.posX << " " << currentPosition.posY << " "
    << currentEncoderState.timestamp << " " << currentBumperVal.value << "\n";
    visLogFile.close();
    logNumber++;
}

/* setter functions */

void OmniRobot::setRobotDriveState( const robotDriveState& inState ) 
{
    state.linear_vel  = inState.linear_vel;
    state.direction   = inState.direction;
    state.angular_vel = inState.angular_vel;
    state.CCW         = inState.CCW;
};

void OmniRobot::setServoSpeed(int servoID, int signal) {
  if(servoID>=0 && servoID<3){
    std::ostringstream convert;
    convert << servoID;
    convert << signal;

    m_status = sendCommand("!P"+convert.str()+"\n",true,&usb_mutex);
    //m_status = sendCommand("!P"+convert.str()+"\n");
  }
}

void OmniRobot::setServoSpeeds(double * servosignals) {
  std::ostringstream convert;
  for(int servoID = 0; servoID < 3; servoID++){
    convert << "!P";
    convert << servoID;
    convert << (int)floor(servosignals[servoID]);
    convert << "\n";
    // Globally store current servo commands
    currentDriveCommands.values[servoID]=(int)floor(servosignals[servoID]);
  }
  m_status = sendCommand(convert.str(),true,&usb_mutex);
  //m_status = sendCommand(convert.str());
}

void OmniRobot::setServoTorque(int servoID, bool enable, int signal) {
  if(servoID>=0 && servoID<3) {
    std::ostringstream convert;
    convert << enable;
    convert << servoID;
    convert << signal;

    m_status = sendCommand("!T"+convert.str()+"\n",false,&usb_mutex);
  }
  else {
    std::cout << "ServoID not in allowed range." << std::endl;
  }
}

void OmniRobot::getEncoderSignals(){
      sendCommand("?PA\n",false,&usb_mutex);
}

void OmniRobot::getBumperSignal(){
    sendCommand("?Ib\n",false,&usb_mutex);
}

void OmniRobot::processKeys(int keysPressed) {
    robotDriveState driveState;

    switch( keysPressed )
    {
        case STOP :
            std::cout << "Stop" << std::endl;
            driveState = robotDriveState(0, true, 0, 0);
            break;
        case FORWARD :
            std::cout << "Forward" << std::endl;
            driveState = robotDriveState(0, true, 50, 0);
            break;
        case BACKWARD :
            std::cout << "Backward" << std::endl;
            driveState = robotDriveState(0, true, 50, 180);
            break;
        case LEFT : 
            std::cout << "Left" << std::endl;
            driveState = robotDriveState(0, true, 50, 270);
            break;
        case RIGHT : 
            std::cout << "Right" << std::endl;
            driveState = robotDriveState(0, true, 50, 90);
            break;
        case TURN_CCW : 
            std::cout << "Turn CCW" << std::endl;
            driveState = robotDriveState(50, true, 0, 0);
            break;
        case TURN_CW : 
            std::cout << "Turn CW" << std::endl;
            driveState = robotDriveState(50, false, 0, 0);
            break;
        default :
            std::cout << "ERROR: Unknown Command" << std::endl;
    }

    this->setRobotDriveState(driveState);
    this->drive();
}

// To overcome invalid argument error when using stoi on non-int value
// Mainly occurring when robot is sending "battery voltage too low" or similar
// Allow - for -128 (faulty value)
bool OmniRobot::isNumber(std::string& str) {
  if (str=="-128") return true;
  if (str.length()==0) return false;
  for (int i=0;i<str.length();i++) {
    if( !((str[i] >= '0' && str[i] <= '9')) ) { //|| str[i]=='-'
      return false;
    }
  }
  return true;
}

void* queryServoThread(void* arg)
{
  // Handles the querying of encoder and bumper state
  struct OmnibotArgs* args = (struct OmnibotArgs*)(arg);

  while(true) {
    if(*(args->thread_status)==-1) {
      break;
    }
    args->omnirobot->getEncoderSignals();
    usleep(SAMPLING_TIME/2);

    args->omnirobot->getBumperSignal();
    usleep(SAMPLING_TIME/2);
  }

  std::cout<<"> Exiting queryThread"<<std::endl;
  pthread_exit(NULL);
}

void* readThreadOmnibot(void* arg)
{
  // Cotinuously reads data from serial port if available
    struct OmnibotArgs* args = (struct OmnibotArgs*)(arg);

    // Not really important in fact
    #define TO_READ    7200   // bytes

    // To receive the data
    int               file_descriptor = *(args->file_descriptor); // The stream
    unsigned char     data[TO_READ];            // Raw stream of events
    // # Bytes read in the raw stream (not really needed here)
    unsigned int      bytesRead;
    // How many bytes have been processed for the current event
    int               eventProcessingIndex = 0;
    Event             current_event;            // Buffer event
    std::list<Event>  event_list;
    timeval           tv;
    timeval start,middle,end;

    // Data structures for the select
    fd_set rFdSet;
    FD_ZERO(&rFdSet);
    FD_SET(file_descriptor, &rFdSet);
    int r;

    while(true) {
      usleep(1000);
      // Check if the thread is still alive
      if(*(args->thread_status)==-1) {
        break;
      }
      //r=select(file_descriptor + 1, &rFdSet, NULL, NULL, NULL);
      r=1;
      if(r) {
        gettimeofday(&start, 0);
        bytesRead = read(file_descriptor, data, TO_READ);
        gettimeofday(&middle, 0);
//        std::cout<<"data: "<<data<<std::endl;

        for( int n = 0; n < (int)bytesRead; n++ ) {
          if (data[n]=='!' || data[n]=='?') {    // Sequencing //|| data[n]=='-'
          //if (data[n]=='-') {    // Start reading on -
            gettimeofday(&tv, 0);
            current_event.timestamp = tv.tv_sec*1000000+tv.tv_usec;
            args->omnirobot->event_mutex.lock();
            // Push current event to back of list
            args->omnirobot->m_event_list.push_back(current_event);
            args->omnirobot->event_mutex.unlock();
            eventProcessingIndex = 0;
            // Empty buffer
            std::fill(current_event.m_data,
                      current_event.m_data + sizeof(current_event.m_data),
                      0);
          }
//          std::cout<<"event: "<<current_event.m_data<<std::endl;
          current_event.m_data[eventProcessingIndex] = data[n];
          eventProcessingIndex++;
        }
      }
      else if (r == -1) {
        std::cout << "> Error in the listening thread" << std::endl;
      }
      else {
        std::cout << "> No data within 5ms" << std::endl;
      }
    }
    std::cout<<"> Exiting readThread"<<std::endl;
    pthread_exit(NULL);
}


//path integration and encoder deltas switched off for the moment (too much computation, events are skipped)
void* controlThread(void* arg)
{
  // Upon arriving events processes data and starts controlling --
  // I.e. parses incoming events, computes error of encoder
  // values to current set command
  // Also calls path integration and visualizes data with ebv-visualizer
  struct OmnibotArgs* args = (struct OmnibotArgs*)(arg);
//  QUdpSocket socket; // Uncommented because Qt not yet installed on Raphaelas parallella
  int pointnumber = 0;
  args->omnirobot->currentPosition = {0,0,0};
  args->omnirobot->currentGlobalPosition = {0,0,0};

  if (LOG) {
    args->omnirobot->logInit();
  }

  // Wait for until very first events have arrived
  while(args->omnirobot->m_event_list.size() < 2) {
    usleep(1);
    if(*(args->thread_status)==-1) {
      break;
    }
  }
  args->omnirobot->event_mutex.lock();
  args->omnirobot->m_event_list.pop_front();  // Pop non valid event
  args->omnirobot->event_mutex.unlock();

  timeval start, current;
  gettimeofday(&start, 0);
  double timer = 0;

  std::cout<<"before while true\n";
  while(true){
    if(*(args->thread_status)==-1) {
      break;
    }
    if(!(args->omnirobot->m_event_list.empty())){
      gettimeofday(&start,0);
      args->omnirobot->lagFlag=false;
      args->omnirobot->parseEncAndBumperEvents();
      if (timer >= 5000) {
      //std::cout<<"Timer\n"<< timer << std::endl;
        //args->omnirobot->computeEncoderDeltas(); //
        //args->omnirobot->pathIntegration();  //comment in again
      }
      if (LOG) {
        //args->omnirobot->log(); important, comment back in
      }
      // Uncommented because Qt not yet installed on Raphaelas parallella
//      args->omnirobot->visualize(&socket, pointnumber);
      pointnumber++;
    }
    else { // generate data with last deltas
      gettimeofday(&current, 0);
      timer = current.tv_usec-start.tv_usec;
      // Hack that "generates" data points when lack of data
//      if (timer > 2*SAMPLING_TIME){
//        args->omnirobot->lagFlag=true;
//        gettimeofday(&start, 0);
//        args->omnirobot->auxPathIntegration();
////        args->omnirobot->visualize(&socket, pointnumber);
//        pointnumber++;
//      }
      usleep(500); //was 500
      continue;
    }
  }
  std::cout<<"> Exiting controlThread"<<std::endl;
  pthread_exit(NULL);
}

int OmniRobot::listen(void)
{
  // Starts the three threads for data acquisition and processing
    if(m_status)
    {
        if(!m_toggleListening)
        {
            m_readthread_args.omnirobot = this;
            m_readthread_args.file_descriptor=&this->fileDescriptor;
            m_readthread_args.thread_status = &m_readThreadStatus;

            std::cout<<"OmniRobot: Activate listening"<<std::endl;
            m_readThreadStatus = OmniRobot::RUN_THREAD;
            m_toggleListening = true;
            pthread_create(&m_readthread[0],
                           NULL,
                           readThreadOmnibot,
                           (void*)&m_readthread_args);
            pthread_create(&m_readthread[1],
                           NULL,
                           queryServoThread,
                           (void*)&m_readthread_args);
            pthread_create(&m_readthread[2],
                           NULL,
                           controlThread,
                           (void*)&m_readthread_args);
            std::cout<<"Done initialising robot control threads\n";
            return 1;
        }
        else
            return 1;
    }
    else {
      std::cout<<"\t Cannot activate listening because "
                 "Omnibot has not been started ...\n"<<std::endl;
      return -1;
    }
}

int OmniRobot::stopListening(void)
{
    if(!m_toggleListening)
    {
        return true;
    }
    else
    {
        m_readThreadStatus = Robot::TERMINATE_THREAD;
        m_toggleListening = false;
        pthread_join(m_readthread[1], NULL);
        pthread_join(m_readthread[0], NULL);
        pthread_join(m_readthread[2], NULL);
    }
    return 0;
}

void OmniRobot::notifyListeners(unsigned int bumper){
  warnEventBumper(bumper);
}

void OmniRobot::warnPosTest( double xpos, double ypos){
  warnEventOmniRobotPos(xpos,ypos);
}

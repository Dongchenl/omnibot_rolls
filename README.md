# omnibot_rolls
Code to interface the omni-directional robot and the ROLLS neuromorphic chip. 

# Code that defines the association network 
The NCSRobotLib includes files that are needed to configure the connectivity on the ROLLS neuromorphic chip, send commands to the Omnibot over serial connection, and listen to the keyboard to execute commands. 

The association_net includes the controller: The network connectivity is specified and it is declared which neural activity results in which motor commands of the robot. <br />

# Usage

## Hardware dependency
To execute the program, the ROLLS neuromorphic chip and the Omnibot are required. <br />

To build the program, the NCSRobotLib needs to be build first:

Type the following in the terminal <br />
$cd NCSRobotLib
$mkdir build
$cd build
$cmake ../
$make

 <br />
After building the library compile the controller in the association_net:
$cd ../../associaion_net
$mkdir build
$cd build
$cmake ../
$make










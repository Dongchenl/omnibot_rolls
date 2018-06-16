# omnibot_rolls
Code to interface the omni-directional robot and the ROLLS neuromorphic chip. 

# Code that defines the association network 
The NCSRobotLib includes files that are needed to configure the connectivity on the ROLLS neuromorphic chip, send commands to the Omnibot over serial connection, and listen to the keyboard to execute commands. 

The association_net includes the controller: The network connectivity is specified and it is declared which neural activity results in which motor commands of the robot. <br />

# Usage

## Hardware dependency
To execute the program, the ROLLS neuromorphic chip and the Omnibot are required. <br />

To build the program simply run the compile.sh script.
<br />
An alternative is to run the following commands:<br />
The NCSRobotLib needs to be build first:

Type the following in the terminal <br />
$cd NCSRobotLib <br />
$mkdir build <br />
$cd build <br />
$cmake ../ <br />
$make <br />

 <br />
After building the library compile the controller in the association_net: <br />
$cd ../../associaion_net <br />
$mkdir build <br />
$cd build <br />
$cmake ../ <br />
$make <br />










# omnibot_rolls
Code to interface the omni-directional robot and the ROLLS neuromorphic chip. 

The NCSRobotLib includes files that are needed to configure the connectivity on the ROLLS neuromorphic chip, send commands to the Omnibot over serial connection, and listen to the keyboard to execute commands. 

The association_net includes the controller: The network connectivity is specified and it is declared which neural activity results in which motor commands of the robot.

To build the program, the NCSRobotLib needs to be build first by the following terminal commands:

cd /NCSRobotLib/build
cmake ../
make

After building the library enter the association_net folder and type:

cd /associaion_net/build
cmake ../
make

To execute the program, the ROLLS neuromorphic chip and the Omnibot are required.

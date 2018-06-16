cd NCSRobotLib
rm -r build
mkdir build
cd build
cmake ../
make

cd association_net
rm -r build
mkdir build
cd build
cmake ../
make

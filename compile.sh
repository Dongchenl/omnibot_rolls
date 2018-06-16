git pull
cd NCSRobotLib
rm -r build
mkdir build
cd build
cmake ../
make

git pull
cd association_net
rm -r build
mkdir build
cd build
cmake ../
make

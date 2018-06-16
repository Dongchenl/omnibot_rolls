#ifndef NCSROBOTEXAMPLES_OMNIBOT_PARALLELLA_MARIO_INCLUDE_DATAGENERATOR_H_
#define NCSROBOTEXAMPLES_OMNIBOT_PARALLELLA_MARIO_INCLUDE_DATAGENERATOR_H_

#include "DataGeneratorListener.h"
#include <pthread.h>
#include <list>
#include <iostream>
#include <fstream>

#include <boost/chrono/thread_clock.hpp>
using namespace boost::chrono;

struct Position
{
    int coord;
    int direction;
    int step;
};

class DataGenerator
{ //: public DataGeneratorListener {
  public:
    DataGenerator(void);
    ~DataGenerator(void);
    //static void* loop(void* obj);
    void generateDataFromOmnibotLogFile();
 		void generateData(int max_val);
    void listen();
    void stopListening();
    void registerListener(DataGeneratorListener *listener);
    void deregisterListener(DataGeneratorListener *listener);
    void warnEvent(int xpos, int ypos, int bumper);

    bool m_toggleListening;

  private:
    pthread_t thread;

    float xMin;
    float xMax;
    float yMin;
    float yMax;

    boost::chrono::thread_clock::time_point start_time;
    std::ofstream logstream;
		std::ifstream omnibot_log_file;

    std::list<DataGeneratorListener *> m_listeners;

    /**
      Changes the moving direction of the robot when it exits the box

      @param coordinate Current x or y coordinate of the robot
      @param max_val Maximum value of the coordinate that should not be exceeded
      @return One if current position is inside the box, minus one otherwise 
  */
    Position changeDirection(Position position, int max_val);

    /**
      Generates random step size 
    */
    int rand_step();

    /**
     * Pefrorms coordinate increment in the current direction
     * 
     * @param position Current x or y positition
     * @param max_val Maximum value of the coordinate that should not be exceeded
     * @param step Step of the grid
     * @return New x or y position
     * */
    Position move(Position position, int max_val, int step);
};

#endif /* NCSROBOTEXAMPLES_OMNIBOT_PARALLELLA_MARIO_INCLUDE_DATAGENERATOR_H_ */

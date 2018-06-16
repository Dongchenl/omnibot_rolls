#include <pthread.h>
#include <list>
#include <iostream>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <random>

#include "DataGenerator.h"

using namespace boost::chrono;

DataGenerator::DataGenerator()
{

    start_time = boost::chrono::thread_clock::now();

    omnibot_log_file.open("visualizerLog.csv", std::ios_base::in);
    
    // read max and min values for x and y
    unsigned long t;
    int bumper;
    int num;
    omnibot_log_file >> num >> xMax >> yMax >> t >> bumper;
    omnibot_log_file >> num >> xMin >> yMin >> t >> bumper;

     if (!omnibot_log_file.is_open())
        std::cerr << "**ERROR** opening the file: visualizerLog.csv" << std::endl;
}

void *loop(void *obj)
{
    DataGenerator *dataGen = (DataGenerator *)obj;
    while (true)
    {
        if (dataGen->m_toggleListening == false)
        {
            break;
        }
        //dataGen->generateData(90);
        dataGen->generateDataFromOmnibotLogFile();
        usleep(50000);
    }
}

void DataGenerator::generateDataFromOmnibotLogFile()
{
    int num;
    float x, y;
    unsigned long t;
    int bumper;

    float x_scaled, y_scaled; //scale from -120, 120 -> 0, 10
    int x_neuron, y_neuron;   // make int 

    if (!omnibot_log_file.eof())
        omnibot_log_file >> num >> x >> y >> t >> bumper;
    else
    {
        omnibot_log_file.close();
        return;
    }
    std::cout << "x: " << x << "; y: " << y << "; bumper: " << bumper << std::endl;

    // scale coordinates
    x_scaled = (x - xMin) / (xMax - xMin) * 10;
    y_scaled = (y - yMin) / (yMax - yMin) * 10;

    x_neuron = (int)x_scaled;
    y_neuron = (int)y_scaled;

    std::cout << "x_scaled: " << x_scaled << "; y_scaled: " << y_scaled << "; bumper: " << bumper << std::endl;

    warnEvent(x_neuron, y_neuron, (int)bumper);
}


void DataGenerator::listen()
{
    std::cout << "Starting DataGenerator listening\n";
    m_toggleListening = true;
    pthread_create(&thread, NULL, &loop, this);
}

void DataGenerator::stopListening()
{
    m_toggleListening = false;
    std::cout << "Stopped generating data\n";
    pthread_join(thread, 0);
}

void DataGenerator::registerListener(DataGeneratorListener *listener)
{
    m_listeners.push_back(listener);
}

void DataGenerator::deregisterListener(DataGeneratorListener *listener)
{
    m_listeners.remove(listener);
}

void DataGenerator::warnEvent(int xpos, int ypos, int bumper)
{
    std::list<DataGeneratorListener *>::iterator it;
    for (it = m_listeners.begin(); it != m_listeners.end(); it++)
    {
        (*it)->receivedNewDataEvent(xpos, ypos, bumper);
    }
}

Position DataGenerator::changeDirection(Position position, int max_val)
{
    if (position.coord == max_val || position.coord == 0)
    {
        Position res;
        res.coord = position.coord >= max_val ? max_val : 0;
        res.step = rand_step();
        res.direction = position.direction * (-1);
        return res;
    }
    else
    {
        return position;
    }
}

int DataGenerator::rand_step()
{
    // Seed with a real random value, if available
    std::random_device r;
    std::default_random_engine e(r());
    std::uniform_int_distribution<int> uniform_dist(1, 3);

    return uniform_dist(e);
}

Position DataGenerator::move(Position position, int max_val, int step)
{
    Position res;
    res.direction = position.direction;
    res.step = position.step;
    res.coord = position.coord + position.step * step * position.direction;
    if (res.coord > max_val)
    {
        res.coord = max_val;
    }
    if (res.coord < 0)
    {
        res.coord = 0;
    }
    return res;
}

#ifndef __ROLLS_LISTENER_DRIVE_H__
#define __ROLLS_LISTENER_DRIVE_H__

#include <ROLLSListener.h>
#include <ROLLSDevice.h>  // NeuronGroup
#include <PushBot.h>
#include <thread>
#include <mutex>
#include <functional>  // bind

class ROLLSListener_Drive : public ROLLSListener {
public:
    ROLLSListener_Drive(PushBot *robot,
        NeuronGroup speed, NeuronGroup left, NeuronGroup right,
        double speed_factor=1, double turn_factor=1)
    : pushbot(robot), speed(speed), drive_left(left), drive_right(right),
      speed_factor(speed_factor), turn_factor(turn_factor),
      motor_thread(std::bind(&ROLLSListener_Drive::loop, this)) {};

    virtual ~ROLLSListener_Drive() {_stop = true; motor_thread.join();}

    void receivedNewROLLSEvent(unsigned int neuron);

private:
    // Pushbot
    PushBot *pushbot;
    void loop();

    // Neuron Populations
    NeuronGroup speed, drive_left, drive_right;

    // counter and states for spikes
    int left_counter, right_counter, speed_counter;
    double n_left, n_right, n_speed;

    // Scaling for speed and turning
    double speed_factor;
    double turn_factor;

    // variables for motor thread
    std::thread motor_thread;
    std::mutex counter_mutex;
    bool _stop = false;

};

#endif

#ifndef ROLLS_DEVICE_H
#define ROLLS_DEVICE_H

#include <ROLLSListener.h>

#include <list>
#include <thread>
#include <mutex>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <unistd.h>

/*! \file ROLLSDevice.h
 *  \brief This file contains all the necessary interfaces to connect to ROLLS.
 */

enum SynapseMode { excitatory, inhibitory };

// Enum for nonplastic synapse weight
// weight_0: Only constant weight
// weight_1: Constant and first weight
// weight_2: Constant and second weight
// weight_3: Constant and both weights
enum SynapseWeight {weight_0, weight_1, weight_2, weight_3};

struct NeuronGroup {
public:
  // Constructors
  NeuronGroup(): start(0), end(0) {};
  NeuronGroup(unsigned char from, unsigned char to)
  : start(from), end(to) {};

  // Values, Neuron Range (first and last neuron index)
  unsigned char start;
  unsigned char end;

  // Function to check if a rolls event was fired by the group
  bool isOrigin(int event);

  // Utilities to get size of group and return all neurons as vector
  int size();
  std::vector<int> getVector();
};

/*! \class ROLLSDevice
 *  \brief Object to handle connection to ROLLS.
 */
class ROLLSDevice {
public:
    ROLLSDevice(std::list<int> = std::list<int>());
    ~ROLLSDevice();
    void loop(); //!< Listening thread
    void readEv();

    // Listener interface
    void registerListener(ROLLSListener * );
    void deregisterListener(ROLLSListener * );
    void warnEvent(unsigned int event);
    void listen(); //!< Starts a new thread listening
    void stopListening(); //!< Stop listening thread
    bool isListening(); //!< determines if an object is listening

    // ROLLS interface
    void stimulate(unsigned int neuron, unsigned int synapse = 1);
    void stimulate(NeuronGroup group, unsigned int synapse = 1);
    void probePlastic();
    void stimPlastic(unsigned int neuron, unsigned int synapse);
    void readPlasticSynapses(int start_neuron, int end_neuron);	
    int setNonplasticSynapse(unsigned char pre_neuron,
                             unsigned char post_neuron,
                             bool recurrent,
                             SynapseMode mode = excitatory,
                             SynapseWeight weight = weight_0);
    int setPlasticSynapse(unsigned char pre_neuron,
                          unsigned char post_neuron,
                          bool recurrent);
    bool isBadNeuron(int);

private:
    //!< Flag indicating if we are listening or not
    bool toggleListening = false;
    std::thread thread;
    std::mutex mutex;

    int aer_dev;
    int aer_head;
    bool _aer_device_open = false;

    unsigned int event;

    //!< List of the registered listeners
    std::list<ROLLSListener *> listeners;

    std::list<int> bad_neurons;
};

#endif

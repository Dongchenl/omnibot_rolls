#include "ROLLSDevice.h"
#include <functional>
#include <iostream>
#include <aerctl.h>
#include <algorithm>
#include <unistd.h>
#include <numeric>  // for iota

/*! \file ROLLSDevice.cpp
 *  \brief This file provides the implementation of the keyboard communication
 */

bool NeuronGroup::isOrigin(int event) {
    // The event is equal to the index of the neuron that fired.
    // Check if it is a neuron in the group
    return ( (event >= start) && (event <= end) );
}

int NeuronGroup::size() {
    // Return the size of the group
    return end-start + 1;
}

std::vector<int> NeuronGroup::getVector() {
    // Get vector of all elements in the Neuron group
    std::vector<int> v(size());
    // iota fills the container beginning with third argument and increasing
    std::iota(v.begin(), v.end(), start);
    return v;
}

ROLLSDevice::ROLLSDevice(std::list<int> neurons) : bad_neurons(neurons) {
    if (_aer_device_open == true) {
        std::cout<<"ROLLS device already open."<<std::endl;
    }
    else {
        std::cout<<"Opening ROLLS aer device: "<<std::endl;
        aer_dev = aerOpenDevice();
        if (aer_dev >= 0) {
            _aer_device_open = true;
            std::cout << "Successfully opened ROLLS device" << std::endl;
        } else {
            std::cout << "Could not open ROLLS device" << std::endl;
        }
    }
}

ROLLSDevice::~ROLLSDevice() {
    aerClose(aer_dev);
}

void ROLLSDevice::loop() {
    std::cout << "ROLLS loop started" << std::endl;

    mutex.lock();
    aer_head = aerFindHead();
    mutex.unlock();

    while (toggleListening) {
        readEv();
    }
}

void  ROLLSDevice::readEv() {
    mutex.lock();
    event = aerReadMem(2*aer_head);
    mutex.unlock();

    if (event != 0xffffffff) {
        aer_head = (aer_head+1)%AER_BUFFER_SIZE;

        // only warn if neuron is not blacklisted
        if (!isBadNeuron(event)) warnEvent(event);
    }
}

void ROLLSDevice::registerListener(ROLLSListener* listener) {
    listeners.push_back(listener);
}

void ROLLSDevice::deregisterListener(ROLLSListener* listener) {
    listeners.remove(listener);
}

void ROLLSDevice::warnEvent(unsigned int event) {
    std::list<ROLLSListener*>::const_iterator it;
    for(it = listeners.begin(); it!=listeners.end(); it++) {
        (*it)->receivedNewROLLSEvent(event);
    }
}

void ROLLSDevice::stimulate(unsigned int neuron, unsigned int synapse) {
    mutex.lock();
    aerStimSyn(VIRTUAL, neuron, synapse);
    mutex.unlock();
}

void ROLLSDevice::stimulate(NeuronGroup group, unsigned int synapse) {
    // Stimulate a whole group of neurons
    mutex.lock();
    for (int neuron = group.start; neuron <= group.end; neuron++) {
        aerStimSyn(VIRTUAL, neuron, synapse);
    }
    mutex.unlock();
}

void ROLLSDevice::probePlastic() {
    for (int i = 0; i<256; i++) {
        std::cout<<i<<std::endl;
        for (int j = 0; j<256; j++) {
            for (int t = 0; t<400; t++) {
                mutex.lock();
                aerStimSyn(PLASTIC, i, j);
                mutex.unlock();
            }
        }
    }
}

void ROLLSDevice::readPlasticSynapses(int start_neuron, int end_neuron) {

	// find local time      
	boost::posix_time::ptime start_t = boost::posix_time::microsec_clock::local_time();
	
	std::cout << "log files are set up in ../logs" << std::endl;
	std::string log_dir = "../logs";
	std::string exp_name = "plastic_syn";

	int diff = 0;
	boost::posix_time::ptime now, now_2;
        boost::posix_time::time_duration diff2; 
	
	// stimulate sync neuron with virtual synapses to start the readout, 700 ms
	std::cout  << "Start stimulating" << std::endl;
	while (diff < 700) {
                now = boost::posix_time::microsec_clock::local_time();
                diff2 = now - start_t;
                diff =  diff2.total_milliseconds();

		stimulate(200);
		usleep(32);
	}
	std::cout  << "Done stimulating the sync neuron" << std::endl;
	usleep(800000);
	
	// stimulate plastic syanpses of all neurons from 1 to 256 in different time windows
	std::cout  << "Start stimulating all neurons" << std::endl;
	for (int i = start_neuron; i < end_neuron+1; i++) {

		now = boost::posix_time::microsec_clock::local_time();
		diff = 0;
				
		// for 50 ms, changed to 500 
  		while (diff < 300) {
			now_2 = boost::posix_time::microsec_clock::local_time();
                	diff2 = now_2 - now;
                	diff = diff2.total_milliseconds();

			
			// stimulate neurons to test (from start to end neuron)
		  	stimulate(i);
			usleep(1.2);
		}
		while (diff>300 && diff< 150000) 
		{
			for (int j = 0; j < 256; j++) 
			{
				now_2 = boost::posix_time::microsec_clock::local_time();
	      			diff2 = now_2 - now;
	      			diff = diff2.total_milliseconds();
				stimulate(j,4),
				stimulate(j,5);
				stimulate(j,6);
				stimulate(j,7);
			}
			usleep(0.4);
		}
		usleep(50000);
	}
	std::cout  << "Done stimulating neurons for readout plastic" << std::endl;
	
	// stimulate sync neuron with end spikes
	now = boost::posix_time::microsec_clock::local_time();        
	diff = 0;
	while (diff < 700) {
                now_2 = boost::posix_time::microsec_clock::local_time();
                diff2 = now_2 - now;
                diff = diff2.total_milliseconds();
                stimulate(200);
                usleep(1.666);
        }
	std::cout  << "Done stimulating sync end neuron" << std::endl;
	std::cout  << "ROLLS stimulation done" << std::endl;		
  }

void ROLLSDevice::stimPlastic(unsigned int neuron, unsigned int synapse) {
     // Simulate Neuron through plastic synapses
     mutex.lock();
     aerStimSyn(0, neuron, synapse);
     mutex.unlock();
}

void  ROLLSDevice::listen( void ) {
    if (!toggleListening) {
        std::cout<<"ROLLSDevice: Activate listening"<<std::endl;
        toggleListening = true;
        thread = std::thread(std::bind(&ROLLSDevice::loop, this));
    }
    else {
        std::cout<< "ROLLSDevice: Already listening, no action required"
                 << std::endl;
    }
}

bool ROLLSDevice::isListening() {
    return toggleListening;
}

void  ROLLSDevice::stopListening( void ) {
    toggleListening = false;
    thread.join();
    std::cout << "Stop listening" << std::endl;
}

int ROLLSDevice::setNonplasticSynapse(unsigned char pre_neuron,
                                      unsigned char post_neuron,
                                      bool recurrent,
                                      SynapseMode mode,
                                      SynapseWeight weight) {
    Synapse synapse;
    synapse.type = NONPLASTIC;
    synapse.source = pre_neuron;
    synapse.target = post_neuron;
    synapse.recurrent = recurrent;
    // Convert mode to flag
    if (mode == excitatory) {
        synapse.mode = EXC;
    } else {
        synapse.mode = INH;
    }
    // Convert weight to flags for the two weight bits
    switch (weight) {
        case weight_0:
            synapse.wgt1 = DISABLE;
            synapse.wgt2 = DISABLE;
            break;
        case weight_1:
            synapse.wgt1 = ENABLE;
            synapse.wgt2 = DISABLE;
            break;
        case weight_2:
            synapse.wgt1 = DISABLE;
            synapse.wgt2 = ENABLE;
            break;
        case weight_3:
            synapse.wgt1 = ENABLE;
            synapse.wgt2 = ENABLE;
            break;
    }
    mutex.lock();
    std::cout << "Nonplastic: " << (int) pre_neuron << " to "
              << (int) post_neuron << std::endl;
    int result = aerSetConn(&synapse);
    mutex.unlock();
    return result;
}

int ROLLSDevice::setPlasticSynapse(unsigned char pre_neuron,
                                   unsigned char post_neuron,
                                   bool recurrent){
    Synapse synapse;
    synapse.type = PLASTIC;
    synapse.source = pre_neuron;
    synapse.target = post_neuron;
    synapse.recurrent = recurrent;
    synapse.wgt1 = 0;

    // We can safely ignore the other fields of synapse since the aerctl code
    // will not use them.

    mutex.lock();
    std::cout << "Plastic: " << (int) pre_neuron << " to "
              << (int) post_neuron << std::endl;
    int result = aerSetConn(&synapse);
    mutex.unlock();
    return result;
}

bool ROLLSDevice::isBadNeuron(int neuron) {
    // checks if a neuron is blacklisted
    return (std::find(bad_neurons.begin(), bad_neurons.end(), neuron)
           != bad_neurons.end());
}


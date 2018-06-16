#ifndef ROLLS_ARCHITECTURE_H
#define ROLLS_ARCHITECTURE_H

#include <ROLLSDevice.h>

#define N_NEURONS 256

/*! \file ROLLSArchitecture.h
 *  \brief Providing a interface to specify a synapse architecture for ROLLS.
 */

/*! \class ROLLSArchitecture
 *  \brief Provides an easy interface to set up a roll architecture
 */
class ROLLSArchitecture {
public:
    // Matrices for plastic and non-plastic synapses
    // Format = Connection[pre_neuron][post_neuron]
    bool plastic [N_NEURONS][N_NEURONS] = { { false } };
    int nonplastic [N_NEURONS][N_NEURONS] = { { 0 } };

    // Method to connect Plastic NeuronGroups
    void connectPlastic(NeuronGroup source, NeuronGroup target, bool setting);

    // Method to connect plastic neurons
    void connectPlastic(int source, int target, bool setting);
    
    // Methods to connect NeuronGroups easily
    void connectNonplastic(NeuronGroup source, NeuronGroup target, int weight);
    void connectNonplastic(NeuronGroup source, int target, int weight);
    void connectNonplastic(int source, NeuronGroup target, int weight);

    // Method to connect neorons
    void connectNonplastic(int source, int target, int weight);

    // Method to connect neurons within a group exciatory
    void connectNonplastic(NeuronGroup group, int weight);

    // Method to apply a architecture to ROLLS
    void apply(ROLLSDevice& rolls);

    void connectNonplasticKernel(
        NeuronGroup group, std::vector<int> kernel, int w_global);

    void connectNonplasticFeedForward(
        NeuronGroup source, NeuronGroup target, int weight);

    void connectNonplasticRandomSource(
        NeuronGroup source, NeuronGroup target, int weight,
        std::function<double(int, NeuronGroup)> func);

    void connectNonplasticRandomTarget(
        NeuronGroup source, NeuronGroup target, int weight,
        std::function<double(int, NeuronGroup)> func);
};

#endif

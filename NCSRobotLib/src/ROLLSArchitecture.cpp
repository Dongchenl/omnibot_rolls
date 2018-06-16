#include <ROLLSArchitecture.h>
#include <random>
#include <algorithm>
#include <numeric>  // ceil

void ROLLSArchitecture::apply(ROLLSDevice& rolls) {
    int setting;
    SynapseMode mode = excitatory;
    SynapseWeight weight = weight_0;

    // Iterate over all neuron combinations
    for (int pre = 0; pre < N_NEURONS; pre++) {
        for (int post = 0; post < N_NEURONS; post++) {
            // don't connect blacklisted neurons
            if (rolls.isBadNeuron(pre) || rolls.isBadNeuron(post))
                continue;

            // Nonplastic synapse
            setting = nonplastic[pre][post];
            if (setting == 0) {
                // Disconnect, a.k.a recurrent = 0
                rolls.setNonplasticSynapse(pre, post, false);
            } else {
                // Excitatory (positive) or inhibitory (negative)
                if (setting > 0) {
                    mode = excitatory;
                } else {
                    setting *= -1;  // To choose weight later
                    mode = inhibitory;
                }

                // Choose weight
                switch(setting) {
                    case 1:
                        weight = weight_0;
                        break;
                    case 2:
                        weight = weight_1;
                        break;
                    case 3:
                        weight = weight_2;
                        break;
                    case 4:
                        weight = weight_3;
                        break;
                }

                rolls.setNonplasticSynapse(pre, post, true, mode, weight);
            }

            // Plastic synapse
            setting = plastic[pre][post];
            rolls.setPlasticSynapse(pre, post, setting);
        }
    }
}

void ROLLSArchitecture::connectPlastic(NeuronGroup source, NeuronGroup target, bool connection) 
{
	for (int src = source.start; src <= source.end; src++) {
		for (int tgt = target.start; tgt <=target.end; tgt++) {
			plastic[src][tgt] = connection;
		}
	}
}

void ROLLSArchitecture::connectPlastic(
		                int pre, int post, bool connection) 
{
	plastic[pre][post] = connection;
}

void ROLLSArchitecture::connectNonplastic(
        NeuronGroup source, NeuronGroup target, int weight) {
    for (int src = source.start; src <= source.end; src++) {
        for (int tgt = target.start; tgt <= target.end; tgt++) {
            nonplastic[src][tgt] = weight;
        }
    }
}

void ROLLSArchitecture::connectNonplastic(
        NeuronGroup source, int tgt, int weight) {
    for (int src = source.start; src <= source.end; src++) {
        nonplastic[src][tgt] = weight;
    }
}

void ROLLSArchitecture::connectNonplastic(
        int src, NeuronGroup target, int weight) {
    for (int tgt = target.start; tgt <= target.end; tgt++) {
        nonplastic[src][tgt] = weight;
    }
}

void ROLLSArchitecture::connectNonplastic(int pre, int post, int weight) {
    nonplastic[pre][post] = weight;
}

void ROLLSArchitecture::connectNonplastic(NeuronGroup group, int weight) {
    connectNonplastic(group, group, weight);
}

void ROLLSArchitecture::connectNonplasticKernel(
        NeuronGroup group, std::vector<int> kernel, int w_global) {
    // Connect every neuron with its neighbors according to the kernel
    // and to every other neuron as specified with w_global
    for (int src = group.start; src <= group.end; src++) {
        for (int tgt = group.start; tgt <= group.end; tgt++) {
            unsigned int distance = abs(tgt - src);
            if (distance < kernel.size()) {
                connectNonplastic(src, tgt, kernel[distance]);
            } else {
                connectNonplastic(src, tgt, w_global);
            }
        }
    }
}

void ROLLSArchitecture::connectNonplasticFeedForward(
        NeuronGroup source, NeuronGroup target, int weight) {
    // we know that source and target have the same size
    for (int idx = 0; idx < source.size(); idx++) {
        connectNonplastic(source.start + idx, target.start + idx, weight);
    }
}

void ROLLSArchitecture::connectNonplasticRandomSource(
        NeuronGroup source, NeuronGroup target, int weight,
        std::function<double(int, NeuronGroup)> func) {
    std::mt19937 generator;  // random number generator, initialized with
                             // always equal default seed

    // Get vector of source neurons and shuffle
    auto source_neurons = source.getVector();
    std::shuffle(source_neurons.begin(), source_neurons.end(), generator);

    unsigned int idx = 0;
    for (int tgt = target.start; tgt <= target.end; tgt++) {
        // evaluate given weight function for current target
        const double factor = func(tgt, target);
        // Ceil to better use available connections, only zero if factor == 0
        const int num_sources = (int) std::ceil(factor * source.size());
        for (int i = 0; i < num_sources; i++) {
            connectNonplastic(source_neurons[idx++], tgt, weight);

            // reset index if end of source_neurons is reached
            if (idx == source_neurons.size()) idx = 0;
        }
    }
}

void ROLLSArchitecture::connectNonplasticRandomTarget(
        NeuronGroup source, NeuronGroup target, int weight,
        std::function<double(int, NeuronGroup)> func) {
    std::mt19937 generator;  // random number generator, initialized with
                             // always equal default seed

    // Get vector of target neurons and shuffle
    auto target_neurons = target.getVector();
    std::shuffle(target_neurons.begin(), target_neurons.end(), generator);

    unsigned int idx = 0;
    for (int src = source.start; src <= source.end; src++) {
        // evaluate given weight function for current source
        const double factor = func(src, source);
        // Ceil to better use available connections, only zero if factor == 0
        const int num_targets = (int) std::ceil(factor * target.size());
        for (int i = 0; i < num_targets; i++) {
            connectNonplastic(src, target_neurons[idx++], weight);

            // reset index if end of target_neurons is reached
            if (idx == target_neurons.size()) idx = 0;
        }
    }
}

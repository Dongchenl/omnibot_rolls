#include <ROLLSListener_Log.h>

void ROLLSListener_Log::receivedNewROLLSEvent(unsigned int neuron) {
    // time since listening started, in ms
    pt::time_duration duration = pt::microsec_clock::local_time() - start;
    timestamp = duration.total_milliseconds();

    fprintf(log, "%ld,%u\n", timestamp, neuron);
}

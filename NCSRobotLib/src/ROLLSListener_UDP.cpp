#include <ROLLSListener_UDP.h>

void ROLLSListener_UDP::receivedNewROLLSEvent(unsigned int neuron)
{
    // time since listening started, in ms
    pt::time_duration duration = pt::microsec_clock::local_time() - start;
    long timestamp = duration.total_milliseconds();

    sprintf(buffer, "%ld,%u\n", timestamp, neuron);
    send(buffer);
}

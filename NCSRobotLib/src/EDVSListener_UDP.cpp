#include <EDVSListener_UDP.h>
#include <iostream>

void EDVSListener_UDP::receivedNewEDVSEvent(EDVSEvent &e)
{
    // check if update timestamp
    if (--delay < 0) {
        delay = TIMESTAMP_RENEWAL_DELAY;
        // time since listening started, in ms
        duration = pt::microsec_clock::local_time() - start;
        timestamp = duration.total_milliseconds();
    }

    fprintf(tempfile, "%ld,%d,%d,%d\n", timestamp, e.m_x, e.m_y, e.m_pol);
};

EDVSListener_UDP::~EDVSListener_UDP() {
    // Go to start of file
    std::rewind(tempfile);

    std::cout << "Sending eDVS events over UDP..." << std::endl;
    while (std::fgets(buffer, sizeof buffer, tempfile) != NULL) {
        send(buffer);
    }
}

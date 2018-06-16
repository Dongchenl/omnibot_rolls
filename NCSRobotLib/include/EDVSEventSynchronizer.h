#ifndef __EDVSEVENTSYNCHRONIZER_H__
#define __EDVSEVENTSYNCHRONIZER_H__

#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <boost/chrono.hpp>

#include "EDVSListener.h"

#define N_AVERAGE 300

//! determines the offset between the timestamps from the EDVS and the computer
/*! This is needed as we need to synchronize all events from different sources, like EDVS, keyboard input and 
 * information from robots and other devices. To do this calibration a flashing window is displayed on the laptop 
 * and the eDVS has to be pointed to it. The calibration is then started by pressing the key 'o'.  For debugging 
 * purposes the events are displayed in a separate window, as well as the center of mass of the image.
 */

class EDVSEventSynchronizer : public EDVSListener
{
public:
    /*! 
     * \param orig_ts is the timestamp delivered by the computer
     */
    EDVSEventSynchronizer(boost::chrono::system_clock::time_point orig_ts);
    ~EDVSEventSynchronizer(void);

    float  m_centerOfMassX = 64.0;
    float  m_centerOfMassY = 64.0;

    void   receivedNewEDVSEvent(EDVSEvent &e);
    //! converts the 2D point indices with respect to the current center of mass to an 1D index of an array
    int    conv2D(int x, int y);
    void   receivedNewEDVSIMUEvent(EDVSIMUEvent &e);
    void   start(void);
    //! return the time offset between computer and eDVS in us
    double get_time_offset();

private:
    cv::Mat         m_coloredImg;
    cv::Mat         m_coloredFrame;
    cv::Mat         m_frame;

    std::string     m_window = "Calibration Tool";
    std::string     m_window2 = "EDVSDevice -- Green / Red Contrast";
    std::string     m_slider = "Frame delay";
    std::string     m_slider2 = "FPS";

    long long *old_ts;

    int m_fps = 25;
    int m_idx = 0;
    int m_contrast = 3;
    int m_delay = 20;
    unsigned int window = 4;

    int context_size;
    
    bool waitToStartInit = true;
    bool synchronizationOver = false;

    boost::chrono::system_clock::time_point t_start;
    double t_ref;

    double time_offset = 0;

    double offset_table[N_AVERAGE];
    unsigned int offset_cpt = 0;  
};

#endif

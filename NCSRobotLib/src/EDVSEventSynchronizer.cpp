#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <boost/chrono.hpp>

#include "EDVSEventSynchronizer.h"

enum displayState{ OFF, ON };

EDVSEventSynchronizer::EDVSEventSynchronizer(boost::chrono::system_clock::time_point orig_ts)
{
    cv::namedWindow(m_window,0);
    cv::namedWindow(m_window2,0);

    /* Setup Windows to display the DVS framed events */
    m_coloredImg.create(128,128,CV_8UC3);
    m_frame.create(128,128,CV_8UC1);
    m_coloredFrame.create(128,128,CV_8UC3);
    cv::createTrackbar(m_slider, m_window, &m_delay, 30, NULL);
    cv::createTrackbar(m_slider2, m_window2, &m_fps, 100, NULL);
    // move windows
    cvMoveWindow("Calibration Tool",200,200);
    cvMoveWindow("EDVSDevice - Green/Red Contrast",200,400);

    context_size = (2*(window-1)+1)*(2*(window-1)+1);

    old_ts = new long long[context_size];

    t_start = orig_ts;
}

EDVSEventSynchronizer::~EDVSEventSynchronizer(void)
{
    delete[] old_ts;
}

void EDVSEventSynchronizer::receivedNewEDVSEvent(EDVSEvent &e)
{
m_coloredFrame.data[e.m_x*128*3 + e.m_y*3 + 0]
    += (e.m_pol==1) ? (unsigned char)1 : (unsigned char)0;
m_coloredFrame.data[e.m_x*128*3 + e.m_y*3 + 1]
    += (e.m_pol==1) ? (unsigned char)0 : (unsigned char)1;
    
if( waitToStartInit )
{
    /* Online mean-shift + ring buffer gradient computation */
    m_centerOfMassX = 0.999*m_centerOfMassX + 0.001*e.m_x;
    m_centerOfMassY = 0.999*m_centerOfMassY + 0.001*e.m_y;
}
else
    if( !synchronizationOver )
    {
        if( e.m_x > ( (unsigned int)m_centerOfMassX - window ) && e.m_x < ( (unsigned int)m_centerOfMassX + window ) )
        {
            if( e.m_y > ( (unsigned int)m_centerOfMassY - window ) && e.m_y < ( (unsigned int)m_centerOfMassY + window ) )
            {
                old_ts[conv2D(e.m_x,e.m_y)] = e.m_timestamp;
                int valid_cpt = 0;
                for( int n = 0; n < context_size; n++)
                {
                    if( e.m_timestamp - old_ts[n] < 15000 )
                        valid_cpt++;
                }
                if( valid_cpt > 40 )
                {
                    if(offset_cpt < N_AVERAGE)
                    {
                        offset_table[offset_cpt++] = e.m_timestamp/1E6-t_ref;
                        std::cout << offset_cpt << " " << e.m_x << " " << e.m_y << " " << e.m_timestamp << " " << t_ref << "\n";
                    }
                    else
                    {
                        synchronizationOver = true;
                        double loc = 0;
                        for( unsigned int m = 0; m < N_AVERAGE ; m++)
                        {
                            loc += offset_table[m];
                        }
                        time_offset = loc/N_AVERAGE;
                        std::cout << "Time offset average is : " << time_offset << " ms\n";
                        // Set record variable for EDVS logger
                    }
                }
            }
        }
    }
}

int EDVSEventSynchronizer::conv2D(int x, int y)
{
    int loc_x = x - ((int)m_centerOfMassX - 3);
    int loc_y = y - ((int)m_centerOfMassY - 3);
    
    return loc_x+loc_y*(2*(window-1)+1);
}

void EDVSEventSynchronizer::receivedNewEDVSIMUEvent(EDVSIMUEvent &e)
{
}

void EDVSEventSynchronizer::start(void)
{
    std::cout << ">> Started calibration" << std::endl;
    char key = ' ';
    displayState state = OFF;
    int          cpt   = 0;
    int          val   = 0;
    
    while( !synchronizationOver ) // <-- here you could add an escape character
    {
        if( m_fps != 0 )
        {
            cv::circle( m_coloredImg,cv::Point(m_centerOfMassY,m_centerOfMassX),3,cv::Scalar(255,255,0) );
            cv::circle( m_coloredImg,cv::Point(64,64),3,cv::Scalar(0,255,255) );

            cv::imshow( m_window, m_frame );
            cv::imshow( m_window2, m_coloredImg );
            boost::chrono::duration<double> ref = boost::chrono::system_clock::now() - t_start;
            t_ref = ref.count();

            m_coloredImg = 0.3*m_coloredImg + 0.7*m_coloredFrame;
            for( int i = 0; i < 128*128; i++ )
            {
                m_coloredImg.data[3*i + 1]
                    = (unsigned char) (255*std::min(m_coloredFrame.data[3*i]/float(m_contrast),float(m_contrast)));

                m_coloredImg.data[3*i + 2]
                    = (unsigned char) (255*std::min(m_coloredFrame.data[3*i+1]/float(m_contrast),float(m_contrast)));
            }
            
            // Generate the blinking image
            cpt++;
            if(cpt >= m_delay)
            {
                cpt = 0;
                if( state == ON )
                {
                    val = 0;
                    state = OFF;
                }
                else
                {
                    val = 255;
                    state = ON;
                }
            }
            for(int i=0;i<128*128;i++)
            {
                m_frame.data[i] = val;
                m_coloredFrame.data[3*i + 0] = 0;
                m_coloredFrame.data[3*i + 1] = 0;
                m_coloredFrame.data[3*i + 2] = 0;
            }
            
            key = cv::waitKey(m_fps); /* waitKey(x) waits/sleeps/blocks for x milliseconds and logs keypresses while the window is active (and only if the window is active!) */

            if(key == 'o') /* press key o to start synchronization */
            {
                waitToStartInit = false;
                printf("Focused on (x,y) = (%f,%f)\n",m_centerOfMassX,m_centerOfMassY);
            }
        }
    }
}

double EDVSEventSynchronizer::get_time_offset()
{
    return time_offset;
}

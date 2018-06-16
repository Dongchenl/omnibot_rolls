#ifndef __DUMMY_LISTENER_H__
#define __DUMMY_LISTENER_H__

#include <stdio.h>
#include <opencv2/opencv.hpp>

#define HISTOGRAM_BUCKETS 10 
#define BUCKET_SIZE HISTOGRAM_BUCKETS/128.

class DumbEDVS4337Listener : public EDVS4337SerialUSBListener
{

    private:
        cv::Mat         m_img;
        cv::Mat         m_coloredImg;
        cv::Mat         m_histogramImg;
        cv::Mat         m_coloredFrame;
        cv::Mat         m_frame;

        std::string     m_window;
        std::string     m_window2;
        std::string     m_slider;

        // Biases settings
        //UserData        m_userData;

        int 						 m_fps;
        int             m_idx;
        int             m_contrast;

        int clearCount;

    public:
        DumbEDVS4337Listener()
            : m_window("EDVS4377SerialUSB - Gray"),
            m_window2("Horizontal histogram"),
            m_slider("Frame delay"),
            m_contrast(3),
            m_fps(25),
            m_idx(0),
            clearCount(0)
              //  m_userData()
    {

        cv::namedWindow(m_window,0);
        cv::namedWindow(m_window2,0);

        // Display the DVS framed events
        m_img.create(128,128,CV_8UC1);
        m_coloredImg.create(128,128,CV_8UC3);
        m_histogramImg.create(128,128,CV_8UC1);
        m_frame.create(128,128,CV_8UC1);
        m_coloredFrame.create(128,128,CV_8UC3);
        cv::createTrackbar(m_slider, m_window, &m_fps, 100, NULL);
    }

        ~DumbEDVS4337Listener(void)
        {
        }

        unsigned int m_historySize = 20;

        float m_centerOfMassX = 0.0;
        float m_centerOfMassY = 0.0;

        std::list<float> m_historyX;
        std::list<float> m_historyY;
        int i = 0;
        float gradX1 = 0.0;
        float gradX2 = 0.0;
        float gradY1 = 0.0;
        float gradY2 = 0.0;
        int iter =  0;

        int histogram[HISTOGRAM_BUCKETS] = {0};

        void receivedNewEDVS4337SerialUSBEvent(EDVS4337SerialUSBEvent &e)
        {
            if (rand()%100 > 10) {
                return;
            }
            //printf("p=%d (x,y)=(%d,%d) ts=%ld\n ",  e.m_pol,
            //        e.m_x,
            //        e.m_y,
            //        e.m_timestamp
            //      );
            //fflush(stdout);

            /* Online mean-shift + ring buffer gradient computation */
            m_centerOfMassX = 0.99*m_centerOfMassX + 0.01*e.m_x;
            m_centerOfMassY = 0.99*m_centerOfMassY + 0.01*e.m_y;

            m_frame.data[e.m_x*128 + e.m_y]
                = (e.m_pol==1) ? (unsigned char)0 : (unsigned char)255;

            m_coloredFrame.data[e.m_x*128*3 + e.m_y*3 + 0]
                += (e.m_pol==1) ? (unsigned char)1 : (unsigned char)0;
            m_coloredFrame.data[e.m_x*128*3 + e.m_y*3 + 1]
                += (e.m_pol==1) ? (unsigned char)0 : (unsigned char)1;

            if (e.m_x > 64) {
                int index = e.m_y * BUCKET_SIZE;
                ++histogram[index];
            }
        }

        void receivedNewEDVS4337SerialUSBIMUEvent(EDVS4337SerialUSBIMUEvent &e)
        {
        }

        void start(void)
        {
            // int counter = 0;
            std::ostringstream oss;

            char key = '0';
            while(key != 'p')
            {

                if(m_historyX.size() < m_historySize)
                {
                    m_historyX.push_back(m_centerOfMassX);
                }
                else
                {
                    m_historyX.pop_front();
                    m_historyX.push_back(m_centerOfMassX);
                }

                if(m_historyY.size() < m_historySize)
                {
                    m_historyY.push_back(m_centerOfMassY);
                }
                else
                {
                    m_historyY.pop_front();
                    m_historyY.push_back(m_centerOfMassY);
                }
                //==========================

                i = 0.0;
                gradX1 = 0.0;
                gradX2 = 0.0;
                for(std::list<float>::iterator it = m_historyX.begin();
                        it != m_historyX.end();
                        it++)
                {
                    if(i < 0.5*m_historySize)
                    {
                        gradX1 += (*it) / (0.5*m_historySize);
                    }
                    else
                    {
                        gradX2 += (*it) / (0.5*m_historySize+1);
                    }
                    i++;
                }
                i = 0.0;
                gradY1 = 0.0;
                gradY2 = 0.0;
                for(std::list<float>::iterator it = m_historyY.begin();
                        it != m_historyY.end();
                        it++)
                {
                    if(i < 0.5*m_historySize)
                    {
                        gradY1 += (*it) / (0.5*m_historySize);
                    }
                    else
                    {
                        gradY2 += (*it) / (0.5*m_historySize+1);
                    }
                    i++;
                }
                // ==========================================

                cv::circle(m_coloredImg,cv::Point(m_centerOfMassY,m_centerOfMassX),3,cv::Scalar(255,255,0));
                cv::circle(m_coloredImg,cv::Point(64,64),3,cv::Scalar(0,255,255));

                cv::line(m_coloredImg,
                        cv::Point(64,64),
                        cv::Point(64,64)+cv::Point(gradY2-gradY1,gradX2-gradX1),
                        cv::Scalar(0,255,255)
                        );

                for (int i=0; i < 128*128; ++i) {
                    m_histogramImg.data[i] = 255;
                }

                for (int i=0; i < HISTOGRAM_BUCKETS; ++i) {
                    histogram[i] = histogram[i] * 0.8 ;
                }

                int max = 0;
                for (int i = 0; i < HISTOGRAM_BUCKETS; ++i) {
                    if (histogram[i] > max) {
                        max = histogram[i];
                    }
                }
                float normalization = 1;
                if (max < 128) {
                    normalization = 128./max;
                }
                // to prevent all-black images on low bucket values
                if (max < 2) {
                    normalization = 0;
                }
                for (int i = 0; i < HISTOGRAM_BUCKETS; ++i) {
                    int width = BUCKET_SIZE;
                    cv::Point top(i*width, 128 - histogram[i]*normalization);
                    cv::Point bottom(i*width, 128);
                    cv::rectangle(m_histogramImg, top, bottom, cv::Scalar(0,255,0), width);
                }

                cv::imshow(m_window,m_coloredImg);
                cv::imshow(m_window2,m_histogramImg);

                // Black&White Frame/Image
                m_img = 0.3*m_img + 0.7*m_frame;

                // Colored Frame/Image
                m_coloredImg = 0.3*m_coloredImg + 0.7*m_coloredFrame;
                for(int i=0;i<128*128;i++)
                {
                    m_coloredImg.data[3*i + 1]
                        = (unsigned char) (255*std::min(m_coloredFrame.data[3*i]/float(m_contrast),float(m_contrast)));

                    m_coloredImg.data[3*i + 2]
                        = (unsigned char) (255*std::min(m_coloredFrame.data[3*i+1]/float(m_contrast),float(m_contrast)));
                }

                // Reset frames
                for(int i=0;i<128*128;i++)
                {
                    m_frame.data[i] = 127;
                    m_coloredFrame.data[3*i + 0] = 0;
                    m_coloredFrame.data[3*i + 1] = 0;
                    m_coloredFrame.data[3*i + 2] = 0;
                }

                if(m_fps == 0)
                    m_fps = 1;
                key = cv::waitKey(m_fps);
            }
        }

};

#endif

#ifndef __EDVS_LISTENER_DISPLAY_EVENTS_H__
#define __EDVS_LISTENER_DISPLAY_EVENTS_H__

#include <list>
#include <stdio.h>
#include <opencv2/opencv.hpp>

#include "EDVSListener.h"

class EDVSListener_DisplayEvents : public EDVSListener
{

private:
   cv::Mat         m_img;
   cv::Mat         m_coloredImg;
   cv::Mat         m_coloredFrame;
   cv::Mat         m_frame;

   std::string     m_window;
   std::string     m_window2;
   std::string     m_slider;
   std::string     m_slider2;

   // Biases settings
   //UserData        m_userData;

   int m_fps;
   int m_idx;
   int m_contrast;
  
public: 
   unsigned int m_historySize;

   float m_centerOfMassX;
   float m_centerOfMassY;

   std::list<float> m_historyX;
   std::list<float> m_historyY;
   int i;
   float gradX1;
   float gradX2;
   float gradY1;
   float gradY2;
   int iter;

public:
   EDVSListener_DisplayEvents();

   ~EDVSListener_DisplayEvents(void);

	 void receivedNewEDVSIMUEvent(EDVSIMUEvent &e);
   
   void receivedNewEDVSEvent(EDVSEvent &e);

   void start(void);

};

#endif

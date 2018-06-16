#include "EDVSListener_DisplayEvents.h"

EDVSListener_DisplayEvents::EDVSListener_DisplayEvents()
       : m_window("EDVSDevice - Gray"),
         m_window2("EDVSDevice - Green/Red Contrast"),
         m_slider("Frame delay"),
         m_slider2("Contrast")
   {
		   
   		m_fps = 25;
   		m_idx = 0;
   		m_contrast = 3;
  
   		m_historySize = 20;

   		m_centerOfMassX = 0.0;
   		m_centerOfMassY = 0.0;

      i = 0;
   		gradX1 = 0.0;
   		gradX2 = 0.0;
   		gradY1 = 0.0;
   		gradY2 = 0.0;
   		iter =  0;
     
       #ifdef __APPLE__
       cv::namedWindow(m_window,1);
       cv::namedWindow(m_window2,1);
       #else
       cv::namedWindow(m_window,0);
       cv::namedWindow(m_window2,0);
       #endif

       // Display the DVS framed events
       m_img.create(128,128,CV_8UC1);
       m_coloredImg.create(128,128,CV_8UC3);
       m_frame.create(128,128,CV_8UC1);
       m_coloredFrame.create(128,128,CV_8UC3);
       cv::createTrackbar(m_slider, m_window, &m_fps, 100, NULL);
       cv::createTrackbar(m_slider2, m_window2, &m_contrast, 10, NULL);
   }

   EDVSListener_DisplayEvents::~EDVSListener_DisplayEvents(void)
   {
   }


	 void EDVSListener_DisplayEvents::receivedNewEDVSIMUEvent(EDVSIMUEvent &e)
	 {
	 }
   
   void EDVSListener_DisplayEvents::receivedNewEDVSEvent(EDVSEvent &e)
   {
       // printf("received a new event\n");
       #ifdef VERBOSE_DUMBDVS128USBLISTENER
       printf("p=%d (x,y)=(%d,%d) ts=%ld\n ",  e.m_pol,
                                               e.m_x,
                                               e.m_y,
                                               e.m_timestamp
             );
       fflush(stdout);
       #endif

       /* Online mean-shift + ring buffer gradient computation */
       m_centerOfMassX = 0.99*m_centerOfMassX + 0.01*e.m_x;
       m_centerOfMassY = 0.99*m_centerOfMassY + 0.01*e.m_y;

       m_frame.data[e.m_x*128 + e.m_y]
               = (e.m_pol==1) ? (unsigned char)0 : (unsigned char)255;

       m_coloredFrame.data[e.m_x*128*3 + e.m_y*3 + 0]
               += (e.m_pol==1) ? (unsigned char)1 : (unsigned char)0;
       m_coloredFrame.data[e.m_x*128*3 + e.m_y*3 + 1]
               += (e.m_pol==1) ? (unsigned char)0 : (unsigned char)1;
   }

   void EDVSListener_DisplayEvents::start(void)
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
           //std::cout << gradX2-gradX1 << std::endl;
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
           //std::cout << gradY2-gradY1 << std::endl;
           // ==========================================

					cv::circle(m_coloredImg,cv::Point(m_centerOfMassY,m_centerOfMassX),3,cv::Scalar(255,255,0));
					cv::circle(m_coloredImg,cv::Point(64,64),3,cv::Scalar(0,255,255));

          cv::line(m_coloredImg,
                    cv::Point(64,64),
										cv::Point(64,64)+cv::Point(gradY2-gradY1,gradX2-gradX1),
                    cv::Scalar(0,255,255)
                    );

           #ifdef __APPLE__
            cv::Mat imgResized;
						cv::resize(m_img,imgResized,cv::Size(m_img.rows,m_img.cols)*4);
            cv::imshow(m_window,imgResized);
           #else
               cv::imshow(m_window,m_img);
               cv::imshow(m_window2,m_coloredImg);
           #endif

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




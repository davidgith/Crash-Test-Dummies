/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdio.h>

// Qt stuff
#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QApplication>
#include <QGraphicsRectItem>
#include <QPen>
#include <QColor>

// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography

#include <opencv2/opencv_modules.hpp>

#ifdef HAVE_OPENCV_NONFREE
  #if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >=4
  #include <opencv2/nonfree/gpu.hpp>
  #include <opencv2/nonfree/features2d.hpp>
  #endif
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/xfeatures2d/cuda.hpp>
#endif


// From this project
//#include "/find_object_2d/include/find_object/ObjWidget.h"
//#include "find_object/QtOpenCV.h"

#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/xfeatures2d.hpp"
using namespace cv::xfeatures2d;

#include <ros/ros.h>
//#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
/*
// gets called whenever a new message is availible in the input puffer
void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl)
{
  *usl = *uslMsg;
}

// gets called whenever a new message is availible in the input puffer
void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
  *usf = *usfMsg;
}

// gets called whenever a new message is availible in the input puffer
void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
  *usr = *usrMsg;
}
*/

void showUsage()
{
	printf("\n");
	printf("Usage :\n");
	printf("  ./example object.png scene.png\n");
	exit(1);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  static int imagecounter = 0; 
  imagecounter++;
  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  
  if(imagecounter == 30)
    {
      cv::imshow("view", image);
      imagecounter = 0;
    } 

  cv::waitKey(1);
}

int main(int argc, char * argv[])
{
  // init this node
  ros::init(argc, argv, "sign_recognition_node");
  // get ros node handle
  ros::NodeHandle nh;
/*
  // sensor message container
  sensor_msgs::Range usr, usf, usl;
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &usf));

  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

*/

  ROS_INFO("Hello world!");

//   // Loop starts here:
//   // loop rate value is set in Hz
//   ros::Rate loop_rate(1);
//   while (ros::ok())
//   {
//  /*   // simple wall crash avoidance algorithm ..
//     if (usr.range < 0.3 && usl.range >= 0.3)
//     {
//       steering.data = -750;
//       motor.data = 300;
//     }
//     else if (usl.range < 0.3 && usr.range >= 0.3)
//     {
//       steering.data = 750;
//       motor.data = 300;
//     }
//     else if (usl.range > 0.5 && usr.range > 0.5)
//     {
//       steering.data = 0;
//       motor.data = 300;
//     }
//     else
//     {
//       steering.data = 0;
//       motor.data = 0;
//     }
//     if (usf.range < 0.3)
//     {
//       motor.data = 0;
//       steering.data = 0;
//     }

//     // publish command messages on their topics
//     motorCtrl.publish(motor);
//     steeringCtrl.publish(steering);
//     // side note: setting steering and motor even though nothing might have
//     // changed is actually stupid but for this demo it doesn't matter too much.
// */
//     // clear input/output buffers
//     ros::spinOnce();
//     // this is needed to ensure a const. loop rate
//     loop_rate.sleep();
//   }
if(argc<3)
	{
		ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1));
	}

  cv::namedWindow("view");
  cv::startWindowThread();
  //change the exposure
  cv::VideoCapture cap;
  cap.set(cv::CAP_PROP_EXPOSURE,10);
  ros::Rate loop_rate(25);
  while (ros::ok())
  {

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();


  }



	QTime time;

	// GUI stuff
	QApplication app(argc, argv);

	time.start();
	//Load as grayscale
	cv::Mat objectImg = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
	cv::Mat sceneImg = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

	if(!objectImg.empty() && !sceneImg.empty())
	{
		printf("Let's start");
		printf("Loading images: %d ms\n", time.restart());
		std::vector<cv::KeyPoint> objectKeypoints;
		std::vector<cv::KeyPoint> sceneKeypoints;
		cv::Mat objectDescriptors;
		cv::Mat sceneDescriptors;

		////////////////////////////
		// EXTRACT KEYPOINTS
		////////////////////////////
		cv::Ptr<cv::FeatureDetector> detector;
		// The detector can be any of (see OpenCV features2d.hpp):
#if CV_MAJOR_VERSION == 2
		// detector = cv::Ptr(new cv::DenseFeatureDetector());
		// detector = cv::Ptr(new cv::FastFeatureDetector());
		// detector = cv::Ptr(new cv::GFTTDetector());
		// detector = cv::Ptr(new cv::MSER());
		// detector = cv::Ptr(new cv::ORB());
		detector = cv::Ptr<cv::FeatureDetector>(new cv::SIFT());
		// detector = cv::Ptr(new cv::StarFeatureDetector());
		// detector = cv::Ptr(new cv::SURF(600.0));
		// detector = cv::Ptr(new cv::BRISK());
#else
		detector = cv::xfeatures2d::SIFT::create();
#endif
		detector->detect(objectImg, objectKeypoints);
		printf("Object: %d keypoints detected in %d ms\n", (int)objectKeypoints.size(), time.restart());
		detector->detect(sceneImg, sceneKeypoints);
		printf("Scene: %d keypoints detected in %d ms\n", (int)sceneKeypoints.size(), time.restart());

		////////////////////////////
		// EXTRACT DESCRIPTORS
		////////////////////////////
		cv::Ptr<cv::DescriptorExtractor> extractor;
#if CV_MAJOR_VERSION == 2
		// The extractor can be any of (see OpenCV features2d.hpp):
		// extractor = cv::Ptr(new cv::BriefDescriptorExtractor());
		// extractor = cv::Ptr(new cv::ORB());
		extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SIFT());
		// extractor = cv::Ptr(new cv::SURF(600.0));
		// extractor = cv::Ptr(new cv::BRISK());
		// extractor = cv::Ptr(new cv::FREAK());
#else
		extractor = cv::xfeatures2d::SIFT::create();
#endif
		extractor->compute(objectImg, objectKeypoints, objectDescriptors);
		printf("Object: %d descriptors extracted in %d ms\n", objectDescriptors.rows, time.restart());
		extractor->compute(sceneImg, sceneKeypoints, sceneDescriptors);
		printf("Scene: %d descriptors extracted in %d ms\n", sceneDescriptors.rows, time.restart());

		////////////////////////////
		// NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
		////////////////////////////
		cv::Mat results;
		cv::Mat dists;
		std::vector<std::vector<cv::DMatch> > matches;
		int k=2; // find the 2 nearest neighbors
		bool useBFMatcher = false; // SET TO TRUE TO USE BRUTE FORCE MATCHER
		if(objectDescriptors.type()==CV_8U)
		{
			// Binary descriptors detected (from ORB, Brief, BRISK, FREAK)
			printf("Binary descriptors detected...\n");
			if(useBFMatcher)
			{
				cv::BFMatcher matcher(cv::NORM_HAMMING); // use cv::NORM_HAMMING2 for ORB descriptor with WTA_K == 3 or 4 (see ORB constructor)
				matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
			}
			else
			{
				// Create Flann LSH index
				cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
				printf("Time creating FLANN LSH index = %d ms\n", time.restart());

				// search (nearest neighbor)
				flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
			}
		}
		else
		{
			// assume it is CV_32F
			printf("Float descriptors detected...\n");
			if(useBFMatcher)
			{
				cv::BFMatcher matcher(cv::NORM_L2);
				matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
			}
			else
			{
				// Create Flann KDTree index
				cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
				printf("Time creating FLANN KDTree index = %d ms\n", time.restart());

				// search (nearest neighbor)
				flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
			}
		}
		printf("Time nearest neighbor search = %d ms\n", time.restart());

		// Conversion to CV_32F if needed
		if(dists.type() == CV_32S)
		{
			cv::Mat temp;
			dists.convertTo(temp, CV_32F);
			dists = temp;
		}


		////////////////////////////
		// PROCESS NEAREST NEIGHBOR RESULTS
		////////////////////////////
		// Set gui data
//		ObjWidget objWidget(0, objectKeypoints, QMultiMap<int,int>(), cvtCvMat2QImage(objectImg));
//		ObjWidget sceneWidget(0, sceneKeypoints, QMultiMap<int,int>(), cvtCvMat2QImage(sceneImg));

		// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
		float nndrRatio = 0.8f;
		std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
		std::vector<int> indexes_1, indexes_2; // Used for homography
		std::vector<uchar> outlier_mask;  // Used for homography
		// Check if this descriptor matches with those of the objects
		if(!useBFMatcher)
		{
			for(int i=0; i<objectDescriptors.rows; ++i)
			{
				// Apply NNDR
				//printf("q=%d dist1=%f dist2=%f\n", i, dists.at<float>(i,0), dists.at<float>(i,1));
				if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 &&
				   dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
				{
					mpts_1.push_back(objectKeypoints.at(i).pt);
					indexes_1.push_back(i);

					mpts_2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
					indexes_2.push_back(results.at<int>(i,0));
				}
			}
		}
		else
		{
			for(unsigned int i=0; i<matches.size(); ++i)
			{
				// Apply NNDR
				//printf("q=%d dist1=%f dist2=%f\n", matches.at(i).at(0).queryIdx, matches.at(i).at(0).distance, matches.at(i).at(1).distance);
				if(matches.at(i).size() == 2 &&
				   matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
				{
					mpts_1.push_back(objectKeypoints.at(matches.at(i).at(0).queryIdx).pt);
					indexes_1.push_back(matches.at(i).at(0).queryIdx);

					mpts_2.push_back(sceneKeypoints.at(matches.at(i).at(0).trainIdx).pt);
					indexes_2.push_back(matches.at(i).at(0).trainIdx);
				}
			}
		}

		// FIND HOMOGRAPHY
		unsigned int minInliers = 8;
		if(mpts_1.size() >= minInliers)
		{
			time.start();
			cv::Mat H = findHomography(mpts_1,
					mpts_2,
					cv::RANSAC,
					1.0,
					outlier_mask);
			printf("Time finding homography = %d ms\n", time.restart());
			int inliers=0, outliers=0;
			for(unsigned int k=0; k<mpts_1.size();++k)
			{
				if(outlier_mask.at(k))
				{
					++inliers;
				}
				else
				{
					++outliers;
				}
			}
			QTransform hTransform(
			H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
			H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
			H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));

			// GUI : Change color and add homography rectangle
			QColor color(Qt::green);
			int alpha = 130;
			color.setAlpha(alpha);
			for(unsigned int k=0; k<mpts_1.size();++k)
			{
				if(outlier_mask.at(k))
				{
//					objWidget.setKptColor(indexes_1.at(k), color);
//					sceneWidget.setKptColor(indexes_2.at(k), color);
				}
				else
				{
//					objWidget.setKptColor(indexes_1.at(k), QColor(255,0,0,alpha));
//					sceneWidget.setKptColor(indexes_2.at(k), QColor(255,0,0,alpha));
				}
			}
			QPen rectPen(color);
			rectPen.setWidth(4);
//			QGraphicsRectItem * rectItem = new QGraphicsRectItem(objWidget.pixmap().rect());
//			rectItem->setPen(rectPen);
//			rectItem->setTransform(hTransform);
//			sceneWidget.addRect(rectItem);
			printf("Inliers=%d Outliers=%d\n", inliers, outliers);
		}
		else
		{
			printf("Not enough matches (%d) for homography...\n", (int)mpts_1.size());
		}

		// Wait for gui
		// objWidget.setGraphicsViewMode(false);
		// objWidget.setWindowTitle("Object");
		// if(objWidget.pixmap().width() <= 800)
		// {
		// 	objWidget.setMinimumSize(objWidget.pixmap().width(), objWidget.pixmap().height());
		// }
		// else
		// {
		// 	objWidget.setMinimumSize(800, 600);
		// 	objWidget.setAutoScale(false);
		// }

		// sceneWidget.setGraphicsViewMode(false);
		// sceneWidget.setWindowTitle("Scene");
		// if(sceneWidget.pixmap().width() <= 800)
		// {
		// 	sceneWidget.setMinimumSize(sceneWidget.pixmap().width(), sceneWidget.pixmap().height());
		// }
		// else
		// {
		// 	sceneWidget.setMinimumSize(800, 600);
		// 	sceneWidget.setAutoScale(false);
		// }

		// sceneWidget.show();
		// objWidget.show();

		int r = app.exec();
		printf("Closing...\n");

		return r;

    
	}
	else
	{
		printf("Images are not valid!\n");
		showUsage();
	}

	return 1;



  ros::spin();
}

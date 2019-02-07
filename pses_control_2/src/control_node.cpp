#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <dynamic_reconfigure/server.h>
#include <pses_control_2/TutorialsConfig.h>
#include "opencv2/opencv.hpp"

#define stopSign 99999999

static int control_deviation = 0;
pses_control_2::TutorialsConfig newconfig;

void callback(pses_control_2::TutorialsConfig &config) 
{
    newconfig = config;
    ROS_INFO("Parameter are reconfigured:\n P:%d\n I:%d\n D:%d\n H_min:%d\n H_max:%d\n S_min:%d\n S_max:%d\n V_min:%d\n V_max:%d\n turn:%d\n", 
    config.P_int_param, 
    config.I_int_param, 
    config.D_int_param, 
    config.H_min_int_param,
    config.H_max_int_param,
    config.S_min_int_param,
    config.S_max_int_param,
    config.V_min_int_param,
    config.V_max_int_param,
    config.turn);
}

class deviation
{
	public:
		int P = 270;
		int I = 0;
		int D = 3;
		int distance = 0;
		int distanceMem[25] = { 0 };
		int distanceSum = 0;

		void pushIntoDistanceMem(int element)
		{
			for (int i = 39; i > 0; i--)
			{
				distanceMem[i] = distanceMem[i - 1];
			}
			distanceMem[0] = element;
		}
		void getDistanceMemSum()
		{
			for (int i = 0; i < 25; i++)
			{
				distanceSum += distanceMem[i];
			}
		}
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  static int imagecounter = 0; 
  imagecounter++;
  try
  {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat HSVImage;
    cv::Mat HSVImage2;
    cvtColor(image,HSVImage,CV_BGR2HSV);
    cvtColor(image,HSVImage2,CV_BGR2HSV);

    // filter green and pink
    cv::Mat ThreshImage;
    cv::Mat ThreshImage2;
    cv::Mat FiltedImage;
    cv::Mat FiltedImage2;
    int H_min = newconfig.H_min_int_param;
    int H_max = newconfig.H_max_int_param;
    int S_min = newconfig.S_min_int_param;
    int S_max = newconfig.S_max_int_param;
    int V_min = newconfig.V_min_int_param;
    int V_max = newconfig.V_max_int_param;  
    inRange(HSVImage,cv::Scalar(H_min,S_min,V_min),cv::Scalar(H_max,S_max,V_max),ThreshImage);
    inRange(HSVImage2,cv::Scalar(135,S_min,V_min),cv::Scalar(175,S_max,V_max),ThreshImage2);
    medianBlur(ThreshImage, FiltedImage, 7);
    medianBlur(ThreshImage2, FiltedImage2, 7);
    // cv::imshow("view", image);
    // cv::imshow("view2", FiltedImage);
    // if(imagecounter == 30)
    // {
    //   cv::imshow("view", FiltedImage);
    //   cv::imshow("view2", FiltedImage2);
    //   imagecounter = 0;
    // }
    //for greea line
    //get submatrix
    int subMatrix[20][100];
		double colpick = FiltedImage.cols / 100.0;
		double rowpick = FiltedImage.rows / 60.0;
		for (int c = 0; c < 100; c++)
		{
      //take 1/3 lower part of the FiltedImage and store it in the submatirx
			for (int r = 0; r <= 19; r++)
			{
				int col = c * colpick;
				int row = (r+40) * rowpick;
        uchar pixel = FiltedImage.at<uchar>(row, col);
				if (pixel > 100)
				{
					subMatrix[r][c] = 1;
				}
				else
				{
					subMatrix[r][c] = 0;
				}
			}
		}
    int standarLineRight[20] = {61,61,61,62,62,62,63,63,63,63,64,64,64,65,65,65,66,66,66,66};
    int standarLineleft[20] = {38,38,38,37,37,37,36,36,36,36,35,35,35,34,34,34,33,33,33,33};

    //for pink line
     int pinkLine[30] = {0};
    double colpick2 = FiltedImage2.cols / 100.0;
		double rowpick2 = FiltedImage2.rows / 60.0;
    for(int r = 0; r < 30; r++)
    {
      for(int c = 0; c < 100; c ++)
      {
        int row2 = (r+30) * rowpick2;
        int col2 = c * colpick2;
        uchar pixel2 = FiltedImage2.at<uchar>(row2, col2);
        if(pixel2 > 100)
        {
          pinkLine[r] = c;
        }     
      }
    }


    //with y = a*x^2 + b -----tmp1 = b  tmp2 = a
    // double tmp1 = 0;
    // double tmp2 = 0;
    // int tmp2cnt = 0;
    // for(int i = 29; i >= 0; i--)
    // {
    //   if(pinkLine[i] != 0)
    //   {
    //     tmp1 = pinkLine[i];
    //     break;
    //   }
    // }
    // for(int i = 28; i >= 0; i--)
    // {
    //   if(pinkLine[i] != 0)
    //   {
    //     tmp2 = tmp2 + (pinkLine[i] - tmp1)/((29-i)*(29-i));
    //     tmp2cnt++;
    //   }
    // }
    // tmp2 = tmp2 / tmp2cnt;
    // for(int i = 29; i >= 0; i--)
    // {
    //   pinkLine[i] = (int)(tmp1 + tmp2*(29-i)*(29-i));
    //   if(pinkLine[i] < 0)
    //   {
    //     pinkLine[i] = 0;
    //   }
    //   if(pinkLine[i] > 99)
    //   {
    //     pinkLine[i] = 99;
    //   }
    // }

    // int subMatrix2[30][100] = {0};
    // for(int r = 0; r < 30; r++)
    // {
    //   subMatrix2[r][pinkLine[r]]  = 1;
    // }

    //with y = a*x + b
    double tmp1 = 0;
    double tmp2 = 0;
    int tmp2cnt = 0;
    for(int i = 29; i >= 0; i--)
    {
      if(pinkLine[i] != 0)
      {
        tmp1 = pinkLine[i];
        break;
      }
    }
    for(int i = 28; i >= 0; i--)
    {
      if(pinkLine[i] != 0)
      {
        tmp2 = tmp2 + (pinkLine[i] - tmp1)/((29-i));
        tmp2cnt++;
      }
    }
    tmp2 = tmp2 / tmp2cnt;
    for(int i = 29; i >= 0; i--)
    {
      pinkLine[i] = (int)(tmp1 + tmp2*(29-i));
      if(pinkLine[i] < 0)
      {
        pinkLine[i] = 0;
      }
      if(pinkLine[i] > 99)
      {
        pinkLine[i] = 99;
      }
    }

    int subMatrix2[30][100] = {0};
    for(int r = 0; r < 30; r++)
    {
      subMatrix2[r][pinkLine[r]]  = 1;
    }


    int turn = newconfig.turn;
    //get the deviation(Abweichung)
		deviation devi;
    //stop flag;
    int noVision = 0;
    devi.distance = 0;
    //tur0 == 1 means it goes alone right side green line, -1 pink line
    if(turn == 1)
    {
      ROS_INFO("green green green");
      int lastValidSearch = 0;
      for(int y = 19; y >= 0; y--)
      {
        for(int search = 0; search < 33; search++)
        {
          int startPonit = standarLineRight[y];
          int toLeft = startPonit - search;
          int toRight = startPonit + search;
          if(subMatrix[y][toLeft] == 1)
          {
            lastValidSearch = -search;
            devi.distance = devi.distance - search;
            break;
          }
          if(subMatrix[y][toRight] == 1)
          {
            lastValidSearch = +search;
            devi.distance = devi.distance + search;
            break;
          }
          //if not find, use last valid search
          if(search == 32)
          {
            noVision++;
            devi.distance = devi.distance + lastValidSearch;
          }
        }
      }
    }

    if(turn == -1)
    {
      ROS_INFO("pink pink pink");
      int lastValidSearch = 0;
      for(int y = 19; y >= 0; y--)
      {
        for(int search = 0; search < 30; search++)
        {
          int startPonit = standarLineRight[y];
          int toLeft = startPonit - search;
          int toRight = startPonit + search;
          if(subMatrix2[y + 10][toLeft] == 1)
          {
            lastValidSearch = -search;
            devi.distance = devi.distance - search;
            break;
          }
          if(subMatrix2[y + 10][toRight] == 1)
          {
            lastValidSearch = +search;
            devi.distance = devi.distance + search;
            break;
          }
          //if not find, use last valid search
          if(search == 29)
          {
            noVision++;
            devi.distance = devi.distance + lastValidSearch;
          }
        }
      }
    }

    
		devi.pushIntoDistanceMem(devi.distance);
		devi.getDistanceMemSum();
    devi.P = newconfig.P_int_param;
    devi.I = newconfig.I_int_param;
    devi.D = newconfig.D_int_param;
    //we actually only used PD Regler
		control_deviation = (devi.P * devi.distanceMem[0] - devi.D * (devi.distanceMem[0] - devi.distanceMem[3]) + devi.I * devi.distanceSum/100)/100;
    //a flag, works when no vision, to stop the car
    if(noVision == 20)
    {
      	control_deviation = stopSign;
    }
    cv::waitKey(10);
    ROS_INFO("Control deviation set! deviation = %d", control_deviation);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void imageCallbackTest(const sensor_msgs::ImageConstPtr& msg) {

}

int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "control_node");
  // get ros node handle
  ros::NodeHandle nh;

  //reconfig
  dynamic_reconfigure::Server<pses_control_2::TutorialsConfig> server;
  dynamic_reconfigure::Server<pses_control_2::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1);
  server.setCallback(f);

  // sensor message container
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages
  // ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
  //     "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1, &control_deviation, &newconfig));
  ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, imageCallback, ros::TransportHints().tcpNoDelay());

  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  // cv::namedWindow("view");
  // cv::namedWindow("view2");
  cv::startWindowThread();
  //change the exposure
  cv::VideoCapture cap;
  cap.set(cv::CAP_PROP_EXPOSURE,10);

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(25);

  //control variable
  int lastValidDeviation;
  //lastside can be replaced later by a flag, which can be set because of other event
  int lastside = newconfig.turn;
  int laneChangeCnt = 30;
  while (ros::ok())
  {
    steering.data = control_deviation;
    motor.data = 300;

    if(steering.data >= 900)
    {
      steering.data = 900;
    }
      if(steering.data <= -900)
    {
      steering.data = -900;
    }

    if(lastside !=  newconfig.turn)
    {
      steering.data = 900 * newconfig.turn;
      if(laneChangeCnt < 10)
      {
        steering.data = 900 * newconfig.turn * (-1);
      }
      motor.data = 300;
      laneChangeCnt--;
      //finish lane change and reset the procedure
      if(laneChangeCnt == 0)
      {
        lastside = newconfig.turn;
        laneChangeCnt = 30;
      }
    }

    // //stop the car because no vision
    if((control_deviation == stopSign) && (lastside == newconfig.turn))
    {
      motor.data = 0;
      motorCtrl.publish(motor);
    }
    else
    {
      lastValidDeviation = control_deviation;
    }
    // publish command messages on their topics
    ROS_INFO("steering %d     motor %d", steering.data, motor.data);
    steeringCtrl.publish(steering);
    motorCtrl.publish(motor);
    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}

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

void callback(pses_control_2::TutorialsConfig &config, uint32_t level, pses_control_2::TutorialsConfig* newconfig) 
{
    *newconfig = config;
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
		int P = 413;
		int I = 0;
		int D = 3;
		int distance = 0;
		int distanceMem[40] = { 0 };
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
			for (int i = 0; i < 40; i++)
			{
				distanceSum += distanceMem[i];
			}
		}
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg, int* control_deviation, pses_control_2::TutorialsConfig* newconfig)
{
  static int imagecounter = 0; 
  imagecounter++;
  try
  {
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat HSVImage;

    cvtColor(image,HSVImage,CV_BGR2HSV);
    //cv::imshow("row", HSVImage);
    

    // filter green
    cv::Mat ThreshImage;
    cv::Mat FiltedImage;
    int H_min = newconfig->H_min_int_param;
    int H_max = newconfig->H_max_int_param;
    int S_min = newconfig->S_min_int_param;
    int S_max = newconfig->S_max_int_param;
    int V_min = newconfig->V_min_int_param;
    int V_max = newconfig->V_max_int_param;  
    inRange(HSVImage,cv::Scalar(H_min,S_min,V_min),cv::Scalar(H_max,S_max,V_max),ThreshImage);
    medianBlur(ThreshImage, FiltedImage, 7);
    if(imagecounter == 30)
    {
      cv::imshow("view", FiltedImage);
      imagecounter = 0;
    }
    //cv::waitKey(30);

    //get submatrix
    int subMatrix[20][100];
		double colpick = FiltedImage.cols / 100.0;
		double rowpick = FiltedImage.rows / 60.0;
		for (int c = 0; c < 100; c++)
		{
      //take the lower half
			for (int r = 19; r >= 0; r--)
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
    int turn = 1;
    turn = newconfig->turn;
    //get the deviation(Abweichung)
		deviation devi;
    //stop flag;
    int noVision = 0;

    devi.distance = 0;
    if(turn == 1)
    {
      int lastValidSearch = 0;
      for(int y = 19; y >= 0; y--)
      {
        for(int search = 0; search < 30; search++)
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
          if(search == 29)
          {
            noVision++;
            devi.distance = devi.distance + lastValidSearch;
          }
        }
      }
    }

    if(turn == -1)
    {
      int lastValidSearch = 0;
      for(int y = 19; y >= 0; y--)
      {
        for(int search = 0; search < 30; search++)
        {
          int startPonit = standarLineleft[y];
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
    devi.P = newconfig->P_int_param;
    devi.I = newconfig->I_int_param;
    devi.D = newconfig->D_int_param;
		*control_deviation = (devi.P * devi.distanceMem[0] + devi.D * (devi.distanceMem[0] - devi.distanceMem[2]) + devi.I * devi.distanceSum/100)/100;
    //a flag, works when no vision, to stop the car
    if(noVision == 20)
    {
      	*control_deviation = stopSign;
    }
    cv::waitKey(1);
    ROS_INFO("Control deviation set! deviation = %d", *control_deviation);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "control_node");
  // get ros node handle
  ros::NodeHandle nh;

  //reconfig
  pses_control_2::TutorialsConfig newconfig;
  dynamic_reconfigure::Server<pses_control_2::TutorialsConfig> server;
  dynamic_reconfigure::Server<pses_control_2::TutorialsConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2, &newconfig);
  server.setCallback(f);

  // sensor message container
  int control_deviation = 0;
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages
  ros::Subscriber imageSub = nh.subscribe<sensor_msgs::Image>(
      "kinect2/qhd/image_color", 1, boost::bind(imageCallback, _1, &control_deviation, &newconfig));

  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  cv::namedWindow("view");
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
  int lastside = 1;
  int laneChangeCnt = 50;
  while (ros::ok())
  {
    //stop the car because no vision
    if(control_deviation == stopSign)
    {
      motor.data = 0;
      motorCtrl.publish(motor);
    }
    else
    {
      lastValidDeviation = control_deviation;
    }
    steering.data = control_deviation;

    if(steering.data >= 1000)
    {
      steering.data = 1000;
    }
      if(steering.data <= -1000)
    {
      steering.data = -1000;
    }
    ROS_INFO("steering %d",steering.data);
    if((lastside !=  newconfig.turn) && (control_deviation != stopSign))
    {
      steering.data = 1000 * newconfig.turn * (-1);
      if(laneChangeCnt < 25)
      {
        steering.data = 1000 * newconfig.turn;
      }
      motor.data = 500;
      laneChangeCnt--;
      //finish lane change and reset the procedure
      if(laneChangeCnt == 0)
      {
        lastside = newconfig.turn;
        laneChangeCnt = 50;
      }
    }
    // publish command messages on their topics
    steeringCtrl.publish(steering);
    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}

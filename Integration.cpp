//made by C
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <cmath>

//all the **** needs to be changed

// gets called whenever a new message is availible in the input puffer
void matrixCallback()
{
	//****
}

//preprocess the matrix,so that if can be fit in a 50*200 matrix for the algorithm
typedef int matrix_msgs;

//this class is
class deviation
{
	public:
		int P = 0;
		int I = 0;
		int D = 0;
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
				distanceSum += devi.distanceMem[i];
			}
		}
};


int main(int argc, char** argv)
{
	// init this node
	ros::init(argc, argv, "integration");
	// get ros node handle
	ros::NodeHandle nh;

	// matrix message container
	matrix_msgs matrix[50][200];
	matrix_msgs s[50][200];

	//output message container
	std_msgs::Int16 motor, steering;

	// generate subscriber for sensor messages
	ros::Subscriber matrixSub = nh.subscribe<matrix_msgs>(
		"/matrix/matrix", 10, boost::bind(matrixCallback, _1, &matrix));

	// generate control message publisher
	ros::Publisher motorCtrl =
		nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
	ros::Publisher steeringCtrl =
		nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

	ROS_INFO("Let's start integration");

	// Loop starts here:
	// loop rate value is set in Hz

	ros::Rate loop_rate(20);
	while (ros::ok())
	{	
		//get the deviation(Abweichung)

		deviation devi;
		int tmp1;
		int tmp2;
		for (int y = 0; y < 50; y++)
		{
			devi.distance = 0;
			tmp1 = -1;
			tmp2 = -1;
			for (int x = 0; x < 200; x++)
			{
				if (matrix[y][x] == 1)
					tmp1 = x;
				if (s[y][x] == 1)
					tmp2 = x;
				if (tmp1 != -1 && tmp2 != -1)
				{
					devi.distance = devi.distance + abs(tmp1 - tmp2);
					break;
				}
			}
		}

		devi.pushIntoDistanceMem(devi.distance);
		devi.getDistanceMemSum();
		steering.data = devi.P * devi.distanceMem[0] + devi.D * (devi.distanceMem[0] - devi.distanceMem[2]) + devi.I * devi.distanceSum;
		



		// publish command messages on their topics
		motorCtrl.publish(motor);
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

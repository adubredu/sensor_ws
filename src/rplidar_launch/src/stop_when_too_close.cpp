#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_msgs/Bool.h>

ros::Publisher stop_pub;

bool too_close = false;

void scanHandler(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	too_close = false;
	for (const float r : scan_in->ranges)
	{
		if (r>= scan_in->range_min && r < 0.5)
		{
			too_close = true;
			break;
		}
	}
}

void velocityCommandsCallback(const geometry_msgs::Twist::ConstPtr& velocity)
{
	geometry_msgs::Twist speed;
	speed.linear.x = velocity->linear.x;
	speed.angular.z = velocity->angular.z;

	if(too_close and (velocity->linear.x > 0 or velocity->angular.z != 0))
		speed.linear.x = 0;

	stop_pub.publish(speed);
    
	
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "stop_when_close");
  	ros::NodeHandle nh;
  	stop_pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel_", 5);
	ros::Subscriber subspeed = nh.subscribe<geometry_msgs::Twist>
		                ("/cmd_vel", 5, velocityCommandsCallback);
	ros::Subscriber subscan = nh.subscribe<sensor_msgs::LaserScan>
		                ("/scan_filtered", 5, scanHandler);;
	ros::spin();

	return 0;
}

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

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>());
ros::Publisher laserpub;
ros::Publisher stop_pub;

double x_offset = 0.345;
double y_offset = 0.0;
double z_offset = -0.63;

bool too_close = false;

double vehicleX = 0.0;
double vehicleY = 0.0;
double vehicleZ = 0.0;
double odomTime = 0.0;
double vehicleYaw = 0.0;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	double roll, pitch, yaw;
	geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
	

	vehicleX = odom->pose.pose.position.x;
  	vehicleY = odom->pose.pose.position.y;
  	vehicleZ = odom->pose.pose.position.z;

  	odomTime = odom->header.stamp.toSec();
}


void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
	
	laserCloud->clear();
	pcl::fromROSMsg(*laserCloud2, *laserCloud);
	pcl::PointXYZI point;
	int size = laserCloud->points.size();

	too_close = false;
	for (int i=0; i<size; i++)
	{
		point = laserCloud->points[i];

		double dist = sqrt(pow((vehicleX-point.x),2) + pow((vehicleY-point.y),2));
		if (dist < 0.5)
		{
			too_close = true;
			break;
		}

	}
}


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

void velocityCommandsCallback(const geometry_msgs::TwistStamped::ConstPtr& velocity)
{
	geometry_msgs::TwistStamped speed;
	speed.header.stamp = velocity->header.stamp;
	speed.header.seq = velocity->header.seq;
	speed.header.frame_id = velocity->header.frame_id;
	speed.twist.linear.x = velocity->twist.linear.x;
	speed.twist.angular.z = velocity->twist.angular.z;

	if(too_close and velocity->twist.linear.x > 0)
		speed.twist.linear.x = 0;

	stop_pub.publish(speed);
    
	
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "stop_when_close");
  	ros::NodeHandle nh;
  	stop_pub = nh.advertise<geometry_msgs::TwistStamped> ("/cmd_vel_", 5);
  	ros::Subscriber subodom = nh.subscribe<nav_msgs::Odometry>
  								("/integrated_to_map",1, odometryCallback);
  	// ros::Subscriber subscan = nh.subscribe<sensor_msgs::PointCloud2>
                                // ("/added_obstacles", 5, cloudHandler);
    ros::Subscriber subspeed = nh.subscribe<geometry_msgs::TwistStamped>
                                ("/cmd_vel", 5, velocityCommandsCallback);
    ros::Subscriber subscan = nh.subscribe<sensor_msgs::LaserScan>
                                ("/scan_filtered", 5, scanHandler);;
    ros::spin();

	return 0;
}
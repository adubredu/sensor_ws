#include <ros/ros.h>
#include <math.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

laser_geometry::LaserProjection projector;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>());
ros::Publisher laserpub;
ros::Publisher stop_pub;

double x_offset = 0.18;
double y_offset = 0.0;
double z_offset = -0.11;

double robotX = 0.0, robotY = 0.0, robotZ = 0.0;
double odomTime = 0.0;
double robotYaw = 0.0;

bool waypoint_forward = true;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	double roll, pitch, yaw;
	geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
	tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

	robotYaw = yaw;

	robotX = odom->pose.pose.position.x;
  	robotY = odom->pose.pose.position.y;
  	robotZ = odom->pose.pose.position.z;

  	odomTime = odom->header.stamp.toSec();
}




void scanHandler(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	sensor_msgs::PointCloud2 cloud;
	projector.projectLaser(*scan_in, cloud);	

	laserCloud->clear();
	transformedCloud->clear();
	pcl::fromROSMsg(cloud, *laserCloud);
	pcl::PointXYZI point;
	int size = laserCloud->points.size();
	double cosYaw = cos(robotYaw);
	double sinYaw = sin(robotYaw);

	for (int i=0; i<size; i++)
	{
		double pointX = laserCloud->points[i].x + x_offset;
		double pointY = laserCloud->points[i].y + y_offset;
		double pointZ = laserCloud->points[i].z + z_offset;

		double x = pointX * cosYaw - pointY * sinYaw;
		double y = pointX * sinYaw + pointY * cosYaw;
		point.x = x + robotX;
		point.y = y + robotY;
		point.z = pointZ;

		transformedCloud->push_back(point); 
	}

	sensor_msgs::PointCloud2 cloud_out;
	pcl::toROSMsg(*transformedCloud, cloud_out);
	cloud_out.header.stamp = ros::Time().fromSec(odomTime);
	cloud_out.header.frame_id = "/map";
	laserpub.publish(cloud_out);

}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "scan_to_pcl");
  	ros::NodeHandle nh;
  	laserpub = nh.advertise<sensor_msgs::PointCloud2> ("/added_obstacles", 5);
  	ros::Subscriber subodom = nh.subscribe<nav_msgs::Odometry>
  				("/state_estimation",1, odometryCallback);
  	ros::Subscriber subscan = nh.subscribe<sensor_msgs::LaserScan>
                                ("/scan_filtered", 5, scanHandler);
    	ros::spin();

	return 0;
}

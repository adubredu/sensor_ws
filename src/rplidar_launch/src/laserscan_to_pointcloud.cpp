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

double x_offset = 0.345;
double y_offset = 0.0;
double z_offset = -0.63;

double vehicleX = 0.0;
double vehicleY = 0.0;
double vehicleZ = 0.0;
double odomTime = 0.0;
double vehicleYaw = 0.0;

bool waypoint_forward = true;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
	double roll, pitch, yaw;
	geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
	tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

	vehicleYaw = yaw;

	vehicleX = odom->pose.pose.position.x;
  	vehicleY = odom->pose.pose.position.y;
  	vehicleZ = odom->pose.pose.position.z;

  	odomTime = odom->header.stamp.toSec();
}

void waypoint_direction(const geometry_msgs::PointStamped::ConstPtr& data)
{
	double pointX = data->point.x - vehicleX;
	double pointY = data->point.y - vehicleY;
	double x = pointX * cos(vehicleYaw) + pointY * sin(vehicleYaw);
	double y = -pointX * sin(vehicleYaw) + pointY * cos(vehicleYaw);

	double ang = atan2(x,y);
	if (ang > 0)
		waypoint_forward = true;
	else if (ang < 0)
		waypoint_forward = false;
}


void scanHandler(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	if (true) {
	sensor_msgs::PointCloud2 cloud;
	projector.projectLaser(*scan_in, cloud);	

	laserCloud->clear();
	transformedCloud->clear();
	pcl::fromROSMsg(cloud, *laserCloud);
	pcl::PointXYZI point;
	int size = laserCloud->points.size();

	for (int i=0; i<size; i++)
	{
		double pointX = laserCloud->points[i].x;
		double pointY = laserCloud->points[i].y;
		double pointZ = laserCloud->points[i].z;

	 	pointX += x_offset;
	 	pointY += y_offset;
	 	pointZ += z_offset;

		if ((sqrt(pow(pointX, 2) + pow(pointY, 2)) > 2.5) or (sqrt(pow(pointX, 2) + pow(pointY, 2)) < 0.7)) {
			continue;
		}

		double x = pointX * cos(vehicleYaw) - pointY * sin(vehicleYaw);
		double y = pointX * sin(vehicleYaw) + pointY * cos(vehicleYaw);
		point.x = x + vehicleX;
		point.y = y + vehicleY;
		point.z = pointZ;

		transformedCloud->push_back(point); 
	}

	sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*transformedCloud, cloud_out);
    cloud_out.header.stamp = ros::Time().fromSec(odomTime);
    cloud_out.header.frame_id = "/map";
	laserpub.publish(cloud_out);
}
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "scan_to_pcl");
  	ros::NodeHandle nh;
  	laserpub = nh.advertise<sensor_msgs::PointCloud2> ("/added_obstacles", 5);
  	ros::Subscriber subodom = nh.subscribe<nav_msgs::Odometry>
  								("/integrated_to_map",1, odometryCallback);
  	ros::Subscriber subscan = nh.subscribe<sensor_msgs::LaserScan>
                                ("/scan", 5, scanHandler);
    ros::Subscriber subwaypoint = nh.subscribe<geometry_msgs::PointStamped>
    							("/way_point",1, waypoint_direction);
    ros::spin();

	return 0;
}
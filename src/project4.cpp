#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <angles/angles.h>
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>



//using namespace boost::posix_time;

class project4
{
	public:

	ros::Publisher pub;

	project4(ros::Publisher paramPub)
	{
		pub = paramPub;
	}
	
	void translate(double d)
	{
		geometry_msgs::Twist msg;
		nav_msgs::OdometryConstPtr odomMsg = 
			ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");

		double x = -odomMsg->pose.pose.position.y;
		double y = odomMsg->pose.pose.position.x;
		//Set the movement command rotation speed
		msg.linear.x = 0.4;
		// Current angle
		double last_x = x;
		double last_y = y;
		double thisD = 0;
		
		while ((thisD < d) && ros::ok()) 
		{
		    nav_msgs::OdometryConstPtr odomMsg2 = 
				ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
			x = -odomMsg2->pose.pose.position.y;
			y = odomMsg2->pose.pose.position.x;

			//Publish the Twist message and sleep for a cycle
		    pub.publish(msg);

		    thisD = sqrt(pow((x - last_x), 2) + pow((y - last_y), 2));
		}

		//Robot stops turning
		msg.linear.x = 0;
		pub.publish(msg);
	}
	void rotate_rel(double angle)
	{
			geometry_msgs::Twist msg;
			nav_msgs::OdometryConstPtr odomMsg = 
				ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
			double theta = tf::getYaw(odomMsg->pose.pose.orientation);
			//Set the movement command rotation speed
			msg.angular.z = 0.6;
			// Current angle
			double last_angle = theta;
			double thisAngle = 0;

			while ((thisAngle < angle) && ros::ok()) 
			{
				nav_msgs::OdometryConstPtr odomMsg2 =
					ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
				theta = tf::getYaw(odomMsg2->pose.pose.orientation);

				//Publish the Twist message and sleep for a cycle
				pub.publish(msg);

				// Compute the amount of rotation since the last loop
				thisAngle += angles::normalize_angle(theta - last_angle);
		        
				last_angle = theta;

			}

			//Robot stops turning
			msg.angular.z = 0;
			pub.publish(msg);
	}
	


};



int main(int argc, char **argv)

{
	ROS_INFO_STREAM("Enters Main");
	ros::init(argc, argv, "project4");
	ros::NodeHandle nh;
	ros::Publisher pubMes = nh.advertise<geometry_msgs::Twist>
		("/mobile_base/commands/velocity", 100);
	project4 *proj = new project4(pubMes);
	proj->translate(1);
	proj->rotate_rel(M_PI/2);
	proj->translate(1);
	proj->rotate_rel(M_PI/2);
	proj->translate(1);
	proj->rotate_rel(M_PI/2);
	proj->translate(1);
	proj->rotate_rel(M_PI/2);
	ros::spin();
 return 0;
}










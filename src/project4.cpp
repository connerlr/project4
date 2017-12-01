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
#include "geometry_msgs/PoseWithCovarianceStamped.h"



//using namespace boost::posix_time;

class project4
{
	public:
	double odomX, odomY, comX, comY, odomHeading, comHeading;
	ros::Subscriber odomCom, odomSub;
	ros::Publisher pub;
	//ros::NodeHandle nh;

	project4(ros::NodeHandle nh)
	{
/**	
		pub = nh.advertise<geometry_msgs::Twist>
			("mobile_base/commands/velocity", 100);
		ros::Subscriber odomSub = nh.subscribe("odom", 
			10, &project4::odomCallback, this);
		ros::Subscriber odomCom = nh.subscribe("robot_pose_ekf/odom_combined", 
			10, &project4::odomCombinedCallBack, this);
*/

		//stage testing
		odomHeading = 0;
		pub = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 100);
		odomCom = nh.subscribe("robot_pose_ekf/odom_combined", 
			10, &project4::odomCombinedCallBack, this);
		odomSub = nh.subscribe("robot_0/odom", 
			10, &project4::odomCallBack, this);
		

	}
	
	void translate(double d)
	{
		geometry_msgs::Twist msg;


		//Set the movement command rotation speed
		msg.linear.x = 0.4;
		// Current angle
		double last_x = odomX;
		double last_y = odomY;
		double thisD = 0;
		ros::Rate rate(10.0);
		while ((thisD < d) && ros::ok()) 
		{

			//Publish the Twist message and sleep for a cycle
		    pub.publish(msg);

		    thisD = sqrt(pow((odomX - last_x), 2) + pow((odomY - last_y), 2));
			ros::spinOnce();
			rate.sleep();
		}

		//Robot stops turning
		msg.linear.x = 0;
		pub.publish(msg);
	}


	void rotate_rel(double angle)
	{
		geometry_msgs::Twist msg;
		//nav_msgs::OdometryConstPtr odomMsg = 
			//ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
		//double theta = tf::getYaw(odomMsg->pose.pose.orientation);
		//Set the movement command rotation speed
/**
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

*/

		double initHead = odomHeading;
		double desiredHead = initHead + angle;

		ROS_INFO("DESIRED HEAD %f" , desiredHead);
		msg.angular.z = 0.6;
		pub.publish(msg);
		ros::Rate rate(10);
		ROS_INFO("init head %f" , initHead);
		while(ros::ok())
		{
			pub.publish(msg);
			ROS_INFO("ODOM %f", odomHeading);
			if(odomHeading < desiredHead + 0.149 && odomHeading > desiredHead - 0.149)
			{
				msg.angular.z = 0.0;
				pub.publish(msg);
				break;
			}
			ros::spinOnce();
			rate.sleep();
		}
			
	}

	void rotate_abs(double angle)
	{
		geometry_msgs::Twist msg;
		//nav_msgs::OdometryConstPtr odomMsg = 
			//ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
		//double theta = tf::getYaw(odomMsg->pose.pose.orientation);
		//Set the movement command rotation speed
/**
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

*/

		//double initHead = odomHeading;
		//double desiredHead = initHead + angle;

		ROS_INFO("DESIRED HEAD %f" , angle);
		msg.angular.z = 0.6;
		pub.publish(msg);
		ros::Rate rate(10);
		while(ros::ok())
		{
			pub.publish(msg);
			ROS_INFO("ODOM %f", odomHeading);
			if(odomHeading < angle + 0.149 && odomHeading > angle - 0.149)
			{
				msg.angular.z = 0.0;
				pub.publish(msg);
				break;
			}
			ros::spinOnce();
			rate.sleep();
		}
	}


	void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
	{
		odomX = -msg->pose.pose.position.y;
		odomY = msg->pose.pose.position.x;
		odomHeading = tf::getYaw(msg->pose.pose.orientation);
			
	}


	void odomCombinedCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
	{
		comX = -msg->pose.pose.position.y;
		comY = msg->pose.pose.position.x;
		comHeading = tf::getYaw(msg->pose.pose.orientation);
	}
	


};



int main(int argc, char **argv)

{
	ROS_INFO_STREAM("Enters Main");
	ros::init(argc, argv, "project4");
	ros::NodeHandle nh;
	ROS_INFO_STREAM("Enters Main");
	project4 *proj = new project4(nh);
	ROS_INFO_STREAM("Enters Main");
	proj->rotate_rel(M_PI/2);

	proj->rotate_abs(0);

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










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
#include <math.h>



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
		odomHeading = 0;//huge number fix
		odomX = 0;
		odomY = 0;
		pub = nh.advertise<geometry_msgs::Twist>
			("mobile_base/commands/velocity", 100);
		odomSub = nh.subscribe("odom", 
			10, &project4::odomCallBack, this);
		odomCom = nh.subscribe("robot_pose_ekf/odom_combined", 
			10, &project4::odomCombinedCallBack, this);


		//stage testing


		//TODO initialize all
/**
		pub = nh.advertise<geometry_msgs::Twist>("robot_0/cmd_vel", 100);
		odomCom = nh.subscribe("robot_pose_ekf/odom_combined", 
			10, &project4::odomCombinedCallBack, this);
		odomSub = nh.subscribe("robot_0/odom", 
			10, &project4::odomCallBack, this);
	*/	

	}
	
	void translate(double d)
	{
		geometry_msgs::Twist msg;


		//Set the movement command rotation speed
		msg.linear.x = 0.4;
		msg.angular.z = 0.0;
		// Current angle
		double last_x;
		double last_y;
		int count = 0;
		double thisD = 0;
		ros::Rate rate(50.0);
		while (ros::ok()) 
		{
			if((odomX + odomY) != 0.0)
			{
				//ROS_INFO_STREAM(thisD);
				//ROS_INFO_STREAM("split this d (above) and d (below");
				//ROS_INFO_STREAM(d);
				if(count == 0)
				{
					count++;
					last_x = odomX;
					last_y = odomY;
				}
				if(thisD >= d)
				{
					
					msg.linear.x = 0.0;
					
					pub.publish(msg);
					break;
				}
			//	ROS_INFO("odomx %f", odomX);
				//ROS_INFO("odomY %f", odomY);
				msg.angular.z = 0.0;
				//Publish the Twist message and sleep for a cycle
				pub.publish(msg);
				//ROS_INFO_STREAM(msg);

				thisD = sqrt(pow((odomX - last_x), 2) + pow((odomY - last_y), 2));
			}	
			//ROS_INFO_STREAM("SHOULD BE DOING THINGS");		
			ros::spinOnce();
			rate.sleep();
		}

		//Robot stops moving
		msg.linear.x = 0.0;
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
		double temp = 0;
		double temp2 = 0;
		double temp3 = 0;
		if(desiredHead > M_PI || desiredHead < -M_PI)
		{
			desiredHead = remainder(desiredHead, 2.0*M_PI);

/**
			temp = 3.14159 - odomHeading;
			temp2 = angle - temp;
			temp3 = -3.14159 + temp2;
			desiredHead = temp3;


			desiredHead = desiredHead*(M_PI / 180);
			desiredHead = fmod(desiredHead + 180, 360);
			if(desiredHead < 0)
			{
				desiredHead += 360;
			}
			desiredHead = desiredHead * (180 / M_PI);
*/
		}

		//ROS_INFO("DESIRED HEAD %f" , desiredHead);
		msg.angular.z = 0.6;
		pub.publish(msg);
		ros::Rate rate(10);
		//ROS_INFO("init head %f" , initHead);
		while(ros::ok())
		{
	
			//ROS_INFO_STREAM("turning");
			pub.publish(msg);
			//ROS_INFO("ODOM %f", odomHeading);
			//ROS_INFO("desired %f", desiredHead);
			if(odomHeading > (desiredHead - 0.09) &&
				 odomHeading < (desiredHead + 0.09))
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
			if(odomHeading < angle + 0.25 && odomHeading > angle - 0.25)
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
		//odomX = 0;
		//odomY = 0;
		//if(std::abs(-msg->pose.pose.position.y - odomX) < 0.5)
	//	{
			odomX = -msg->pose.pose.position.y;
		//}
		//if(std::abs(msg->pose.pose.position.x - odomY) < 0.5)
		odomY = msg->pose.pose.position.x;
		odomHeading = tf::getYaw(msg->pose.pose.orientation);
			
	}


	void odomCombinedCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
	{
		comX = -msg->pose.pose.position.y;
		comY = msg->pose.pose.position.x;
		comHeading = tf::getYaw(msg->pose.pose.orientation);
	}
	
	void pauseRobot()
	{
		ros::Time begin = ros::Time::now();
		ros::Duration five_seconds(5, 0);
		ros::Rate rate(10);
		while(ros::Time::now() < begin + five_seconds)
		{
			ros::spinOnce();
			rate.sleep();
		}
	}
	


};



int main(int argc, char **argv)

{
	ROS_INFO_STREAM("Enters Main");
	ros::init(argc, argv, "project4");
	ros::NodeHandle nh;
	project4 *proj = new project4(nh);
//	proj->rotate_rel(M_PI/2);

	//proj->rotate_abs(0);
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;

	ROS_INFO_STREAM("translate 1");
	proj->translate(1);////////////////////////
proj->pauseRobot();
	ROS_INFO_STREAM("TURN 1");
	proj->rotate_rel(M_PI/2);/////////////////////
proj->pauseRobot();
	//proj->rotate_abs(M_PI/2);
	ROS_INFO_STREAM("translate 2");
	proj->translate(1);////////////////
proj->pauseRobot();

	ROS_INFO_STREAM("TURN 2");
	proj->rotate_rel(M_PI/2);/////////////////////
proj->pauseRobot();
	//proj->rotate_abs(M_PI);

	ROS_INFO_STREAM("translate 3");
	proj->translate(1);//////////////////
proj->pauseRobot();

	ROS_INFO_STREAM("TURN 3");
	proj->rotate_rel(M_PI/2);///////////////////////
proj->pauseRobot();
	//proj->rotate_abs(-M_PI);

	ROS_INFO_STREAM("translate 4");
	proj->translate(1);///////////////////
proj->pauseRobot();
	
	ROS_INFO_STREAM("TURN 4");
	proj->rotate_rel(M_PI/2);/////////////////
	//proj->rotate_abs(0);

	ros::spin();

 return 0;
}










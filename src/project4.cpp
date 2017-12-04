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
//#include "geometry_msgs/Point.h"
#include <math.h>



//using namespace boost::posix_time;

class project4
{
	public:
	double odomX, odomY, odomHeading;
	double comX, comY, comHeading;
	ros::Subscriber odomCom, odomSub;
	ros::Publisher pub;
	//ros::NodeHandle nh;
	
	project4(ros::NodeHandle nh)
	{
		odomHeading = 0;//huge number fix
		comHeading = 0;
		odomX = 0;
		odomY = 0;
		comX = 0;
		comY = 0;
		pub = nh.advertise<geometry_msgs::Twist>
			("mobile_base/commands/velocity", 100);
		odomSub = nh.subscribe("odom", 
			10, &project4::odomCallBack, this);
		odomCom = nh.subscribe("robot_pose_ekf/odom_combined", 
			10, &project4::odomCombinedCallBack, this);

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
			//ROS_INFO("odomX %f", odomX);
		//	ROS_INFO("comX %f", comX);
		//	ROS_INFO("odomY%f", odomY);
		//	ROS_INFO("comY %f", comX);
			if((odomX + odomY) != 0.0)
			{
				if(count == 0)
				{		
					ROS_INFO_STREAM("UNDER THIS IS BEFORE WHILE LOOP");
					ROS_INFO("odomX %f", odomX);
					ROS_INFO("comX %f", comX);
					ROS_INFO("odomY%f", odomY);
					ROS_INFO("comY %f", comY);
					ROS_INFO_STREAM("ABOVE THIS IS BEFORE WHILE LOOP");
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
				else if (d - thisD < 0.4)
				{
					msg.linear.x = 0.2;
				}
				else if (d - thisD < 0.15)
				{
					msg.linear.x = 0.1;
				}
				else
				{
					msg.linear.x = 0.4;
				}

				msg.angular.z = 0.0;

				pub.publish(msg);

				thisD = sqrt(pow((odomX - last_x), 2) + pow((odomY - last_y), 2));
			}	
		ros::spinOnce();
		rate.sleep();
		}
		ROS_INFO_STREAM("UNDER THIS IS AFTER WHILE LOOP");
		ROS_INFO("odomX %f", odomX);
		ROS_INFO("comX %f", comX);
		ROS_INFO("odomY%f", odomY);
		ROS_INFO("comY %f", comY);
		ROS_INFO_STREAM("ABOVE THIS IS AFTER WHILE LOOP");

		//Robot stops moving
		msg.linear.x = 0.0;
		pub.publish(msg);
	}


	void rotate_rel(double angle)
	{
		
		geometry_msgs::Twist msg;
		double initHead = odomHeading;
		double desiredHead = initHead + angle;
		double temp = 0;
		double temp2 = 0;
		double temp3 = 0;
		if(desiredHead > M_PI || desiredHead < -M_PI)
		{
			desiredHead = remainder(desiredHead, 2.0*M_PI);
		}

		msg.angular.z = 0.6;
		//ROS_INFO_STREAM(msg);
		pub.publish(msg);
		ros::Rate rate(10);
		ROS_INFO("ODOM %f", odomHeading);
		ROS_INFO("comH %f", comHeading);
		while(ros::ok())
		{
			pub.publish(msg);
			//ROS_INFO("ODOM %f", odomHeading);
			//ROS_INFO("comH %f", comHeading);
			if(odomHeading > (desiredHead - 0.14) &&
				 odomHeading < (desiredHead + 0.14))
			{
				msg.angular.z = 0.0;
				pub.publish(msg);
				break;
			}
			ros::spinOnce();
			rate.sleep();
		}
			ROS_INFO("ODOM %f", odomHeading);
			ROS_INFO("comH %f", comHeading);
			
	}

	void rotate_abs(double angle)
	{
		geometry_msgs::Twist msg;
		ROS_INFO("DESIRED HEAD %f" , angle);
		if(odomHeading < 0 && angle >=0 && 
			   (-odomHeading + angle) > ((M_PI + odomHeading) + (M_PI - angle)))
		{
			ROS_INFO_STREAM("faster to turn right");
			msg.angular.z = -0.6;
		}
		else if(odomHeading < 0 && angle >=0 && 
			   (-odomHeading + angle) < ((M_PI + odomHeading) + (M_PI - angle)))
		{
			ROS_INFO_STREAM("faster to turn left");
			msg.angular.z = 0.6;
		}
		else if(odomHeading >= 0 && angle < 0 &&
			   ((M_PI - odomHeading) + (M_PI - angle)) < (odomHeading + (-M_PI + angle)))
		{
			msg.angular.z = -0.6;
			ROS_INFO_STREAM("faster to turn right complex");
		}
		else if(odomHeading >= 0 && angle < 0 &&
			   ((M_PI - odomHeading) + (M_PI - angle)) > (odomHeading + (-M_PI + angle)))
		{
			msg.angular.z = 0.6;
			ROS_INFO_STREAM("faster to turn left complex hopefully");
		}
		else if(odomHeading >= 0 && angle >= 0 && (angle > odomHeading))
		{
			msg.angular.z = 0.6;
		}
		else if(odomHeading >= 0 && angle >= 0 && (angle < odomHeading))
		{
			msg.angular.z = -0.6;
		}
		else if(odomHeading < 0 && angle < 0 && (angle > odomHeading))
		{
			msg.angular.z = 0.6;
		}
		else if(odomHeading < 0 && angle < 0 && (angle < odomHeading))
		{
			msg.angular.z = -0.6;
		}
		else
		{
			msg.angular.z = 0.6;
			ROS_INFO_STREAM("AM CONFUSED");
		}
		pub.publish(msg);
		ros::Rate rate(10);
		ROS_INFO("ODOM %f", odomHeading);
		ROS_INFO("comH %f", comHeading);
		while(ros::ok())
		{
			//ROS_INFO_STREAM(odomHeading);
			pub.publish(msg);
			if(odomHeading < angle + 0.14 && odomHeading > angle - 0.14)
			{
				msg.angular.z = 0.0;
				pub.publish(msg);
				break;
			}
			ros::spinOnce();
			rate.sleep();
		}
		ROS_INFO("ODOM %f", odomHeading);
		ROS_INFO("comH %f", comHeading);
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
	
	void pauseRobot()
	{
		geometry_msgs::Twist msg;
		msg.linear.x = 0.0;
		msg.angular.z = 0.0;
		pub.publish(msg);//suppress warnings 
		ros::Time begin = ros::Time::now();
		ros::Duration two_seconds(5, 0);
		ros::Rate rate(10);
		while(ros::Time::now() < begin + two_seconds)
		{
			pub.publish(msg);//suppress warnings.
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
/**
proj->pauseRobot();
	proj->rotate_abs(0);
proj->pauseRobot();
	proj->rotate_abs(M_PI/2);
proj->pauseRobot();
	proj->rotate_abs(M_PI);
proj->pauseRobot();
	proj->rotate_abs(-M_PI/2);
proj->pauseRobot();
	proj->rotate_abs(0);
*/
	geometry_msgs::Twist msg;
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
proj->pauseRobot();
	proj->rotate_abs(0);
proj->pauseRobot();
	ROS_INFO_STREAM("translate 1");
	proj->rotate_abs(M_PI/2);
	//proj->translate(1);////////////////////////
//proj->pauseRobot();
	ROS_INFO_STREAM("TURN 1");
	//proj->rotate_rel(M_PI/2);/////////////////////
//proj->pauseRobot();
	//proj->rotate_abs(M_PI/2);
	ROS_INFO_STREAM("translate 2");
	//proj->translate(1);////////////////
//proj->pauseRobot();

	ROS_INFO_STREAM("TURN 2");
	//proj->rotate_rel(M_PI/2);/////////////////////
//proj->pauseRobot();
	//proj->rotate_abs(M_PI);

	ROS_INFO_STREAM("translate 3");
	//proj->translate(1);//////////////////
//proj->pauseRobot();

	ROS_INFO_STREAM("TURN 3"); 
	//proj->rotate_rel(M_PI/2);///////////////////////
//proj->pauseRobot();
	//proj->rotate_abs(-M_PI);

	ROS_INFO_STREAM("translate 4");
	//proj->translate(1);///////////////////
//proj->pauseRobot();
	
	ROS_INFO_STREAM("TURN 4");
	//proj->rotate_rel(M_PI/2);/////////////////
	//proj->rotate_abs(0);
	ros::shutdown();
	ros::spin();
 return 0;
}










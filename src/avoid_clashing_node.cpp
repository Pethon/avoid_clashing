#define DISTANCE 1.5f

#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_vel_stop;
geometry_msgs::Twist cmd_vel_back;
ros::Publisher pub_cmd_vel;
bool obstacle = false;
double stop_time;
double now_time;

void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg){

	if(laser_msg->ranges[laser_msg->ranges.size()/2] <= DISTANCE){
		if(obstacle == false){
			stop_time = ros::Time::now().toSec();
		}
		obstacle = true;
	}
	else{
		obstacle = false;
	}
}

void callback_move_base(const geometry_msgs::Twist::ConstPtr& cmd_vel_from_move_base){

	if(obstacle == true){
		now_time = ros::Time::now().toSec();
		if(now_time - stop_time < 3){
			pub_cmd_vel.publish(cmd_vel_stop);
			ROS_ERROR("OBSTACLE DETECTED!!");
		}
		else if(now_time - stop_time < 15){
			cmd_vel_back.linear.x = -0.1;
			pub_cmd_vel.publish(cmd_vel_back);
			ROS_INFO("MOVE BACK!!");
		}
	}
	else{
		pub_cmd_vel.publish(cmd_vel_from_move_base);
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "avoid_clashing_node");

	ros::NodeHandle n;

	ros::Subscriber sub_laser = n.subscribe("/scan", 1, callback_laser);
	ros::Subscriber sub_move_base = n.subscribe("/cmd_vel_from_move_base", 1, callback_move_base);

	pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel", 1);

	ros::spin();
}

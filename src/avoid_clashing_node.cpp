#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_vel_stop;
ros::Publisher pub_cmd_vel;
bool obstacle = false;

void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg){

	if(laser_msg->ranges[laser_msg->ranges.size()/2] <= 2) obstacle = true;
	else obstacle = false;
}

void callback_move_base(const geometry_msgs::Twist::ConstPtr& move_base_cmd_vel){

	if(obstacle == false) pub_cmd_vel.publish(move_base_cmd_vel);
	else{
		pub_cmd_vel.publish(cmd_vel_stop);
		ROS_ERROR("DANGER!!");
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

#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist cmd_vel;
ros::Publisher pub_cmd_vel;

void callback_laser(const sensor_msgs::LaserScan::ConstPtr& msg){

	if(msg->ranges[msg->ranges.size()/2] <= 5){
		//cmd_vel.angular.z = 5;
		for(int i = 0; i < 4; i++){
			pub_cmd_vel.publish(cmd_vel);
			ROS_INFO("DANGER!!");
			usleep(50*1000);
		}
	}
}

int main(int argc, char **argv){

	ros::init(argc, argv, "avoid_clashing_node");

	ros::NodeHandle n;

	ros::Subscriber sub_laser = n.subscribe("/scan", 1, callback_laser);

	pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel", 1);

	ros::spin();
}

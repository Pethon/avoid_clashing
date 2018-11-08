#define DISTANCE 1.0f

#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

geometry_msgs::Twist cmd_vel_stop;
geometry_msgs::Twist cmd_vel_back;
move_base_msgs::MoveBaseActionGoal next_goal;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_goal;
bool obstacle = false;
double stop_time;
double now_time;

void callback_goal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal){

	next_goal.goal.target_pose.pose.position.x = goal->goal.target_pose.pose.position.x;
	next_goal.goal.target_pose.pose.position.y = goal->goal.target_pose.pose.position.y;
	next_goal.goal.target_pose.pose.position.z = goal->goal.target_pose.pose.position.z;

	next_goal.goal.target_pose.pose.orientation.x = goal->goal.target_pose.pose.orientation.x;
	next_goal.goal.target_pose.pose.orientation.y = goal->goal.target_pose.pose.orientation.y;
	next_goal.goal.target_pose.pose.orientation.z = goal->goal.target_pose.pose.orientation.z;

	ROS_INFO("goal subscribed");

}

void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg){

	if(laser_msg->ranges[laser_msg->ranges.size()/2] <= DISTANCE){
		if(obstacle == false){
			stop_time = ros::Time::now().toSec();
		}
		obstacle = true;
	}
	else{
		if(obstacle == true){
			pub_goal.publish(next_goal);
			ROS_INFO("goal published");
		}
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
	ros::Subscriber sub_goal = n.subscribe("/move_base/goal", 1, callback_goal);

	pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/icart_mini/cmd_vel", 1);
	pub_goal = n.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);

	ros::spin();
}

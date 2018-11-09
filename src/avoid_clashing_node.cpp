#define FRONT_DISTANCE 0.8f
#define BACK_DISTANCE 1.0f

#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

geometry_msgs::Twist cmd_vel_stop;
geometry_msgs::Twist cmd_vel;
move_base_msgs::MoveBaseActionGoal next_goal;
ros::Publisher pub_cmd_vel;
ros::Publisher pub_goal;
bool obstacle = false;
bool can_move_back = true;
double stop_time;
double now_time;
int move_back_step_count;

void callback_goal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& goal){

	next_goal.header.seq = goal->header.seq;
	next_goal.header.stamp = goal->header.stamp;
	next_goal.header.frame_id = goal->header.frame_id;
	next_goal.goal_id.stamp = goal->goal_id.stamp;
	next_goal.goal_id.id = goal->goal_id.id;

	next_goal.goal.target_pose.header.seq = goal->goal.target_pose.header.seq;
	next_goal.goal.target_pose.header.stamp = goal->goal.target_pose.header.stamp;
	next_goal.goal.target_pose.header.frame_id = goal->goal.target_pose.header.frame_id;

	next_goal.goal.target_pose.pose.position.x = goal->goal.target_pose.pose.position.x;
	next_goal.goal.target_pose.pose.position.y = goal->goal.target_pose.pose.position.y;
	next_goal.goal.target_pose.pose.position.z = goal->goal.target_pose.pose.position.z;

	next_goal.goal.target_pose.pose.orientation.x = goal->goal.target_pose.pose.orientation.x;
	next_goal.goal.target_pose.pose.orientation.y = goal->goal.target_pose.pose.orientation.y;
	next_goal.goal.target_pose.pose.orientation.z = goal->goal.target_pose.pose.orientation.z;
	next_goal.goal.target_pose.pose.orientation.w = goal->goal.target_pose.pose.orientation.w;

	ROS_INFO("goal subscribed");

}

void callback_laser(const sensor_msgs::LaserScan::ConstPtr& laser_msg){

	bool front_obstacle_check = false;
	bool back_obstacle_check = false;

	//check front scan
	for(int i = 480; i < laser_msg->ranges.size()-480; i++){
		if(laser_msg->ranges[i] <= FRONT_DISTANCE){
			front_obstacle_check = true;
		}
	}
	//check back scan
	for(int i = 0; i < 70; i++){
		if(laser_msg->ranges[i] <= BACK_DISTANCE){
			back_obstacle_check = true;
		}
	}
	for(int i = laser_msg->ranges.size()-70; i < laser_msg->ranges.size(); i++){
		if(laser_msg->ranges[i] <= BACK_DISTANCE){
			back_obstacle_check = true;
		}
	}

	if (front_obstacle_check == true){
		if(obstacle == false){
			stop_time = ros::Time::now().toSec();
			move_back_step_count = 0;
		}
		obstacle = true;
		ROS_INFO("front obstacle = true");
	}
	else{
		if(obstacle == true){
			pub_goal.publish(next_goal);
			ROS_INFO("goal published");
		}
		obstacle = false;
		ROS_INFO("front obstacle = false");
	}

	if(back_obstacle_check == true){
		can_move_back = false;
		ROS_INFO("back obstacle = true");
	}
	else{
		can_move_back = true;
		ROS_INFO("back obstacle = false");
	}
}

void callback_move_base(const geometry_msgs::Twist::ConstPtr& cmd_vel_from_move_base){

	if(obstacle == true){
		now_time = ros::Time::now().toSec();
		if(now_time - stop_time < 3){
			pub_cmd_vel.publish(cmd_vel_stop);
			ROS_ERROR("STOP!!");
		}
		else if(can_move_back){
			cmd_vel.linear.x = -0.1;
			cmd_vel.angular.z = 0.0;
			pub_cmd_vel.publish(cmd_vel);
			ROS_INFO("MOVE BACK!!");
		}
		else if(now_time - stop_time < 180){
			pub_cmd_vel.publish(cmd_vel_stop);
			ROS_ERROR("CAN'T MOVE BACK!!");
		}
		else{
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.2;
			pub_cmd_vel.publish(cmd_vel);
			ROS_INFO("START ROTATING!!");
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

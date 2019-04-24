#ifndef _GRIPPER_COMMANDER_H_
#define _GRIPPER_COMMANDER_H_


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

class GripperCommander {
	private:
	
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;
	
		ros::Subscriber gripper_command_sub;
		ros::Subscriber joint_states_sub;
		ros::Publisher dynamixel_goal_pub;

		double torque;
		bool torqued;
		std::string goal_position_topic;
	
	public:
		std::string TOPIC_PARAMETER_NAME_ = "goal_position_topic_name";
		
		GripperCommander();
		~GripperCommander();

		void onCommand(const std_msgs::String::ConstPtr& msg);
		void onJointState(const sensor_msgs::JointStateConstPtr& msg);
};


#endif //define _GRIPPER_COMMANDER_H_

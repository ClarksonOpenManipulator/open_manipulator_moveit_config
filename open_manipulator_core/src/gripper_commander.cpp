#include "open_manipulator_core/gripper_commander.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"


GripperCommander::GripperCommander() : nh_(""), priv_nh_("~") {
	if (!priv_nh_.getParam(TOPIC_PARAMETER_NAME_, goal_position_topic)) {
		ROS_INFO_STREAM("Param '" << TOPIC_PARAMETER_NAME_ << "' was not set. Using default topic of goal_position!");
		goal_position_topic = "goal_position";
	}

	gripper_command_sub = nh_.subscribe("/gripper_command", 1, &GripperCommander::onCommand, this);
	joint_states_sub = nh_.subscribe("/joint_states", 10, &GripperCommander::onJointState, this);
	dynamixel_goal_pub = nh_.advertise<sensor_msgs::JointState>(goal_position_topic, 10);
	torque = 1000;
	torqued = false;
}

GripperCommander::~GripperCommander() {}

void GripperCommander::onJointState(const sensor_msgs::JointStateConstPtr& msg) {
   int pos = std::find(msg->name.begin(), msg->name.end(), "id_7") - msg->name.begin();
   
	if (pos >= msg->name.size()) {
      ROS_INFO("No gripper effort received...");
      return;
   }
   
	if ((msg->effort[pos] > torque) && (!torqued)) {
		ROS_INFO("Torque too high...");
		sensor_msgs::JointState goalState;
		goalState.header.stamp = ros::Time::now();
		goalState.name.push_back("id_7");
		goalState.position.push_back(msg->position[pos]);
		goalState.velocity.push_back(0);
		goalState.effort.push_back(0);
		torqued = true;
		dynamixel_goal_pub.publish(goalState);
	}
}

void GripperCommander::onCommand(const std_msgs::String::ConstPtr& msg) {
	
	sensor_msgs::JointState gripperState;
	gripperState.header.stamp = ros::Time::now();
	gripperState.name.push_back("id_7");
	gripperState.velocity.push_back(0);
	gripperState.effort.push_back(0);

	if (msg->data == "grip_off") {
		ROS_INFO("Grip set to 'off'");
		gripperState.position.push_back(-3.14f);
	} else if (msg->data == "grip_on") {
		ROS_INFO("Grip set to 'on'");
		gripperState.position.push_back(-1.00f);
	} else {
		ROS_WARN("If you want to grip or release something, publish 'grip_on', 'grip_off' or 'neutral'");
		return;
	}
	torqued = false;
	dynamixel_goal_pub.publish(gripperState);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "gripper_commander");

	ROS_INFO("Starting gripper commander;");
	GripperCommander gripper_commander;

	ROS_INFO("Spinning...");
	ros::spin();

	return 0;
}

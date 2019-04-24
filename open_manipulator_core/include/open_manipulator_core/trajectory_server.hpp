#ifndef _TRAJECTORY_SERVER_H_
#define _TRAJECTORY_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <vector>

template <typename T> std::vector<T> operator-(const std::vector<T>& lhs, const std::vector<T>& rhs) {
	std::vector<T> res;
	res.reserve(lhs.size());

	for (size_t idx = 0; idx < lhs.size(); idx++) {
		res.push_back(lhs[idx] - rhs[idx]);
	}
	return res;
}

class DynamixelTrajectoryAction {
	public:
		std::string RATE_PARAMETER_NAME_ = "trajectory_action_rate";
		std::string TOPIC_PARAMETER_NAME_ = "goal_position_topic_name";
	
		DynamixelTrajectoryAction(const std::string& name);

		void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal);
		void onJointState(const sensor_msgs::JointState::ConstPtr& joints);

		double max_position_error = 0.5;
	
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle priv_nh_;
	
		std::string action_name_;
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	
		ros::Publisher jointPub;
		ros::Subscriber jointSub;
	
		sensor_msgs::JointState currentState;
	
		int rate;
      	std::string goal_position_topic;
};

#endif // _TRAJECTORY_SERVER_H_

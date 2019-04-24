#include "open_manipulator_core/trajectory_server.hpp"
#include <numeric>
#include <vector>
#include <array>
#include <cstdio>
#include <cstdlib>
#include "open_manipulator_core/spline.h"




DynamixelTrajectoryAction::DynamixelTrajectoryAction(const std::string& name) : nh_(""), priv_nh_("~"), as_(nh_, name, boost::bind(&DynamixelTrajectoryAction::executeCB, this, _1), false), action_name_(name) {
	if (!priv_nh_.getParam(RATE_PARAMETER_NAME_, rate)) {
		ROS_INFO_STREAM("Param '" << RATE_PARAMETER_NAME_ << "' was not set. Using default rate of 100hz.");
		rate = 100;
	}
	if (!priv_nh_.getParam(TOPIC_PARAMETER_NAME_, goal_position_topic)) {
		ROS_INFO_STREAM("Param '" << TOPIC_PARAMETER_NAME_ << "' was not set. Using default topic of goal_position!");
		goal_position_topic = "goal_position";
	}
	ROS_INFO_STREAM("Trajectory Action Sample Rate: " << rate);
	ROS_INFO_STREAM("Trajectory Action Goal Topic: " << goal_position_topic);
	
	jointPub = nh_.advertise<sensor_msgs::JointState>(goal_position_topic, 10);
	jointSub = nh_.subscribe<sensor_msgs::JointState>("joint_states", 1, boost::bind(&DynamixelTrajectoryAction::onJointState, this, _1));
	as_.start();
}

void DynamixelTrajectoryAction::onJointState(const sensor_msgs::JointState::ConstPtr& joints) {
  	currentState = *joints;
}

void DynamixelTrajectoryAction::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal) {
	//Get useful info from goal message
	std::vector<trajectory_msgs::JointTrajectoryPoint> points = goal->trajectory.points;
	std::vector<std::string> names = goal->trajectory.joint_names;
   
   	sensor_msgs::JointState goalState;
   	goalState.header.seq = 0;
   
   	ros::Time startTime = ros::Time::now();
   	bool success = true;
   	ROS_INFO("Begin trajectory...");
	
	ros::Rate sampleRate(rate);
	for (size_t pointIdx = 0; pointIdx < points.size(); pointIdx++) {
		if (!ros::ok()) break;
		
		ros::Time now = ros::Time::now();
      	ros::Duration sinceStart = now - startTime;
		
		goalState.header.seq++;
      	goalState.header.stamp = now;
      	std::vector<double> position;
      	std::vector<double> velocity;
      	std::vector<double> effort;
      	for (size_t jointIdx = 0; jointIdx < names.size(); jointIdx++) {
         	position.push_back(points[pointIdx].positions[jointIdx]);
         	velocity.push_back(points[pointIdx].velocities[jointIdx]);
         	effort.push_back(points[pointIdx].effort[jointIdx]);
      	}
      	goalState.name = names;
      	goalState.position = position;
      	goalState.velocity = velocity;
      	goalState.effort = effort;
		
      	jointPub.publish(goalState);
      
      	if (as_.isPreemptRequested()) {
         	as_.setPreempted();
         	success = false;
         	break;
      	}
		
	
		
		control_msgs::FollowJointTrajectoryFeedback feedback;
	  	feedback.joint_names = names;
		
	  	feedback.desired.positions = goalState.position;

		std::vector<double> actualPositions;
		actualPositions.resize(names.size());
		
		for (size_t i = 0; i < names.size(); i++) {
			for (size_t j = 0; j < currentState.name.size(); j++) {
				if (names[i] == currentState.name[j]) {
					actualPositions[i] = currentState.position[j]; 
				}
			}				 
		}
		
		feedback.actual.positions = actualPositions;
	  	feedback.error.positions = feedback.actual.positions - feedback.desired.positions;

	  	feedback.desired.time_from_start = sinceStart;
	  	feedback.actual.time_from_start = sinceStart;
	  	feedback.error.time_from_start = sinceStart;

	  	as_.publishFeedback(feedback);
	  	bool limit_exceeded = false;
	  	
	  	for (size_t jointIdx = 0; jointIdx < feedback.error.positions.size(); jointIdx++) {
         	if (std::abs(feedback.error.positions[jointIdx]) > max_position_error) {
            	limit_exceeded = true;
            	break;
         	}
      	}
		
		if (limit_exceeded) {
         	control_msgs::FollowJointTrajectoryResult result;
         	ROS_ERROR_STREAM("Too high position error!!!");
         
         	std::stringstream ss;
         
         	ss << "Position error: [";
         	for(size_t i = 0; i < feedback.error.positions.size(); i++)
         	{
           	if(i != 0)
             	ss << ", ";
          	ss << feedback.error.positions[i];
         	}
         	ss << "]";
         	std::string s = ss.str();
         
         	ROS_INFO_STREAM(s);
         
         	result.error_code = result.PATH_TOLERANCE_VIOLATED;
         	as_.setAborted(result);
         	success = false;
         	break;
      	}
		sampleRate.sleep();
	}
   
   	if (success) {
      	control_msgs::FollowJointTrajectoryResult result;
      	result.error_code = result.SUCCESSFUL;
      	ROS_INFO_STREAM("Current trajectory complete...");
      	as_.setSucceeded(result);
   	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "trajectory_action_server");

	ROS_INFO("Starting action server...");
	DynamixelTrajectoryAction action("arm/joint_trajectory_action");

	ROS_INFO("Spinning...");
	ros::spin();

	return 0;
}

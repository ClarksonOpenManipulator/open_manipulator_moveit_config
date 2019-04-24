#include "open_manipulator_moveit_config/uniform_n_point_filter.h"
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <sstream>

using namespace open_manipulator_moveit_config;
         
UniformNPointFilter::UniformNPointFilter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~") {
   if (!nh_.getParam(RATE_PARAMETER_NAME_, sample_time)) {
      ROS_INFO_STREAM("Param '" << RATE_PARAMETER_NAME_ << "' was not set. Using default rate.");
      sample_time = 0.25;
   }
   if (!nh_.getParam(COUNT_PARAMETER_NAME_, sample_count)) {
      ROS_INFO_STREAM("Param '" << COUNT_PARAMETER_NAME_ << "' was not set. Using default count.");
      sample_count = 10;
   }
   ROS_INFO_STREAM("Uniform_N_Point_Filter parameters:\n\t" <<
                    RATE_PARAMETER_NAME_ << ": " << sample_time << "\n\t" <<
                    COUNT_PARAMETER_NAME_ << ": " << sample_count);
}
         
UniformNPointFilter::~UniformNPointFilter() { };
         
std::string UniformNPointFilter::getDescription() const { return "Resamples trajectory to a specified sample count at a fixed sample rate."; }
         
bool UniformNPointFilter::adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                          const planning_interface::MotionPlanRequest& req,
                          planning_interface::MotionPlanResponse& res,
                          std::vector<size_t>& added_path_index) const {
   bool result = planner(planning_scene, req, res); //Call moveit planning before doing anything
   
   if (result && res.trajectory_) { //Initial plan was successful
      robot_trajectory::RobotTrajectory traj = *res.trajectory_; //Copy trajectory so original can be cleared
      
      double goalEndTime = sample_time * sample_count; //Total trajectory time after this filter is applied
      size_t plannedPointCount = traj.getWayPointCount(); //Number of points moveit in the trajectory created by moveit
      double plannedEndTime = traj.getWayPointDurationFromStart(plannedPointCount - 1); //Total trajectory time as planned by moveit (before this filter is applied)
      ROS_INFO_STREAM("\nRunning Uniform_N_Point_Filter" << 
                      "\n\tgoalEndTime: " << goalEndTime << 
                      "\n\tplannedPointCount: " << plannedPointCount << 
                      "\n\tplannedEndTime: " << plannedEndTime);
                      
      res.trajectory_->clear(); //Clear original trajectory (rebuilding completely)
      double lastTime = 0;
      for (size_t idx = 0; idx <= sample_count; idx++) {
         double timeInGoalRange = sample_time * idx; //Time of current point in final trajectory (after this filter is applied)
         double timeInPlannedRange = (timeInGoalRange / goalEndTime) * plannedEndTime; //Corresponding time in original trajectory
         
         ROS_INFO_STREAM("timeInPlannedRange: " << timeInPlannedRange);
         
         ROS_INFO_STREAM("IDX: " << idx);
         
         int idxBefore = 0; int idxAfter = 0; double blend = 1;
         traj.findWayPointIndicesForDurationAfterStart(timeInPlannedRange, idxBefore, idxAfter, blend); //Find points in original trajectory on either side of current time.
         ROS_INFO_STREAM("Before: " << idxBefore << " After: " << idxAfter << " Blend: " << blend << "\n");
         
         robot_state::RobotState stateBefore = traj.getWayPoint(idxBefore);
         robot_state::RobotState stateAfter = traj.getWayPoint(idxAfter);
         robot_state::RobotState state = stateBefore;
         stateBefore.interpolate(stateAfter, blend, state); //Linear interpolate to find point at current time.
         
         res.trajectory_->addSuffixWayPoint(state, timeInGoalRange - lastTime); //Add interpolated state and time since last state to trajectory 
         lastTime = timeInGoalRange;
      }
      
      ROS_INFO_STREAM("END LOOP");
                      
      /*std::stringstream s("");
      s << "\n\nTRAJ:\n";
      for (size_t i = 0; i < plannedPointCount; i++){
         s << "\n" << i << ", " << traj.getWayPointDurationFromStart(i);
         moveit::core::RobotState curState = traj.getWayPoint(i);
         for (size_t j = 0; j < curState.getVariableCount(); j++) {
            s << ", " << curState.getVariablePosition(j);
         }
      }
      s << "\n\nRES.TRAJ:\n";
      for (size_t i = 0; i < sample_count; i++){
         s << "\n" << i << ", " << res.trajectory_->getWayPointDurationFromStart(i);
         moveit::core::RobotState curState = res.trajectory_->getWayPoint(i);
         for (size_t j = 0; j < curState.getVariableCount(); j++) {
            s << ", " << curState.getVariablePosition(j);
         }
      }
      ROS_INFO_STREAM(s.str());*/
   }
   
   return result;
}

const std::string UniformNPointFilter::RATE_PARAMETER_NAME_ = "uniform_n_point_filter_rate";
const std::string UniformNPointFilter::COUNT_PARAMETER_NAME_ = "uniform_n_point_filter_count";

CLASS_LOADER_REGISTER_CLASS(open_manipulator_moveit_config::UniformNPointFilter, planning_request_adapter::PlanningRequestAdapter)

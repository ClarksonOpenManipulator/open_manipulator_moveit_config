#include "open_manipulator_moveit_config/cubic_spline_adapter.hpp"

using namespace open_manipulator_moveit_config;

CubicSplineAdapter::CubicSplineAdapter() : planning_request_adapter::PlanningRequestAdapter(), nh_("~") {
	if (!nh_.getParam(RATE_PARAMETER_NAME_, sample_rate)) {
		ROS_INFO_STREAM("Param '" << RATE_PARAMETER_NAME_ << "' was not set. Using default rate of 100hz.");
		sample_rate = 100;
	}
	ROS_INFO_STREAM("Cubic Spline Adapter Sample Rate: " << sample_rate);
}
	
CubicSplineAdapter::~CubicSplineAdapter() { }

std::string CubicSplineAdapter::getDescription() const { return "Cubic Spline Interpolation of JointTrajectoryController"; }

bool CubicSplineAdapter::adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
									  const planning_interface::MotionPlanRequest& req,
									  planning_interface::MotionPlanResponse& res,
									  std::vector<size_t>& added_path_index) const {
	bool result = planner(planning_scene, req, res);		//Call moveit planning before doing anything
	
	if (result && res.trajectory_) { //Initial plan was successful
		robot_trajectory::RobotTrajectory traj = *res.trajectory_;
		
		size_t varCount = traj.getFirstWayPoint().getVariableCount();
		
		// 4 splines per variable [position, velocity, acceleration, effort]
		// format = [var1_pos var1_vel var1_acc var1_eff var2_pos var2_vel var2_acc var2_eff ....]
		// length = varCount * 4
		std::vector<tk::spline> splines;
		
		std::vector<double> time;
		for (size_t pointIdx = 0; pointIdx < traj.getWayPointCount(); pointIdx++) {
			time.push_back(traj.getWayPointDurationFromStart(pointIdx));
		}
		
		for (size_t varIdx = 0; varIdx < varCount; varIdx++) {
			std::vector<double> position;
			std::vector<double> velocity;
			std::vector<double> acceleration;
			std::vector<double> effort;
			
			for (size_t pointIdx = 0; pointIdx < traj.getWayPointCount(); pointIdx++) {
				position.push_back(traj.getWayPoint(pointIdx).getVariablePosition(varIdx));
				velocity.push_back(traj.getWayPoint(pointIdx).getVariableVelocity(varIdx));
				acceleration.push_back(traj.getWayPoint(pointIdx).getVariableAcceleration(varIdx));
				effort.push_back(traj.getWayPoint(pointIdx).getVariableEffort(varIdx));
			}
			
			tk::spline posSpline;
			posSpline.set_points(time, position);
			splines.push_back(posSpline);
			
			tk::spline velSpline;
			velSpline.set_points(time, velocity);
			splines.push_back(velSpline);
			
			tk::spline accSpline;
			accSpline.set_points(time, acceleration);
			splines.push_back(accSpline);
			
			tk::spline effSpline;
			effSpline.set_points(time, effort);
			splines.push_back(effSpline);
		}
		
		res.trajectory_->clear(); //Clear original trajectory (rebuilding completely)
		
		double startTime = traj.getWayPointDurationFromStart(0);
		double endTime = traj.getWayPointDurationFromStart(traj.getWayPointCount() - 1);
		double period = 1.0 / sample_rate;
		for (double curTime = startTime; curTime <= endTime; curTime += period) {
			//ROS_INFO_STREAM("StartTime: " << startTime << " CurrentTime: " << curTime << " EndTime: " << endTime << " Period: " << period);
			robot_state::RobotState curState(traj.getRobotModel());
			
			for (size_t varIdx = 0; varIdx < varCount; varIdx++) {
				size_t splineBaseIdx = (varIdx * 4);
				curState.setVariablePosition		(varIdx, splines[splineBaseIdx + 0](curTime));
				curState.setVariableVelocity		(varIdx, splines[splineBaseIdx + 1](curTime));
				curState.setVariableAcceleration	(varIdx, splines[splineBaseIdx + 2](curTime));
				curState.setVariableEffort			(varIdx, splines[splineBaseIdx + 3](curTime));
			}
			res.trajectory_->addSuffixWayPoint(curState, period);
		}
	}
	return result;
}

const std::string CubicSplineAdapter::RATE_PARAMETER_NAME_ = "cubic_spline_adapter_rate";

CLASS_LOADER_REGISTER_CLASS(open_manipulator_moveit_config::CubicSplineAdapter, planning_request_adapter::PlanningRequestAdapter)
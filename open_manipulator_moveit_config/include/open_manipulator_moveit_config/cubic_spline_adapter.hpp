#ifndef _CUBIC_SPLINE_ADAPTER_
#define _CUBIC_SPLINE_ADAPTER_

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "open_manipulator_moveit_config/spline.h"
////////////////////////////////////////// IDFK IF NEEDED B~RRRRR~A
#include <numeric>
#include <vector>
#include <array>
#include <cstdio>
#include <cstdlib>

namespace open_manipulator_moveit_config {
	class CubicSplineAdapter : public planning_request_adapter::PlanningRequestAdapter {
		public:
			static const std::string RATE_PARAMETER_NAME_;
			
			CubicSplineAdapter();
			~CubicSplineAdapter();

			virtual std::string getDescription() const;
			virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
									  const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
									  std::vector<size_t> &added_path_index) const;
		private:
			ros::NodeHandle nh_;
			int sample_rate;
	};
}

#endif //_CUBIC_SPLINE_ADAPTER_

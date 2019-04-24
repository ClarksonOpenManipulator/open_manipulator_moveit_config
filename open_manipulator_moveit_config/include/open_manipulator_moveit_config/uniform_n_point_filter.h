#ifndef _UNIFORM_N_POINT_FILTER_
#define _UNIFORM_N_POINT_FILTER_

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <class_loader/class_loader.h>

#include <ros/ros.h>
#include <ros/console.h>

namespace open_manipulator_moveit_config {
   class UniformNPointFilter : public planning_request_adapter::PlanningRequestAdapter {
      public:
         static const std::string RATE_PARAMETER_NAME_;
         static const std::string COUNT_PARAMETER_NAME_;
         
         UniformNPointFilter();
         ~UniformNPointFilter();
         
         virtual std::string getDescription() const;
         virtual bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                                   const planning_interface::MotionPlanRequest& req,
                                   planning_interface::MotionPlanResponse& res,
                                   std::vector<size_t> &added_path_index) const;
      private:
         ros::NodeHandle nh_;
         std::string filter_name_;
         double sample_time;
         int sample_count;
   };
}

#endif //_UNIFORM_N_POINT_FILTER_

#include "open_manipulator_moveit_config/move_group_commander_service.h"

MoveGroupCommanderService::MoveGroupCommanderService() : _nh(""), _priv_nh("~") {
   if (!_priv_nh.getParam(MOVE_GROUP_NAME_PARAMETER_NAME_, group_name)) {
      ROS_INFO_STREAM("Param '" << MOVE_GROUP_NAME_PARAMETER_NAME_ << "' was not set. Using default name.");
      group_name = "arm"; //"default_move_group_name";
   }
   
   END_EFFECTOR_OFFSET_X_PARAMETER_NAME_ = group_name + "_offset_x";
   END_EFFECTOR_OFFSET_Y_PARAMETER_NAME_ = group_name + "_offset_y";
   END_EFFECTOR_OFFSET_Z_PARAMETER_NAME_ = group_name + "_offset_z";
   
   bool gotXOffset = _priv_nh.getParam(END_EFFECTOR_OFFSET_X_PARAMETER_NAME_, xOffset);
   bool gotYOffset = _priv_nh.getParam(END_EFFECTOR_OFFSET_Y_PARAMETER_NAME_, yOffset);
   bool gotZOffset = _priv_nh.getParam(END_EFFECTOR_OFFSET_Z_PARAMETER_NAME_, zOffset);
   
   if (!(gotXOffset && gotYOffset && gotZOffset)) {
      ROS_INFO_STREAM("Offset params were not properly set. Using defaults...");
      xOffset = 0;
      yOffset = 0;
      zOffset = 0.1; //////////////////////////////////////////////////////////////////////////////
   }
   
   ROS_INFO_STREAM("Offsets: " << xOffset << ", " << yOffset << ", " << zOffset);
   
   commander_service = _nh.advertiseService("open_manipulator/moveit/" + group_name + "/commander",
                                            &MoveGroupCommanderService::commanderCb, this);
   groupInterface = new moveit::planning_interface::MoveGroupInterface(group_name);
   ROS_INFO_STREAM("group_name: " << group_name);
}

MoveGroupCommanderService::~MoveGroupCommanderService() {

}
      
bool MoveGroupCommanderService::commanderCb(open_manipulator_moveit_config::MoveGroupCommand::Request& req, 
                                            open_manipulator_moveit_config::MoveGroupCommand::Response& res) {
   req.target.position.x += xOffset;
   req.target.position.y += yOffset; 
   req.target.position.z += zOffset;                                
   groupInterface->setPoseTarget(req.target);
   moveit::planning_interface::MoveGroupInterface::Plan move_plan;
   
   bool planSuccess = (bool)groupInterface->plan(move_plan);
   bool executeSuccess = (bool)groupInterface->execute(move_plan);
   ROS_INFO_STREAM("Commander service finished...");
   
   bool success = planSuccess && executeSuccess;
   res.success = success;
   return success;
}

int main(int argc, char* argv[]) {
   ros::init(argc, argv, "move_group_commander_service");
   
   ROS_INFO("Starting commander service");
   MoveGroupCommanderService commanderService;
   
   ROS_INFO("Spinning...");
   ros::AsyncSpinner spinner(2);
   spinner.start();
   ros::waitForShutdown();
   return 0;
}

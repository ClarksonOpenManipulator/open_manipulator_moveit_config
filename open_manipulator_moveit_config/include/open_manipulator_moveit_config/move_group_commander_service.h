#ifndef _MOVE_GROUP_COMMANDER_SERVICE_H_
#define _MOVE_GROUP_COMMANDER_SERVICE_H_

#include <ros/ros.h>
#include <ros/console.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include "open_manipulator_moveit_config/MoveGroupCommand.h"

class MoveGroupCommanderService {

   private:
      const std::string MOVE_GROUP_NAME_PARAMETER_NAME_ = "group_name";
      std::string END_EFFECTOR_OFFSET_X_PARAMETER_NAME_;
      std::string END_EFFECTOR_OFFSET_Y_PARAMETER_NAME_;
      std::string END_EFFECTOR_OFFSET_Z_PARAMETER_NAME_;
   
      std::string group_name;
      
      double xOffset;
      double yOffset;
      double zOffset;
      
      ros::NodeHandle _nh;
		ros::NodeHandle _priv_nh;
      ros::ServiceServer commander_service;
      
      moveit::planning_interface::MoveGroupInterface* groupInterface;
      
   public:
      MoveGroupCommanderService();
      ~MoveGroupCommanderService();
      
      bool commanderCb(open_manipulator_moveit_config::MoveGroupCommand::Request& req, open_manipulator_moveit_config::MoveGroupCommand::Response& res);

};


#endif //_MOVE_GROUP_COMMANDER_SERVICE_H_

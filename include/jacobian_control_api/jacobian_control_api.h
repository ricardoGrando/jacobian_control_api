#ifndef JACOBIAN_CONTROL_H
#define JACOBIAN_CONTROL_H

#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include "thormang3_manipulation_module_msgs/JointPose.h"
#include "thormang3_manipulation_module_msgs/KinematicsPose.h"
#include "thormang3_kinematics_dynamics/kinematics_dynamics.h"

bool jacobian(jacobian_control_api::jacobian::Request &req,
             jacobian_control_api::jacobian::Response &res);

thormang3::KinematicsDynamics *robotis_;

#endif


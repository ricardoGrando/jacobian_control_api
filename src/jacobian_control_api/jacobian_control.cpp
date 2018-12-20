#include "ros/ros.h"
#include "jacobian_control_api/jacobian.h"
#include "jacobian_control_api/jacobian_control_api.h"
#include <stdio.h>
#include <iostream>

bool jacobian(jacobian_control_api::jacobian::Request &req,
             jacobian_control_api::jacobian::Response &res)
{
  
  // id of the joints.
  int ik_id_start_ = req.ik_id_start;
  int ik_id_end_ = req.ik_id_end;  // 14 is the last 34 is the end effector

  // set all joint angles
  for (auto jointPose = req.angles.begin(); jointPose != req.angles.end(); jointPose++)
  {
    for (int id = ik_id_start_ ; id <= ik_id_end_ ; id += 1)
    {
      if (robotis_->thormang3_link_data_[id]->name_ == jointPose->name)
      {
        robotis_->thormang3_link_data_[id]->joint_angle_ = jointPose->value;
        //std::cout << jointPose->value << std::endl;
        //std::cout << id << std::endl;
      }
    }
  }

  // Make the FK 
  robotis_->calcForwardKinematics(ik_id_start_);

  // set the end effector matrix to multoply
  Eigen::MatrixXd delta_end_effector  = Eigen::MatrixXd::Zero(6,1);
  delta_end_effector.coeffRef(0, 0) = req.desiredStep.position.x;
  delta_end_effector.coeffRef(1, 0) = req.desiredStep.position.y;
  delta_end_effector.coeffRef(2, 0) = req.desiredStep.position.z;
  delta_end_effector.coeffRef(3, 0) = req.desiredStep.orientation.x;
  delta_end_effector.coeffRef(4, 0) = req.desiredStep.orientation.y;
  delta_end_effector.coeffRef(5, 0) = req.desiredStep.orientation.z;
  
  thormang3_manipulation_module_msgs::JointPose jointPose;
     
  // joints id's for jacobian calculation
  std::vector<int> idx = robotis_->findRoute(ik_id_start_, ik_id_end_);
  
  //calc the jacobian for them
  Eigen::MatrixXd jacobian = robotis_->calcJacobian(idx);

  // Invert the matrix
  Eigen::MatrixXd jacobian_trans = jacobian * jacobian.transpose();
  Eigen::MatrixXd jacobian_inv = jacobian.transpose() * jacobian_trans.inverse();

  // And find the delta angles
  Eigen::MatrixXd delta_angles = jacobian_inv*delta_end_effector;

  //   ROS_INFO("Here is the delta angles:");
  //   std::cout << delta_angles << std::endl;

  // The pose for the push back
  thormang3_manipulation_module_msgs::JointPose deltaAngle;

  // push back all delta joints
  int counter = 0;
  for (auto jointPose = req.angles.begin(); jointPose != req.angles.end(); jointPose++)
  {
    for (int id = ik_id_start_ ; id <= ik_id_end_ ; id += 1)
    {
      if (robotis_->thormang3_link_data_[id]->name_ == jointPose->name)
      {
        deltaAngle.name = robotis_->thormang3_link_data_[id]->name_;
        deltaAngle.value = delta_angles.coeffRef(counter, 0);
        res.deltaAngles.push_back(deltaAngle); 
        
        counter = counter + 1;
      }
    }
  } 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jacobian_server");
  ros::NodeHandle n;
  
  //ROS_INFO("Creating the KinematicsDynamics object");
  robotis_ = new thormang3::KinematicsDynamics(thormang3::WholeBody);
  // These are declared in thormang3_kinematics_dynamics/kinematics_dynamics.h

  ros::ServiceServer service = n.advertiseService("jacobian_control_api/jacobian", jacobian);
  ROS_INFO("The service is ready!");

  ros::spin();

  return 0;
}


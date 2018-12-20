#!/usr/bin/env python

import rospy
from jacobian_control_api.srv import *
from jacobian_control_api_msgs.msg import *
from std_msgs.msg import *
import numpy as np
import math

def create_msg(joints, jointsValue, desiredStep):
    cjp = []
    time = rospy.get_time()
    for value, j in enumerate(joints, 0):
        cjp.append(thormang3_manipulation_module_msgs.msg.JointPose(j, jointsValue[value], time))
    tp = geometry_msgs.msg.Point(
            x=desiredStep[0],y=desiredStep[1],z=desiredStep[2])
    to = geometry_msgs.msg.Quaternion(\
            x=desiredStep[3],y=desiredStep[4],\
            z=desiredStep[5],w=0)
    dp = geometry_msgs.msg.Pose(tp, to)
    
    return cjp, dp

def call_jacobian_service(joints, jointsValue, desiredStep, ik_id_start, ik_id_end):
    rospy.wait_for_service('jacobian_control_api/jacobian')
    cjp, dp = create_msg(joints, jointsValue, desiredStep)
    # Call the service and get the response
    calc_ik_function = rospy.ServiceProxy('jacobian_control_api/jacobian', jacobian)
    res = calc_ik_function(cjp,dp, ik_id_start, ik_id_end)

    return res

def inverse_kinematics_callback(data):
    # Gets the target cartesian and the angles and put in np array
    target_position = np.array([cart for cart in data.target_end_effector]) 
    angles = np.array([joint for joint in data.initial_angles])

    # verify which member is. There is only left arm and right arm for now
    if data.part == 'left_arm':
        joints = [   'l_arm_sh_p1', 'l_arm_sh_r', 'l_arm_sh_p2', 'l_arm_el_y', 'l_arm_wr_r', 'l_arm_wr_y', 'l_arm_wr_p']
    else:
        joints = [   'r_arm_sh_p1', 'r_arm_sh_r', 'r_arm_sh_p2', 'r_arm_el_y', 'r_arm_wr_r', 'r_arm_wr_y', 'r_arm_wr_p']

    # Gets the first and final ID
    ik_id_start = data.ik_id_start # 2 is the l_arm_sh_p1
    ik_id_end = data.ik_id_end  # 14 is the l_arm_wr_p and(Note that 34 is the id of the end effector)

    while(True):
        # Inside the loop towards the target position the first thing is to do the FK. The FK package msut be implemented. Its return must be the cartesian
        # coordinates with the angles in EULER. This is the example of the return when calling this service
        ##################################################################################################
        ####################### call the fk to get the actual position and orientation ###################
        actualPose = np.array([0.43, 0.463, 0.121, 3.1415, 0.0, 0.0]) # the fk for all angles == 0 in the left arm. THE ORIENTATION MUST BE IN EULER REPRESENTATION
        ##################################################################################################
        ################################################################################################

        # Then, with the actual pose, the distance is calculated
        distance = target_position - actualPose

        # If the biggest element of the distance is bigger then the step_size, the process towards decreasing the distance is made
        if (np.linalg.norm(distance) > data.step_size):
            # The delta end effector is calculated considering its limits
            delta_end_effector = ((distance)*data.step_size)/np.max(max(distance))             
            # The service to calculate the delta angles is called
            res = call_jacobian_service(joints, angles, delta_end_effector, ik_id_start, ik_id_end)
            
            deltaAngles = np.array([    res.deltaAngles[0].value, 
                                        res.deltaAngles[1].value,
                                        res.deltaAngles[2].value,
                                        res.deltaAngles[3].value,
                                        res.deltaAngles[4].value,
                                        res.deltaAngles[5].value,
                                        res.deltaAngles[6].value
                                    ])
            # And the angles is summed with the deltas
            angles += deltaAngles
        # If the biggest element of the distance is lower then the step_size, the position is achieved
        else:
            print("Arrived!!!!")
            break

def ik_jacobian_control():
		
	rospy.init_node("ik_jacobian_control", anonymous=False)

	# This topic receives as object CartsianMsg:
    #  target_end_effector is the position and the orientation(IN EULER) for the end-effector
    #  initial_angles is the set of the initial joints of the member of thormang. For example: all the seven joints angles of the left arm
    #  step_size is the maximum step to go to target position. The step size refers to the distance for x, y, z and its orientation
    #  part is the member of the thormang that the IK must be done. The left_arm and right_arm.
    #  ik_id_start is the id of the first joint of the member. Id = 2 is the joint of the left_arm for example
    #  ik_id_end is the id of the last joint of the member(NOT ITS END EFECTOR NECESSARLY). id = 14 is last of the left_arm
	rospy.Subscriber('/ik_jacobian_control', CartesianMsg, inverse_kinematics_callback)
	
	rospy.spin()

####################################### Exemplo para testar a aplicacao #######################################
# rostopic pub -1 /cartesian_position inno_arm_control_msgs/CartesianMsg "target_end_effector: [-17, 2, 20, -2.7, 1.46, 2.67] step_size: 0.1"		

if __name__ == "__main__":       
    try:
        ik_jacobian_control()
    except rospy.ROSInterruptException:
        pass

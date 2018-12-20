# Jacobian Control

A package to make the thormang's IK using the inverse jacobian matrix

## How to Run

***cd catkin_ws/src***

***git clone https://github.com/ricardoGrando/jacobian_control_api.git***   

***git clone https://github.com/ricardoGrando/jacobian_control_api_msgs.git***   

***cd ~/catkin_ws && catkin_make -j4***   

***roscore*** 

Open new Terminal

***source ~/catkin_ws/devel/setup.bash***  

***rosrun jacobian_control_api jacobian_control***  

Open new Terminal

***chmod +x ~/catkin_ws/src/jacobian_control_api/scripts/ik.py***  

***rosrun jacobian_control ik.py***  

## Usage example

*** rostopic pub -1 /ik_jacobian_control jacobian_control_api_msgs/CartesianMsg "target_end_effector: [0.5, 0.462, 0.121, 3.1415, 0.0, 0.0] 
initial_angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
step_size: 0.001 
part: 'left_arm' 
ik_id_start: 2 
ik_id_end: 14"




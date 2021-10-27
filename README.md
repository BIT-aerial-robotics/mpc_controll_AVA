# mpc_controll_AVA
This code is the MPC controller of the Aerial Vehicles Assembly(AVA) platform. It is a ROS node and can run on the ubuntu18.04/ROS Melodic system.The function of this node is to keep the hover of the AVA platform.

![image](https://github.com/BIT-aerial-robotics/mpc_controll_AVA/blob/master/%E4%BB%A3%E7%A0%81%E6%9E%B6%E6%9E%84.png)

As shown in the figure, if the node get_master_make_m publishes data 1 to /mode_switch, run the mpc_controller node, otherwise run the controller node. The output of the controller node and the mpc_controller node are the same, both are the attitude and pulling force of the three-shelf aircraft. The plant node is the dynamic simulation model of the AVA platform.


1. Use ACADO to realize the relevant code explanation of MPC controller


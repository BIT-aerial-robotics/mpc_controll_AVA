/**
 * author: kaidi wang
 * list all the global param 
*/
#include <stdio.h>
#include <string.h>

#define PI 3.1415926
//this value is very importent, it directly determines whether the program can run smoothly
#define MID 1520 
int node_id = 0;//self ID, master node ID = 0 
int child_node_num=3;// the number of child node, in this system contain 3 child px4
bool get_home_position = 0;//if get the home_position, set to 1

//define some parameter of keep changing roll and pitch angle
float keep_roll_speed = 0.005;
float roll_angle_max = -0.1;


float keep_pitch_speed = 0.025;
float pitch_angle_max = 0.5;



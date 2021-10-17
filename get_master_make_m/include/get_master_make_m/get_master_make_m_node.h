/*author: kaidi wang*/
/*date:2020.12.17*/

//define some functions used in get_master_make_m_node.cpp

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <get_master_make_m/param_list.h>//include the param list 
#include <fstream>//output log file
#include <math.h>
//define some function
//Coordinate System transform, enu to ned, position
geometry_msgs::Point enu2ned_pos(geometry_msgs::Point enu_pos)
{
    geometry_msgs::Point ned_pos;
    ned_pos.x = enu_pos.y;
    ned_pos.y = enu_pos.x;
    ned_pos.z = -enu_pos.z; 
    return ned_pos;
}
//Coordinate System transform, enu to ned, velcity
geometry_msgs::Point enu2ned_vel(geometry_msgs::Point enu_vel)
{
    geometry_msgs::Point ned_vel;
    ned_vel.x = enu_vel.y;
    ned_vel.y = enu_vel.x;
    ned_vel.z = -enu_vel.z; 
    return ned_vel;
}

//Coordinate System transform, enu to ned, euler
geometry_msgs::Point enu2ned_euler(geometry_msgs::Point enu_euler)
{   
    geometry_msgs::Point ned_euler;
    ned_euler.x =  enu_euler.x;
    ned_euler.y = -enu_euler.y;
    ned_euler.z = -enu_euler.z + PI/2;
    return ned_euler;
}
//Coordinate System transform, enu to ned, euler_rate
geometry_msgs::Point enu2ned_euler_rate(geometry_msgs::Point enu_euler_rate)
{   
    geometry_msgs::Point ned_euler_rate;
    ned_euler_rate.x =  enu_euler_rate.y;
    ned_euler_rate.y =  enu_euler_rate.x;
    ned_euler_rate.z = -enu_euler_rate.z;
    return ned_euler_rate;
}

//define some class
//class list
class vector3
{
private:
    /* data */
public:
    float x;
    float y;
    float z;
    vector3(/* args */);
    ~vector3();
};

vector3::vector3(/* args */)
{
}

vector3::~vector3()
{
}

class vector4
{
private:
    /* data */
public:
    float x;
    float y;
    float z;
    float w;
    vector4(/* args */);
    ~vector4();
};

vector4::vector4(/* args */)
{
}

vector4::~vector4()
{
}


class home_position_pose
{
private:
    /* data */
public:

    vector3 pos;
    vector4 quaternion;
    vector3 linear_velocity;
    vector3 angular_velocity;
    //float ;
    home_position_pose(/* args */);
    ~home_position_pose();
};


//init data of original position and pose
home_position_pose::home_position_pose(/* args */)
{
 
}

home_position_pose::~home_position_pose()
{
}


class sw_mapping
{
private:
    float fly_speed  = 1;          //unit m/s------ slower to 5m/s
    float yaw_speed  = 0.1;        //1rad/s
    float lift_speed = 1;          //5m/s
    float angular_velocity = 0.1;  //0.02 rad/s
    // route plan 2021.9.5
    float velocity = 0;                //unit m/s
    float linear_velocity     = 0.2;   //unit m/s
    float linear_acceleration = 0;     //unit m/s^2
    float deceleration_time   = 15;    //unit s
    float acceleration_time   = 10;
    int run_times = 0;
    float Time    = 0;  //unit s

    int channel1_up = 1942;
    int channel1_down=1102;
    int channel1_mid =1520;

    int channel2_up = 1107;
    int channel2_down=1939;
    int channel2_mid =1654;

    int channel3_up = 1934;
    int channel3_down=1077;
    int channel3_mid =1492;

    int channel4_up = 1934;
    int channel4_down=1094;
    int channel4_mid =1514;

    double included_angle_last ;
    geometry_msgs::Point pos_sp_last;
    // int mid = 1497;
    // int up = 1932;
    // int down = 1065;

    // int mid2 = 1493;
    // int mid3 = 1430;//thrush mid positon velue
    // int down3=1056;
    // int up3=1932;
public:
    float vel[4];
    float ang_vel[4];//define a vector to store angular velocity
    geometry_msgs::Point pos_setpoint;//setpoint position
    geometry_msgs::Point att_setpoint;//setpoint attitude
    void sw_map_vel(int* sw)
    {
        if((float)(abs(sw[0]-channel1_mid))<=15)
        {
            vel[0]=0;
        }
        else
        {
                vel[0]=((float)(sw[0]-channel1_mid)/(float)(channel1_up-channel1_mid))*fly_speed;//channel 1
            }
        
        if((float)(abs(sw[1]-channel2_mid))<=15)
        {
            vel[1]=0;
        }
        else{
            vel[1]=((float)(sw[1]-channel2_mid)/(float)(channel2_up-channel2_mid))*fly_speed;//channel 2
        }

        if((float)(abs(sw[2]-channel3_mid))<=25)
        {
            vel[2]=0;
        }
        else
        {
                vel[2]=((float)(sw[2]-channel3_mid)/(float)(channel3_up-channel3_mid))*lift_speed;//channel 3
        }
        
        if((float)(abs(sw[3]-channel4_mid))<=15)
        {
            vel[3]=0;
        }
        else{
                vel[3]=-((float)(sw[3]-channel4_mid)/(float)(channel4_up-channel4_mid))*yaw_speed;//channel 4        
        }
	//debug line section 
        // for(int i=0;i<4;i++)
        // {
        //     ROS_INFO_STREAM("sw "<<i<<" : "<<sw[i]);
        //     ROS_INFO_STREAM("vel "<<i<<" : "<<vel[i]);
        // }
    }

    void sw_map_ang_vel(int* sw)
    {
        if((float)(abs(sw[0]-channel1_mid))<=15)
        {
            ang_vel[0]=0;
        }
        else
        {
            ang_vel[0]=((float)(sw[0]-channel1_mid)/(float)(channel1_up-channel1_mid))*angular_velocity;//channel 1
        }
        
        if((float)(abs(sw[1]-channel2_mid))<=15)
        {
            ang_vel[1]=0;
        }
        else{
            ang_vel[1]=((float)(sw[1]-channel2_mid)/(float)(channel2_up-channel2_mid))*angular_velocity;//channel 2
        }

        if((float)(abs(sw[2]-channel3_mid))<=25)
        {
            ang_vel[3]=0;
        }
        else
        {
            ang_vel[3]=((float)(sw[2]-channel3_mid)/(float)(channel3_up-channel3_mid))*lift_speed;//channel 3
        }

        if((float)(abs(sw[3]-channel4_mid))<=15)
        {
            ang_vel[2]=0;
        }
        else
        {
            ang_vel[2]=-((float)(sw[3]-channel4_mid)/(float)(channel4_up-channel4_mid))*yaw_speed;//channel 3
        }
    }
    //calc the setpoint position function
    void calc_setpoint
    (geometry_msgs::Point pos_last, geometry_msgs::Point att_last,float *vel,float t)
    {
        pos_setpoint.x = pos_last.x + vel[1]*t;//this line maybe wrong, kaidi
        pos_setpoint.y = pos_last.y + vel[0]*t;//this line maybe wrong, kaidi 
        // pos_setpoint.x = 2 + vel[1]*t;
        // pos_setpoint.y = 1 + vel[0]*t;
        pos_setpoint.z = pos_last.z + vel[2]*t;
	    att_setpoint.x = att_last.x;
	    att_setpoint.y = att_last.y;
        att_setpoint.z = att_last.z + vel[3]*t;
        //  att_setpoint.z =3.124+1.57+vel[3]*t;
        // ROS_INFO_STREAM("yaw: "<<att_last.z);
    } 
    //calc the setpoint attitude function
    void calc_setpoint_attitude
    (geometry_msgs::Point pos_last, geometry_msgs::Point att_last,float *ang_vel,float t)
    {
        pos_setpoint.x = pos_last.x ;
        pos_setpoint.y = pos_last.y ;
        pos_setpoint.z = pos_last.z + ang_vel[3]*t;
        att_setpoint.x = att_last.x + ang_vel[1]*t;
        att_setpoint.y = att_last.y + ang_vel[0]*t;
        att_setpoint.z = att_last.z + ang_vel[2]*t;
        //ROS_INFO_STREAM("yaw: "<<att_last.z);
    }
    //keep change roll angle 
    void keep_change_roll_pitch_angle
    (geometry_msgs::Point att_last,geometry_msgs::Point pos_last,float *vel,float t)
    {
        
        //roll angle not reach the max 
        if ( att_last.x>=roll_angle_max)
        {
           att_setpoint.x = att_last.x - keep_roll_speed*t;
        }
        else
        {
            att_setpoint.x = att_last.x;
        }

        //pitch angle not reach the max
        if (att_last.y >= -pitch_angle_max)
        {
            att_setpoint.y = att_last.y - keep_pitch_speed*t;
        }
        else
        {
            att_setpoint.y = att_last.y;
        }

        //route plan 
        
        run_times++;
        Time = run_times*t;
        if(Time<20)
        {
            linear_acceleration = linear_velocity/acceleration_time;
            velocity = velocity+linear_acceleration*t;

            pos_setpoint.x = pos_last.x + velocity*t;
            pos_setpoint.y = pos_last.y + 0*t;//this line maybe wrong, kaidi 
            if (velocity>linear_velocity)
            {
                velocity = linear_velocity;
            }
            
        }
        else
        {
            //calc acceleration
            linear_acceleration = linear_velocity/deceleration_time;

            //start to decelerate
            linear_velocity = linear_velocity-linear_acceleration*t;
            pos_setpoint.x = pos_last.x + linear_velocity*t;
            pos_setpoint.y = pos_last.y + 0*t;
            if(linear_velocity<0)
            {
                pos_setpoint.y = pos_last.y;
                pos_setpoint.x = pos_last.x;
            }
        }
        pos_setpoint.z = pos_last.z + vel[2]*t;
    }

    //back roll and pitch to origin
    void back_roll_pitch_to_origin
    (geometry_msgs::Point init_euler_angles,geometry_msgs::Point att_last,float t)
    {
        if ( att_last.x<=init_euler_angles.x)
        {
           att_setpoint.x = att_last.x + (keep_roll_speed)*t;
        }
        else
        {
            att_setpoint.x = att_last.x;
        }

        //pitch angle not reach the max
        if (att_last.y <= init_euler_angles.y)
        {
            att_setpoint.y = att_last.y + (keep_pitch_speed)*t;
        }
        else
        {
            att_setpoint.y = att_last.y;
        }  
    }

    //circle path tracking
    /*
    att_sp         : the attitude of S3Q, it could be a constant or variable
    SoC            : start of circle
    line_speed     : line speed 
    radius         : circle radius
    unit_of_time   : unit of time
    direction_mask : if set to 0, left direction; set to 1, right direction
    */
    void fly_circle
    (geometry_msgs::Point att_sp, geometry_msgs::Point SoC, double line_speed,double radius, double unit_of_time, int direction_mask)
    {
        
        //define start position
        geometry_msgs::Point start_c;
        double included_angle = 0;
        //check which direction
        if(0==direction_mask)
        {
            start_c.x = SoC.x;
            start_c.y = SoC.y-radius;
            start_c.z = SoC.z;
            // calc include angle
            // included_angle = included_angle_last + 2*asin(line_speed*unit_of_time);
            // included_angle_last = included_angle;
            // pos_setpoint.x = start_c.x + radius*cos(included_angle);
            // pos_setpoint.y = start_c.y + radius*sin(included_angle);
            // pos_setpoint.z = start_c.z;
        }
        else
        {
            start_c.x = SoC.x;
            start_c.y = SoC.y+radius;
            start_c.z = SoC.z;
        }
        //set attitude while flying circle path
        att_setpoint.x = att_sp.x ;
        att_setpoint.y = att_sp.y ;
        att_setpoint.z = att_sp.z ;

        //calc include angle
        if(abs(included_angle_last-2*PI)<=0.07)
        {
            included_angle_last = 0;
        }
        included_angle = included_angle_last + 2*asin(line_speed*unit_of_time/(2*radius));
        included_angle_last = included_angle;
        
        //Parametric equation to calc position setpoint
        pos_setpoint.x = start_c.x + radius*cos(included_angle);
        pos_setpoint.y = start_c.y + radius*sin(included_angle);
        pos_setpoint.z = start_c.z;
        ROS_INFO_STREAM("x:"<<pos_setpoint.x);
        ROS_INFO_STREAM("y:"<<pos_setpoint.y);
        ROS_INFO_STREAM("z:"<<pos_setpoint.z);
    }

    //polygon path tracking
    /*
    direction_mask: if set to 0, left direction; if set to 1, right direction
    */
    void fly_Polygon_4
    (geometry_msgs::Point att_sp,  geometry_msgs::Point start_position,double line_distence,double line_speed, double unit_of_time,int direction_mask)
    {
        //define step number
        int step = 0;
        pos_sp_last.x = start_position.x;
        pos_sp_last.y = start_position.y;
        pos_sp_last.z = start_position.z;
        if(0==direction_mask)
        {
            if(step == 0 )
            {
                pos_setpoint.x =  pos_sp_last.x + line_speed*unit_of_time;
                pos_sp_last.x = pos_setpoint.x;
                pos_setpoint.y =  pos_sp_last.y;
                pos_setpoint.z =  pos_sp_last.z;
                if(abs(pos_setpoint.x-start_position.x)<=0.5 && abs(pos_setpoint.y-start_position.y)<=0.05)
                {
                    step=1;
                }
                
            }
            if(step == 1)
            {
                pos_setpoint.x =  pos_sp_last.x;
                pos_setpoint.y =  pos_sp_last.y+ line_speed*unit_of_time;
                pos_sp_last.y = pos_setpoint.y;
                pos_setpoint.z =  pos_sp_last.z;
                if(abs(pos_setpoint.x-start_position.x)<=0.5 && abs(pos_setpoint.y-start_position.y)<=0.5)
                {
                    step=2;
                }
            }
            if(step == 2)
            {
                pos_setpoint.x =  pos_sp_last.x-line_speed*unit_of_time;
                pos_sp_last.x = pos_setpoint.x;
                pos_setpoint.y =  pos_sp_last.y;
                pos_setpoint.z =  pos_sp_last.z;
                if(abs(pos_setpoint.x-start_position.x)<=0.05 && abs(pos_setpoint.y-start_position.y)<=0.5)
                {
                    step=3;
                }
            }
            if(step == 3)
            {
                pos_setpoint.x =  pos_sp_last.x;
                pos_setpoint.y =  pos_sp_last.y- line_speed*unit_of_time;
                pos_sp_last.y = pos_setpoint.y;
                pos_setpoint.z =  pos_sp_last.z;
                if(abs(pos_setpoint.x-start_position.x)<=0.05 && abs(pos_setpoint.y-start_position.y)<=0.05)
                {
                    step=4;
                }
            }
            if(step==4)
            {
                //take down function

            }
            
        }
        else
        {
            
        }
        //set attitude while flying circle path
        att_setpoint.x = att_sp.x ;
        att_setpoint.y = att_sp.y ;
        att_setpoint.z = att_sp.z ;

    }

    //Polar coordinates path tracking
    // void fly_polar_coordinates()
    // {

    // }

    sw_mapping(/* args */);
    ~sw_mapping();
};

sw_mapping::sw_mapping(/* args */)
{
    for(int i=0;i<4;i++)
    {
        vel[i]=0;
    }

    included_angle_last = 0;
    pos_sp_last.x = 0;
    pos_sp_last.y = 0;
    pos_sp_last.z = 0;
    // att_last.x = 0;
    // att_last.y = 0;
    // att_last.z = 0;
}

sw_mapping::~sw_mapping(/* args */)
{
    
}

//function of output log txt file 
//input: setpoint pos , real pos, 
//output: a txt file that record input information.
void get_master_log_txt
(geometry_msgs::Point pos_setpoint, 
    geometry_msgs::Point att_setpoint,
    geometry_msgs::Point main_position,
    geometry_msgs::Point main_eular_angles,std::ofstream &out)
{
    //std::ofstream out;
    //write a timestamp
    out<<"Time is: "<<ros::Time::now()<<std::endl;
    out<<"setpoint position, x:"<<pos_setpoint.x<<" y:"<<pos_setpoint.y<<" z:"<<pos_setpoint.z<<std::endl;
    out<<"real position,     x:"<<main_position.x<<" y:"<<main_position.y<<" z:"<<main_position.z<<std::endl;
    out<<"setpoint attitude, roll:"<<att_setpoint.x<<" pitch:"<<att_setpoint.y<<" yaw:"<<att_setpoint.z<<std::endl;
    out<<"real attitude,     roll:"<<main_eular_angles.x<<" pitch:"<<main_eular_angles.y<<" yaw:"<<main_eular_angles.z<<std::endl;
    //write a enter line as a 
    out<<std::endl;
    //return out;
}

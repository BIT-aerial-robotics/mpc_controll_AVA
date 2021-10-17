//author  : kaidi wang
//date    : 2020.12.20
//descript: master policy decision node 
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>
#include <mavros_msgs/RCIn.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <tf/transform_datatypes.h>
#include <get_master_make_m/get_master_make_m_node.h>
//#include <Eigen/Core>
#include <Eigen/Geometry>

#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>


//global variable list
home_position_pose hps;

//callback fucntion list
//current state callback function
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

//callback function of get_rc data
mavros_msgs::ManualControl get_rc;
void get_rc_setpoint_cb(const mavros_msgs::ManualControl::ConstPtr& msg)
{
    //ROS_INFO("enter the rc callback function....");
    get_rc = *msg;
}

//callback function of get_rc_channel_cb
mavros_msgs::RCIn get_rc_channel;
int sw_arm;
int sw_auto_land;
int sw_kill;
int thrust;
int switch_yaw_xyz[4];//xyz,yaw

//kaidi wang comment this lines on 2021.10.13
/*
change_pitch: geometry controller change pitch angle
change_roll:  geometry controller change roll angle
*/
int change_pitch;//change pitch
int change_roll;//change roll

// define a switch to change controller 
// define a switch var named control_mode_switch, which mapping it to channel 14
// in this software mapping to get_rc_channel.channels[13]
int control_mode_switch;

//define a val to record the switch mode of position and attitude
int switch_position_attitude=0;
void get_rc_channel_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	get_rc_channel = *msg;
	sw_arm = get_rc_channel.channels[6];
	sw_auto_land = get_rc_channel.channels[5];
	sw_kill = get_rc_channel.channels[6];

	//map a channel of RC to the vel, the channel number should be changed
	switch_position_attitude = get_rc_channel.channels[4];
	//get the thrust volume
	thrust = get_rc_channel.channels[2];
	for(int i =0 ; i<4; i++)
	{
		switch_yaw_xyz[i]=get_rc_channel.channels[i];
	}
	//pitch corresponding 13 channel
	change_pitch = get_rc_channel.channels[12];
	//roll corresponding 14 channel
	change_roll  = get_rc_channel.channels[13];

	//kaidi wang 2021.10.13, map the get_rc_channel.channels[13] to control_mode_switch
	control_mode_switch = get_rc_channel.channels[13];

	// ROS_INFO_STREAM("channel 1: "<<get_rc_channel.channels[0]<<" "<<switch_yaw_xyz[0]);
	// ROS_INFO_STREAM("channel 2: "<<get_rc_channel.channels[1]<<" "<<switch_yaw_xyz[1]);
	// ROS_INFO_STREAM("channel 3: "<<get_rc_channel.channels[2]<<" "<<switch_yaw_xyz[2]);
	// ROS_INFO_STREAM("channel 4: "<<get_rc_channel.channels[3]<<" "<<switch_yaw_xyz[3]);
	// ROS_INFO_STREAM("                             ");
	// ROS_INFO_STREAM("channel 7: "<<get_rc_channel.channels[6]);
	// ROS_INFO_STREAM("channel 9: "<<get_rc_channel.channels[8]);
	// ROS_INFO_STREAM("channel 5: "<<get_rc_channel.channels[4]);
	// ROS_INFO_STREAM("change_pitch: "<<change_pitch);
	// ROS_INFO_STREAM("change_roll "<<change_roll);

}

//is_all_connected callback function
std_msgs::Bool is_all_connected;
void is_all_connected_cb(const std_msgs::Bool::ConstPtr& msg)
{
	is_all_connected = *msg;
	ROS_INFO_STREAM("receive info of is_all_connected:"<<is_all_connected);
}

//is_all_armd callback function
std_msgs::Bool is_all_armd;
void is_all_armd_cb(const std_msgs::Bool::ConstPtr& msg)
{
	is_all_armd = *msg;
	ROS_INFO_STREAM("receive info of is_all_armd:"<<is_all_armd);
}

//local pos callback function
geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	local_pos = *msg;
	//ROS_INFO_STREAM("enu pos x: "<<local_pos.pose.position.x);
	//ROS_INFO_STREAM("enu pos y: "<<local_pos.pose.position.y);
	//ROS_INFO_STREAM("enu pos z: "<<local_pos.pose.position.z);
}

//local velocity callback function
geometry_msgs::TwistStamped local_vel;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	local_vel = *msg;
}


//main body rate simulate callback function
geometry_msgs::Point sim_main_body_rates;
void main_body_rates_sim_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sim_main_body_rates = *msg;
}
//main euler angles simulate callback function 
geometry_msgs::Point sim_main_euler_angles;
void main_euler_angles_sim_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sim_main_euler_angles = *msg;
}
//main position simulate callback function 
geometry_msgs::Point sim_main_position;
void main_position_sim_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sim_main_position = *msg;
}
//main velocity simulate callback function 
geometry_msgs::Point sim_main_velocity;
void main_velocity_sim_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	sim_main_velocity = *msg;
}


//node1 yaw callback function
//bias of yaw 
std_msgs::Float64 node1_yaw;
std_msgs::Float64 node2_yaw;
std_msgs::Float64 node3_yaw;

std_msgs::Float64 node1_yaw_pub;
std_msgs::Float64 node2_yaw_pub;
std_msgs::Float64 node3_yaw_pub;
void node_yaw_1_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node1_yaw = *msg;
	//ROS_INFO_STREAM("node1_yaw: "<<node1_yaw);
}

void node_yaw_2_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node2_yaw = *msg;
	//ROS_INFO_STREAM("node2_yaw: "<<node2_yaw);
}

void node_yaw_3_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node3_yaw = *msg;
	//ROS_INFO_STREAM("node3_yaw: "<<node3_yaw);
}


/*****    customize function define list    *****/
//trans PX4 attitude to controller input, position
geometry_msgs::Point make_main_position(geometry_msgs::PoseStamped local_pos)
{
	geometry_msgs::Point main_pos;
	main_pos.x = local_pos.pose.position.x;
	main_pos.y = local_pos.pose.position.y;
	main_pos.z = local_pos.pose.position.z;
	return main_pos;
}

//tran PX4 attitude to controller input, eular angles
geometry_msgs::Point make_eular_angles(geometry_msgs::PoseStamped local_pos)
{
	Eigen::Quaterniond q;
	q.x()=local_pos.pose.orientation.x;
	q.y()=local_pos.pose.orientation.y;
	q.z()=local_pos.pose.orientation.z;
	q.w()=local_pos.pose.orientation.w;

	q.normalize();
	geometry_msgs::Quaternion q_geo;
	q_geo.x = q.x();
	q_geo.y = q.y();
	q_geo.z = q.z();
	q_geo.w = q.w();


	geometry_msgs::Point main_eul_ang;
	double roll, pitch, yaw;
	tf::Quaternion quat;
	tf::quaternionMsgToTF(q_geo, quat);
	//mavros send the euler of xyz order
	tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

	//tf::Matrix3x3(quat).getEulerZYX(yaw, pitch, roll);

	main_eul_ang.x=roll;
	main_eul_ang.y=pitch;
	main_eul_ang.z=yaw;

	//main_eul_ang.x=roll;
	//main_eul_ang.y=pitch;
	//main_eul_ang.z=yaw;

	return main_eul_ang;
} 


//tran PX4 attitude to controller input, velocity
geometry_msgs::Point make_main_velocity(geometry_msgs::TwistStamped local_vel)
{
	geometry_msgs::Point main_velocity;
	main_velocity.x = -local_vel.twist.linear.y;
	main_velocity.y = local_vel.twist.linear.x;
	main_velocity.z = local_vel.twist.linear.z;
	//ROS_INFO_STREAM("main_velocity_x_ned_local: "<< main_velocity.x);
	//ROS_INFO_STREAM("main_velocity_y_ned_local: "<< main_velocity.y);
	//ROS_INFO_STREAM("main_velocity_z_ned_local: "<< main_velocity.z);

	return main_velocity;
}


//tran PX4 attitude to controller input, body_rates
geometry_msgs::Point make_main_body_rates(geometry_msgs::TwistStamped local_vel)
{
	geometry_msgs::Point main_body_rates;
	main_body_rates.x = -local_vel.twist.angular.y;
	main_body_rates.y = local_vel.twist.angular.x;
	main_body_rates.z = local_vel.twist.angular.z;

	//ROS_INFO_STREAM("main_body_rates_x_ned_local: "<< main_body_rates.x);
	//ROS_INFO_STREAM("main_body_rates_y_ned_local: "<< main_body_rates.y);
	//ROS_INFO_STREAM("main_body_rates_z_ned_local: "<< main_body_rates.z);

	return main_body_rates;
}

//main function 
int main(int argc, char **argv)
{
	ros::init(argc, argv, "get_master_make_m");
	ros::NodeHandle nh;

	/**subscribe list**/
	ros::Subscriber state_sub     = nh.subscribe<mavros_msgs::State>
		("mavros/state", 10, state_cb);
	//subscribe manual control
    ros::Subscriber get_rc_sub = nh.subscribe<mavros_msgs::ManualControl>
		("/mavros/manual_control/control",100,get_rc_setpoint_cb);
	//subacribe RC channels value
	ros::Subscriber get_rc_channel_sub = nh.subscribe<mavros_msgs::RCIn>
		("/mavros/rc/in",100,get_rc_channel_cb);
	//subscribe if all child nodes have already connected to PX4
	ros::Subscriber is_all_connected_sub = nh.subscribe<std_msgs::Bool>
		("is_all_connected",10,is_all_connected_cb);
	//subscribe if all child nodes have already armd
	ros::Subscriber is_all_armd_sub = nh.subscribe<std_msgs::Bool>
		("is_all_armd",10,is_all_armd_cb);
	//subscribe local position
	//kd edit it for vicon indoor
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
	//ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose",10,local_pos_cb);
	//subscriber local_velocity , NWU frame of body rate
	ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("mavros/local_position/velocity_body",10,local_vel_cb);
	
	//simulation of the ros node message 
	ros::Subscriber main_body_rates_sim_sub = nh.subscribe<geometry_msgs::Point>
		("/main_body_rates_sim",100,main_body_rates_sim_sub_cb);
	ros::Subscriber main_euler_angles_sim_sub = nh.subscribe<geometry_msgs::Point>
		("/main_euler_angles_sim",100,main_euler_angles_sim_sub_cb);
	ros::Subscriber main_position_sim_sub = nh.subscribe<geometry_msgs::Point>
		("/main_position_sim",100,main_position_sim_sub_cb);
	ros::Subscriber main_velocity_sim_sub = nh.subscribe<geometry_msgs::Point>
		("/main_velocity_sim",100,main_velocity_sim_sub_cb);

	ros::Subscriber node_yaw_1_sub = nh.subscribe<std_msgs::Float64>
		("node1_yaw",10,node_yaw_1_sub_cb);
	ros::Subscriber node_yaw_2_sub = nh.subscribe<std_msgs::Float64>
		("node2_yaw",10,node_yaw_2_sub_cb);
	ros::Subscriber node_yaw_3_sub = nh.subscribe<std_msgs::Float64>
		("node3_yaw",10,node_yaw_3_sub_cb);

	/**publish list**/
	ros::Publisher local_pos_pub  = nh.advertise<geometry_msgs::PoseStamped>
		("mavros/setpoint_position/local", 10);
	//advertise a bool param, named child/arming...
	ros::Publisher arm_child_flight    = nh.advertise<std_msgs::Bool>("child/arming",10);
	//child/autoland
	ros::Publisher takeoff_pub = nh.advertise<std_msgs::Bool>("child/takeoff",10);
	//child/kill
	ros::Publisher normal_pub = nh.advertise<std_msgs::Bool>("child/normal",10);

	//nominal_position
	ros::Publisher nominal_position_pub = nh.advertise<geometry_msgs::Point>("/nominal_position",100);
	//nominal_eular_angles
	ros::Publisher nominal_eular_angles_pub = nh.advertise<geometry_msgs::Point>("/nominal_euler_angles",100);
	//controller input of main_position
	ros::Publisher main_position_pub = nh.advertise<geometry_msgs::Point>("/main_position",100);
	//controller input of main_velocity
	ros::Publisher main_velocity_pub = nh.advertise<geometry_msgs::Point>("/main_velocity",100);
	//controller input of main_eular_angles
	ros::Publisher main_eular_angles_pub = nh.advertise<geometry_msgs::Point>("/main_euler_angles",100);
	//controller input of main_body_rates
	ros::Publisher main_body_rates_pub = nh.advertise<geometry_msgs::Point>("/main_body_rates",100);
	
	//init cmd publish: euler angles
	ros::Publisher init_euler_angles_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_euler_angles_cmd",100);
	//init cmd publish: position
	ros::Publisher init_pos_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_position_cmd",100);
	//init cmd publish: body_rates
	ros::Publisher init_body_rates_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_body_rates_cmd",100);
	//init cmd publish: velocity
	ros::Publisher init_velocity_cmd_pub = nh.advertise<geometry_msgs::Point>("/init_velocity_cmd",100);
	
	//control switch of when publish attitude to child node
	ros::Publisher control_start_pub_att = nh.advertise<std_msgs::Bool>("start_pub_att",100);
	ros::Publisher psi_bias_1_pub=nh.advertise<std_msgs::Float64>("/psi_bias_1",10);
	ros::Publisher psi_bias_2_pub=nh.advertise<std_msgs::Float64>("/psi_bias_2",10);
	ros::Publisher psi_bias_3_pub=nh.advertise<std_msgs::Float64>("/psi_bias_3",10);

	//contorl mode switch ros msg define, which need to publish to other nodes
	ros::Publisher control_mode_switch_pub = nh.advertise<std_msgs::Bool>("/mode_switch",10);

	/**sercice list**/
	ros::ServiceClient arming_client   = nh.serviceClient<mavros_msgs::CommandBool>
		("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");

	//define some variables for publish
	geometry_msgs::Point nominal_positon;//setpoint position
	geometry_msgs::Point nominal_euler_angles;//euler angle setpoint position
	//define setpoint last variables for integral last setpoint
	geometry_msgs::Point nominal_position_last;//
	geometry_msgs::Point nominal_euler_angles_last;	

	geometry_msgs::Point main_position;
	geometry_msgs::Point main_velocity;
	geometry_msgs::Point main_eular_angles;
	geometry_msgs::Point main_body_rates;

	//define init switch setpoint msg
	geometry_msgs::Point init_euler_angles;
	geometry_msgs::Point init_position;
	geometry_msgs::Point init_body_rate;
	geometry_msgs::Point init_velocity;


	//define message publish to pose_controller
	geometry_msgs::Point nominal_pos_ned;
	geometry_msgs::Point nominal_eular_angles_ned;
	geometry_msgs::Point main_pos_ned;
	geometry_msgs::Point main_velocity_ned;
	geometry_msgs::Point main_eular_angles_ned;
	geometry_msgs::Point main_body_rates_ned;

	//define a var to trans euler
	geometry_msgs::PoseStamped quat2euler;

	//test line,open the armd section
	is_all_armd.data=true;

	//wait the px4 flight to be connected 
	ros::Rate rate(100.0);
	//ROS_INFO("current_state_connected is %d",current_state.connected);
	while (ros::ok() && !current_state.connected && !is_all_connected.data) {
		ROS_INFO("px4 is unconnected...");
		ros::spinOnce();
		rate.sleep();
	}
	
	while (ros::ok() && !get_home_position)
	{
		/* Continuous acquisition 1000 times as the original position*/
		for (int i = 0; i < 500; i++)
		{
			hps.pos.x = local_pos.pose.position.x;
			hps.pos.y = local_pos.pose.position.y;
			hps.pos.z = local_pos.pose.position.z;
			
			//ROS_INFO_STREAM("home position,x:"<<hps.pos.x<<" y:"<<hps.pos.y<<" z:"<<hps.pos.z);
			hps.quaternion.x = local_pos.pose.orientation.x;
			hps.quaternion.y = local_pos.pose.orientation.y;
			hps.quaternion.z = local_pos.pose.orientation.z;
			hps.quaternion.w = local_pos.pose.orientation.w;

			hps.linear_velocity.x=local_vel.twist.linear.x;
			hps.linear_velocity.y=local_vel.twist.linear.y;
			hps.linear_velocity.z=local_vel.twist.linear.z;

			hps.angular_velocity.x=local_vel.twist.angular.x;
			hps.angular_velocity.y=local_vel.twist.angular.y;
			hps.angular_velocity.z=local_vel.twist.angular.z;

			quat2euler.pose.orientation.x=hps.quaternion.x;
			quat2euler.pose.orientation.y=hps.quaternion.y;
			quat2euler.pose.orientation.z=hps.quaternion.z;
			quat2euler.pose.orientation.w=hps.quaternion.w;
			
			//set the home_position as the integral value

			ros::spinOnce();
			rate.sleep();
		}
		//assign the home position, home euler angle 
		get_home_position = 1;//
		break;	
	}
	
	//define set_mode 
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	//define arm_cmd
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value=true;

	//define time::now()
	ros::Time last_request = ros::Time::now();

	//define arm_bool param
	std_msgs::Bool arm_bool;
	arm_bool.data =false;//init arm_bool to false to ensure safety

	std_msgs::Bool autoland_bool;
	autoland_bool.data = false;

	std_msgs::Bool normal_bool;
	normal_bool.data = false;

	//define the msg of when start to publish attitude
	std_msgs::Bool start_pub_att;
	std_msgs::Bool mode_switch;

	//define sw class
	sw_mapping sw;
	int thrust_last_time=0;
	int count = 0;
	bool let_fly = false;//bool fly switch
	bool init_controller = false; //bool init controller

	//detect the RC command.
	//set offboard mode and then arm
	while(ros::ok())
	{
		//let_fly = true;
		//arm switch
		if(sw_auto_land < 1500 && sw_arm > 1500)
		{
			//ROS_INFO_STREAM("arm");
			arm_bool.data=true;
			autoland_bool.data = false;
			normal_bool.data = false;
		}
		else if((sw_auto_land > 1500 && sw_arm > 1500)||(sw_auto_land > 1500 && sw_arm < 1500))
		{
			//take off switch
			//ROS_INFO_STREAM("takeoff");
			arm_bool.data=false;
			autoland_bool.data = true;
			normal_bool.data = false;

		}
		else if(sw_auto_land < 1500 && sw_arm < 1500)
		{
			//normal crc switch
			//ROS_INFO_STREAM("normal");
			arm_bool.data=false;
			autoland_bool.data = false;
			normal_bool.data = true;
		}


		//judge if the thrust is up to middle
		if(arm_bool.data==true && thrust_last_time <= MID && thrust >= MID)
		{
			ROS_INFO_STREAM("armed and the uav will fly..");
			let_fly = true;//start the controller and the vehicle will fly right now
			init_controller = true;
			count ++ ;
			
		}
		else if (arm_bool.data==false)
		{
			//ROS_INFO_STREAM("not arm..");
			let_fly = false;
			init_controller = false;
			count = 0;
		}

		//init main,cmd,and setpoint
		if(init_controller && count == 1)
		{	
			//	system("./record.sh");
			//close the init controller
			init_controller = false;
			ROS_INFO_STREAM("init controller ....");

			main_position = make_main_position(local_pos);
			main_pos_ned  = enu2ned_pos(main_position);
			
			main_eular_angles = make_eular_angles(local_pos);
			main_eular_angles_ned = enu2ned_euler(main_eular_angles);

			main_velocity = make_main_velocity(local_vel);
			main_velocity_ned = enu2ned_vel(main_velocity);

			main_body_rates=make_main_body_rates(local_vel);
			main_body_rates_ned = enu2ned_euler_rate(main_body_rates);


			nominal_position_last.x = main_position.x;
			nominal_position_last.y = main_position.y;
			nominal_position_last.z = main_position.z;
			nominal_euler_angles_last.x = main_eular_angles.x;
			nominal_euler_angles_last.y = main_eular_angles.y;
			nominal_euler_angles_last.z = main_eular_angles.z;
			//after 100 times get positon, I think the vehicle is ready to fly
			//set the get_home_position to 1

			init_body_rate.x=0;
			init_body_rate.y=0;
			init_body_rate.z=0;

			init_euler_angles.x=main_eular_angles_ned.x;
			init_euler_angles.y=main_eular_angles_ned.y;
			//kaidi wang change on 2021.9.11
			//init_euler_angles.z=main_eular_angles_ned.z;
			init_euler_angles.z=0;
			//init_euler_angles = enu2ned_euler(init_euler_angles);

			init_velocity.x=0;
			init_velocity.y=0;
			init_velocity.z=0;
		
			init_position.x=main_position.y;
			init_position.y=main_position.x;
			init_position.z=-main_position.z;

			//init setpoint to controller
			nominal_position_last = enu2ned_pos(nominal_position_last);
			nominal_euler_angles_last = enu2ned_euler(nominal_euler_angles_last);

			nominal_position_pub.publish(nominal_position_last);
			nominal_eular_angles_pub.publish(nominal_euler_angles_last);

			//init cmd to controller
			init_euler_angles_cmd_pub.publish(init_euler_angles);
			init_velocity_cmd_pub.publish(init_velocity);
			init_pos_cmd_pub.publish(init_position);
			init_body_rates_cmd_pub.publish(init_body_rate);


			nominal_position_last.x = main_position.x;
			nominal_position_last.y = main_position.y;
			nominal_position_last.z = main_position.z;
			nominal_euler_angles_last.x=main_eular_angles.x;
			nominal_euler_angles_last.y=main_eular_angles.y;
			nominal_euler_angles_last.z=main_eular_angles.z;
			//main_position = make_main_position(local_pos);

			//node1_yaw_pub.data=node1_yaw.data-main_eular_angles.z;
			//node2_yaw_pub.data=node2_yaw.data-main_eular_angles.z;
			//node3_yaw_pub.data=node3_yaw.data-main_eular_angles.z;	
			
			//ROS_INFO_STREAM("yaw1: "<<node1_yaw_pub.data);
			//ROS_INFO_STREAM("yaw2: "<<node2_yaw_pub.data);
			//ROS_INFO_STREAM("yaw3: "<<node3_yaw_pub.data);
			node1_yaw.data=0;
			node2_yaw.data=2.09433;//1.0471
			node3_yaw.data=-2.09433;//1.0461
			
			psi_bias_1_pub.publish(node1_yaw);
			psi_bias_2_pub.publish(node2_yaw);
			psi_bias_3_pub.publish(node3_yaw);

			main_position_pub.publish(main_pos_ned);
			main_eular_angles_pub.publish(main_eular_angles_ned);
			main_velocity_pub.publish(main_velocity_ned);
			main_body_rates_pub.publish(main_body_rates_ned);
		}


		// kaidi wang, 2021.10.13. change control mode
		if(control_mode_switch<1500)
		{
			mode_switch.data = false;
			//ROS_INFO_STREAM("control_mode is ordinary...");
		}
		else
		{
			mode_switch.data = true;
			//ROS_INFO_STREAM("control_mode is MPC...");
		}

		//normol function
		if(let_fly)
		{
			//get the RC control map mode
			if(switch_position_attitude <1500 && change_roll<1500)//position_mode
			{	
				ROS_INFO_STREAM("positon map mode ");
				sw.sw_map_vel(switch_yaw_xyz);
				sw.calc_setpoint(nominal_position_last,nominal_euler_angles_last,sw.vel,0.01);
				nominal_position_last = sw.pos_setpoint;
				nominal_euler_angles_last = sw.att_setpoint;
			}
			else if(switch_position_attitude >1500 && change_roll<1500)//attitude_mode
			{
				ROS_INFO_STREAM("attitude map mode ");
				sw.sw_map_ang_vel(switch_yaw_xyz);
				sw.calc_setpoint_attitude(nominal_position_last,nominal_euler_angles_last,sw.ang_vel,0.01);
				nominal_position_last = sw.pos_setpoint;
				nominal_euler_angles_last = sw.att_setpoint;
			}

			// kaidi wang, 2021.10.13. this section is used to test attitude and position change at the same time
			// start change roll angle and pitch angle function block
			// if(change_roll>1500)
			// {
			// 	sw.sw_map_vel(switch_yaw_xyz);
			// 	sw.keep_change_roll_pitch_angle(nominal_euler_angles_last,nominal_position_last,sw.vel,0.01);
			// 	//nominal_position_last = sw.pos_setpoint;
			// 	nominal_euler_angles_last = sw.att_setpoint;
			// 	nominal_position_last = sw.pos_setpoint;
			// }
			// if(change_pitch>1500)
			// {
			// 	sw.back_roll_pitch_to_origin(init_euler_angles,nominal_euler_angles_last,0.01);
			// 	//nominal_position_last = sw.pos_setpoint;
			// 	nominal_euler_angles_last = sw.att_setpoint;
			// }

			// kaidi wang, 2021.10.13, comments. back roll angle and pitch angle to origin
			// ROS_INFO_STREAM("yaw: "<<init_euler_angles.z);
			
			//decete if all child nodes are armd
			if(is_all_armd.data)
			{
				//ROS_INFO_STREAM("forward and backward: "<<sw.pos_setpoint.x);
				//ROS_INFO_STREAM("lefe and right: "<<sw.pos_setpoint.y);
				//ROS_INFO_STREAM("up and down: "<<sw.pos_setpoint.z);
				//ROS_INFO_STREAM("yaw: "<<sw.att_setpoint.z);
				//ROS_INFO_STREAM("roll: "<<sw.att_setpoint.x);
				//ROS_INFO_STREAM("pitch: "<<sw.att_setpoint.y);
				//ROS_INFO_STREAM("yaw: "<<sw.att_setpoint.z);

				//publish arm command to child nodes 
				//trans the enu ot ned sueset
				nominal_pos_ned = enu2ned_pos(sw.pos_setpoint);
				nominal_eular_angles_ned = enu2ned_euler(sw.att_setpoint);
				nominal_position_pub.publish(nominal_pos_ned);
				nominal_eular_angles_pub.publish(nominal_eular_angles_ned);
				// ROS_INFO_STREAM("forward and backward: "<<nominal_pos_ned.x);
				// ROS_INFO_STREAM("lefe and right: "<<nominal_pos_ned.y);
				// ROS_INFO_STREAM("up and down: "<<nominal_pos_ned.z);
				// ROS_INFO_STREAM("yaw: "<<nominal_eular_angles_ned.z);

				//publish the real position,velocity,eular angles,eular rates
				main_position = make_main_position(local_pos);
				main_pos_ned  = enu2ned_pos(main_position);
				//main_position_pub.publish(main_pos_ned);
				
				main_eular_angles = make_eular_angles(local_pos);
				main_eular_angles_ned = enu2ned_euler(main_eular_angles);
			    //main_eular_angles_pub.publish(main_eular_angles_ned);

				main_velocity = make_main_velocity(local_vel);
				main_velocity_ned = enu2ned_vel(main_velocity);
			  	//main_velocity_pub.publish(main_velocity_ned);

				main_body_rates=make_main_body_rates(local_vel);
				main_body_rates_ned = enu2ned_euler_rate(main_body_rates);
				//main_body_rates_pub.publish(main_body_rates_ned);

				//ROS_INFO_STREAM("eular angles x: "<<main_eular_angles.x);
				//ROS_INFO_STREAM("eular angles y: "<<main_eular_angles.y);
				//ROS_INFO_STREAM("eular angles z: "<<main_eular_angles.z);
				
				//ROS_INFO_STREAM("main_velocity_ned x: "<<main_velocity_ned.x);
				//ROS_INFO_STREAM("main_velocity_ned y: "<<main_velocity_ned.y);
				//ROS_INFO_STREAM("main_velocity_ned z: "<<main_velocity_ned.z);

				//ROS_INFO_STREAM("yaw1: "<<node1_yaw.data);
				//ROS_INFO_STREAM("yaw2: "<<node2_yaw.data);
				//ROS_INFO_STREAM("yaw3: "<<node3_yaw.data);
				//ROS_INFO_STREAM("yaw1: "<<node1_yaw_pub.data);
				//ROS_INFO_STREAM("yaw2: "<<node2_yaw_pub.data);
				//ROS_INFO_STREAM("yaw3: "<<node3_yaw_pub.data);
				//publish messages to simulate ros node, plant
				/**
				 * geometry_msgs::Point sim_main_body_rates;
				 * geometry_msgs::Point sim_main_euler_angles;
				 * geometry_msgs::Point sim_main_position;
				 * geometry_msgs::Point sim_main_velocity;
				 ***/

		 		 main_body_rates_pub.publish(sim_main_body_rates);
				 main_eular_angles_pub.publish(sim_main_euler_angles);
				 main_position_pub.publish(sim_main_position);
				 main_velocity_pub.publish(sim_main_velocity);
				 ROS_INFO_STREAM("set x: "<<sw.pos_setpoint.x<<" simu_x: "<<sim_main_position.y);
				 ROS_INFO_STREAM("set y: "<<sw.pos_setpoint.y<<" sim_y: "<<sim_main_position.x);
				 ROS_INFO_STREAM("set z: "<<sw.pos_setpoint.z<<" sim_z: "<<-sim_main_position.z);
				 ROS_INFO_STREAM("set roll: "<<sw.att_setpoint.x<<" sim_roll: "<<-sim_main_euler_angles.y);
				 ROS_INFO_STREAM("set pitch: "<<sw.att_setpoint.y<<" sim_pitch: "<<-sim_main_euler_angles.x);
				 ROS_INFO_STREAM("set yaw: "<<sw.att_setpoint.z<<" sim_yaw: "<<-sim_main_euler_angles.z+1.5708);

			}
			else
			{
				/*list which child node is not armd*/
				//ROS_INFO_STREAM("not armd node id : ");
			}
		}
		else
		{
			/* code */
		}
		start_pub_att.data = let_fly;
		control_start_pub_att.publish(start_pub_att);
		//publish mode switch msg, 10.13
		control_mode_switch_pub.publish(mode_switch);

		arm_child_flight.publish(arm_bool);
		takeoff_pub.publish(autoland_bool);
		normal_pub.publish(normal_bool);
		//storage the current thrust as the lastest volume
		thrust_last_time = thrust;

		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}

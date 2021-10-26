//kaidi wang, 2021.9.14
//mpc controller of AVA
#include <mpc_controller/mpc.h>
/* Some convenient definitions. */
//using namespace casadi;
using Eigen::MatrixXd;

//construct function list
mpc::mpc(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    J(0,0)=param.I_sys.x;
	J(1,1)=param.I_sys.y;
	J(2,2)=param.I_sys.z;
	J(0,1)=J(0,2)=J(1,0)=J(1,2)=J(2,0)=J(2,1)=0;
	pos_s1=pos_mat(param.fly1_pos.x,param.fly1_pos.y,param.fly1_pos.z);
	pos_s2=pos_mat(param.fly2_pos.x,param.fly2_pos.y,param.fly2_pos.z);
	pos_s3=pos_mat(param.fly3_pos.x,param.fly3_pos.y,param.fly3_pos.z);
    init_subscriber();// init subscriber list handle
	init_publisher(); // init publisher list handle

	//init nominal_position and nominal_euler_angles
	nominal_position.x = init_position.x;
	nominal_position.y = init_position.y;
	nominal_position.z = init_position.z;
	nominal_euler_angles.x = init_euler_angles.x;
	nominal_euler_angles.y = init_euler_angles.y;
	nominal_euler_angles.z = init_euler_angles.z;


	//position bound
	u_pos_x=100;
	l_pos_x=-100;

	u_pos_y=100;
	l_pos_y=-100;

	u_pos_z=100;
	l_pos_z=-100; 
    //attitude bound
	u_ang_x=PI/3;
	l_ang_x=-PI/3;

	u_ang_y=PI/3;
	l_ang_y=-PI/3;
	
	u_ang_z=PI;
	l_ang_z=-PI;
    //velocity bound
	u_vel_x=2;
	l_vel_x=-2;

	u_vel_y=2;
	l_vel_y=-2;

	u_vel_z=2;
	l_vel_z=-2;
    //ang velocity bound
	u_ang_vel_x=0.5;
	l_ang_vel_x=-0.5;

	u_ang_vel_y=0.5;
	l_ang_vel_y=-0.5;
	
	u_ang_vel_z=0.5;
	l_ang_vel_z=-0.5;
    //thrust and torque bound
	u_thrust_x=5;
	l_thrust_x=-5;

	u_thrust_y=5;
	l_thrust_y=-5;

	u_thrust_z=65;
	l_thrust_z=-65;

	u_torque_x=1;
	l_torque_x=-1;

	u_torque_y=1;
	l_torque_y=-1;

	u_torque_z=1;
	l_torque_z=-1;

	//param of first-order param
	a_thu_x=0.1; 
    a_thu_y=0.1; 
    a_thu_z=0.1; 
    a_tor_x=0.1; 
    a_tor_y=0.1; 
    a_tor_z=0.1;
	//thrust and torque init
	for (int i = 0; i < 3; i++)
	{
		in_thrust[i]=0;
		in_torque[i]=0;
	}


	//kaidi wang need to generate the acado library called once mpc state function
	ROS_INFO_STREAM("Whether the control issue needs to be regenerated:Y/n");
	std::string str;
	std::getline(std::cin,str);
	if (str == "Y")
	{
		ROS_INFO_STREAM("generate new control issue.");
		mpc_state_function();
		ros::shutdown();// close the ros node
	}
	else
	{
		ROS_INFO_STREAM("don't generate new control issue and go ahead.");
	}
	//init mark with 0
	mark.data = 0;
	init_fun_switch = 0;
    calc_timer = nh.createTimer(ros::Duration(T), &mpc::calc_cb, this);
}

//publish function, all publish is in NED frame
void mpc::init_publisher()
{
    //init this publisher pub, and then i can use these handle in every private member function of this class
	angle1_pub = nh.advertise<geometry_msgs::Point>("/angle1",10,this);
	angle2_pub = nh.advertise<geometry_msgs::Point>("/angle2",10,this);
	angle3_pub = nh.advertise<geometry_msgs::Point>("/angle3",10,this);

	thrust1_pub = nh.advertise<std_msgs::Float64>("/thrust1",10,this);
	thrust2_pub = nh.advertise<std_msgs::Float64>("/thrust2",10,this);
	thrust3_pub = nh.advertise<std_msgs::Float64>("/thrust3",10,this);

	//publish the mpc controller running mark
	mpc_mark_pub = nh.advertise<std_msgs::Float64>("/mark",10,this);
}


//subscriber function
void mpc::init_subscriber()
{
    nominal_position_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_position",1,&mpc::nominal_position_sub_cb,this);
	nominal_eular_angles_sub = nh.subscribe<geometry_msgs::Point>
	("/nominal_euler_angles",1,&mpc::nominal_eular_angles_sub_cb, this);
	main_position_sub = nh.subscribe<geometry_msgs::Point>
	("/main_position",1,&mpc::main_position_sub_cb,this);
	main_velocity_sub = nh.subscribe<geometry_msgs::Point>
	("/main_velocity",1,&mpc::main_velocity_sub_cb,this);
	main_eular_angles_sub = nh.subscribe<geometry_msgs::Point>
	("/main_euler_angles",1,&mpc::main_eular_angles_sub_cb,this);
	main_body_rates_sub = nh.subscribe<geometry_msgs::Point>
	("/main_body_rates",1,&mpc::main_body_rates_sub_cb,this);
	init_euler_angles_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_euler_angles_cmd",1,&mpc::init_euler_angles_cmd_sub_cb,this);
	init_pos_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_position_cmd",1,&mpc::init_pos_cmd_sub_cb,this);
	init_body_rates_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_body_rates_cmd",1,&mpc::init_body_rates_cmd_sub_cb,this);
	init_velocity_cmd_sub = nh.subscribe<geometry_msgs::Point>
	("/init_velocity_cmd",1,&mpc::init_velocity_cmd_sub_cb,this);


	control_start_sub_att = nh.subscribe<std_msgs::Bool>
	("/start_pub_att",1,&mpc::control_start_sub_att_cb,this);
	//kaidi wang, 2021.10.13, mode_switch_sub define
	mode_switch_sub       = nh.subscribe<std_msgs::Bool>
	("/mode_switch",1,&mpc::mode_switch_sub_cb,this);
	
	psi_bias_1_sub = nh.subscribe<std_msgs::Float64>("/psi_bias_1",1,&mpc::psi_bias_1_sub_cb,this);
	psi_bias_2_sub = nh.subscribe<std_msgs::Float64>("/psi_bias_2",1,&mpc::psi_bias_2_sub_cb,this);
	psi_bias_3_sub = nh.subscribe<std_msgs::Float64>("/psi_bias_3",1,&mpc::psi_bias_3_sub_cb,this);
}

//generate pos matrix
Eigen::Matrix3f mpc::pos_mat(double x,double y,double z)
{
	Eigen::Matrix3f s;
	s(0,0)=s(1,1)=s(2,2)=0;
	s(0,1)=-z;
	s(0,2)= y;
	s(1,0)= z;
	s(1,2)=-x;
	s(2,0)=-y;
	s(2,1)= x;
	return s;
}

//the state function of the mpc controller
void mpc::mpc_state_function()
{
	//only be called once at the construction function
	pos[0]=pos_x;
	pos[1]=pos_y;
	pos[2]=pos_z;
	
	att[0]=ang_roll;
	att[1]=ang_pitch;
	att[2]=ang_yaw;

	pos_vel[0]=pos_vel_x;
	pos_vel[1]=pos_vel_y;
	pos_vel[2]=pos_vel_z;

	ang_vel[0]=ang_vel_roll;
	ang_vel[1]=ang_vel_pitch;
	ang_vel[2]=ang_vel_yaw;

	//control thrust
	force[0]=u_thu_x;
	force[1]=u_thu_y;
	force[2]=u_thu_z;
	tao[0]=u_tor_x;
	tao[1]=u_tor_y;
	tao[2]=u_tor_z;
	//define the rotation matrix
	rotation[0][0]=(cos(ang_pitch) * cos(ang_yaw));
	rotation[0][1]=(sin(ang_roll) * sin(ang_pitch) * cos(ang_yaw) - cos(ang_roll) * sin(ang_yaw));
	rotation[0][2]=(cos(ang_roll) * sin(ang_pitch) * cos(ang_yaw) + sin(ang_roll) * sin(ang_yaw));
	rotation[1][0]=(cos(ang_pitch) * sin(ang_yaw));
	rotation[1][1]=(sin(ang_roll) * sin(ang_pitch) * sin(ang_yaw) + cos(ang_roll) * cos(ang_yaw));
	rotation[1][2]=(cos(ang_roll) * sin(ang_pitch) * sin(ang_yaw) - sin(ang_roll) * cos(ang_yaw));
	rotation[2][0]=(-sin(ang_pitch));
	rotation[2][1]=(sin(ang_roll) * cos(ang_pitch));
	rotation[2][2]=(cos(ang_roll) * cos(ang_pitch));
	//inter param
	MtMangv[0]=J(0,0)*ang_vel[0];
	MtMangv[1]=J(1,1)*ang_vel[1];
	MtMangv[2]=J(2,2)*ang_vel[2];
	//get the map matrix of MtMangv
	mapMtMangv[0][0]=0;
	mapMtMangv[0][1]=-MtMangv[2];
	mapMtMangv[0][2]=MtMangv[1];
	mapMtMangv[1][0]=MtMangv[2];
	mapMtMangv[1][1]=0;
	mapMtMangv[1][2]=-MtMangv[0];
	mapMtMangv[2][0]=-MtMangv[1];
	mapMtMangv[2][1]=MtMangv[0];
	mapMtMangv[2][2]=0;
	//map matrix multiple ang_vel
	mapMang_v[0]=mapMtMangv[0][0]*ang_vel[0]+mapMtMangv[0][1]*ang_vel[1]+mapMtMangv[0][2]*ang_vel[2];
	mapMang_v[1]=mapMtMangv[1][0]*ang_vel[0]+mapMtMangv[1][1]*ang_vel[1]+mapMtMangv[1][2]*ang_vel[2];
	mapMang_v[2]=mapMtMangv[2][0]*ang_vel[0]+mapMtMangv[2][1]*ang_vel[1]+mapMtMangv[2][2]*ang_vel[2];
	
	//calc the acc vector
	acc[0]=(1/param.S3Q_mass)*(rotation[0][0]*force[0]+rotation[0][1]*force[1]+rotation[0][2]*force[2]);
	acc[1]=(1/param.S3Q_mass)*(rotation[1][0]*force[0]+rotation[1][1]*force[1]+rotation[1][2]*force[2]);
	acc[2]=(1/param.S3Q_mass)*(rotation[2][0]*force[0]+rotation[2][1]*force[1]+rotation[2][2]*force[2])-G;
	//calc the ang acc vector 
	ang_acc[0]=(J.inverse())(0,0)*(tao[0]+mapMang_v[0])+(J.inverse())(0,1)*(tao[1]+mapMang_v[1])+(J.inverse())(0,2)*(tao[2]+mapMang_v[2]);
	ang_acc[1]=(J.inverse())(1,0)*(tao[0]+mapMang_v[0])+(J.inverse())(1,1)*(tao[1]+mapMang_v[1])+(J.inverse())(1,2)*(tao[2]+mapMang_v[2]);
	ang_acc[2]=(J.inverse())(2,0)*(tao[0]+mapMang_v[0])+(J.inverse())(2,1)*(tao[1]+mapMang_v[1])+(J.inverse())(2,2)*(tao[2]+mapMang_v[2]);
	//Differential Equation f
	f << dot(pos_x) == pos_vel[0];
	f << dot(pos_y) == pos_vel[1];
	f << dot(pos_z) == pos_vel[2];
	f << dot(ang_roll)  == ang_vel[0];//angle vel
	f << dot(ang_pitch) == ang_vel[1];
	f << dot(ang_yaw)   == ang_vel[2];
	f << dot(pos_vel_x) == acc[0];
	f << dot(pos_vel_y) == acc[1];
	f << dot(pos_vel_z) == acc[2];
	f << dot(ang_vel_roll)  == ang_acc[0];
	f << dot(ang_vel_pitch) == ang_acc[1];
	f << dot(ang_vel_yaw)   == ang_acc[2];

	//inner objective
	//position
	h << pos_x;
	h << pos_y;
	h << pos_z;
	//angle
	h << ang_roll;
	h << ang_pitch;
	h << ang_yaw;
	//pos vel 
	h << pos_vel_x;
	h << pos_vel_y;
	h << pos_vel_z;
	//ang vel
	h << ang_vel_roll;
	h << ang_vel_pitch;
	h << ang_vel_yaw;

	//u thrust x,y
	h << u_thu_x;
	h << u_thu_y;
	//u torque x,y,z
	h << u_torque_x;
	h << u_torque_y;
	h << u_torque_z;
	// //acc
	// h << acc[0];
	// h << acc[1];
	// h << acc[2];
	// //ang_acc
	// h << ang_acc[0];
	// h << ang_acc[1];
	// h << ang_acc[2];
	//the last state function
	hN<< pos_x;
	hN<< pos_y;
	hN<< pos_z;
	hN<< ang_roll;
	hN<< ang_pitch;
	hN<< ang_yaw;
	hN<< pos_vel_x;
	hN<< pos_vel_y;
	hN<< pos_vel_z;
	hN<< ang_vel_roll;
	hN<< ang_vel_pitch;
	hN<< ang_vel_yaw;
	// const int N  = 10;
	// const int Ni = 4;
	// const double Ts = 0.01;
	const double t_start = 0.0;
	const double t_end = 2;
	const double dt = 0.1;
	const int N = round((t_end-t_start)/dt);
	//init weight as a identity matrix
	DMatrix W = eye<double>(h.getDim());
	DMatrix WN = eye<double>(hN.getDim());
	//position
	W(0,0) = 100;
	W(1,1) = 100;
	W(2,2) = 500;
	//angle
	W(3,3) =1.5;
	W(4,4) =1.5;
	W(5,5) =1.5;
	//pos vel
	W(6,6) = 5;
	W(7,7) = 5;
	W(8,8) = 30;
	//ang vel
	W(9,9)   = 2;
	W(10,10) = 2;
	W(11,11) = 2;

	//thrust
	W(12,12) = 2;// thrust x
	W(13,13) = 2;// thrust y
	//torque
	W(14,14) = 2;// torque x
	W(15,15) = 2;// torque y
	W(16,16) = 2;// torque z

	// //acc
	// W(12,12) = 0.1;
	// W(13,13) = 0.1;
	// W(14,14) = 0.1;
	// //ang acc
	// W(15,15) = 0.1;
	// W(16,16) = 0.1;
	// W(17,17) = 0.1;

	// WN matrix 
	WN(0,0)= 100;//pos x
	WN(1,1)= 100;//pos y
	WN(2,2)= 500;//pos z
	WN(3,3)= 1;//ang x
	WN(4,4)= 1;//ang y
	WN(5,5)= 1;//ang z

	WN(6,6)= 1;//vel x
	WN(7,7)= 1;//vel y
	WN(8,8)= 30;//vel z
	WN(9,9)= 2;  //ang vel x
	WN(10,10)= 2;//ang vel y
	WN(11,11)= 2;//ang vel z

	// WN *=10;
	// Q(15,15) = 1;
	// Q(16,16) = 1;
	// Q(17,17) = 1;
	// ROS_INFO_STREAM("Q:"<<W);
	// ROS_INFO_STREAM("Qn:"<<WN);

	OCP ocp(t_start,t_end,N);//what is this line mean
	ocp.minimizeLSQ(W,h);
	ocp.minimizeLSQEndTerm(WN,hN);
	//constraints from dynamic model
	ocp.subjectTo( f );
//position
	ocp.subjectTo( l_pos_x <= pos_x <= u_pos_x );
	ocp.subjectTo( l_pos_y <= pos_y <= u_pos_y );
	ocp.subjectTo( l_pos_z <= pos_z <= u_pos_z );
	//angle
	ocp.subjectTo( l_ang_x <= ang_roll <= u_ang_x  );
	ocp.subjectTo( l_ang_y <= ang_pitch <= u_ang_y );
	ocp.subjectTo( l_ang_z <= ang_yaw <= u_ang_z   );
	//position vel 
	ocp.subjectTo( l_vel_x <= pos_vel_x <= u_vel_x );
	ocp.subjectTo( l_vel_y <= pos_vel_y <= u_vel_y );
	ocp.subjectTo( l_vel_z <= pos_vel_z <= u_vel_z );
	//angle vel
	ocp.subjectTo( l_ang_vel_x <= ang_vel_roll  <= u_ang_vel_x );
	ocp.subjectTo( l_ang_vel_y <= ang_vel_pitch <= u_ang_vel_y );
	ocp.subjectTo( l_ang_vel_z <= ang_vel_yaw   <= u_ang_vel_z );

	ocp.subjectTo( l_thrust_x <= u_thu_x <= u_thrust_x );
	ocp.subjectTo( l_thrust_y <= u_thu_y <= u_thrust_y );
	ocp.subjectTo( l_thrust_z <= u_thu_z <= u_thrust_z );
	ocp.subjectTo( l_torque_x <= u_tor_x <= u_torque_x );
	ocp.subjectTo( l_torque_y <= u_tor_y <= u_torque_y );
	ocp.subjectTo( l_torque_z <= u_tor_z <= u_torque_z );
	
 	OptimizationAlgorithm algorithm(ocp);
	//SETTING UP THE MPC CONTROLLER:
	OCPexport mpc_obj( ocp);
	mpc_obj.set( HESSIAN_APPROXIMATION,GAUSS_NEWTON);
	mpc_obj.set( DISCRETIZATION_TYPE,  MULTIPLE_SHOOTING);
	mpc_obj.set( QP_SOLVER, QP_QPOASES); 
	mpc_obj.set( NUM_INTEGRATOR_STEPS, N); //mpc_obj.set( NUM_INTEGRATOR_STEPS, N * Ni);
	mpc_obj.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2);//mpc_obj.set( SPARSE_QP_SOLUTION, FULL_CONDENSING);
	mpc_obj.set( HOTSTART_QP, YES);        
	mpc_obj.set( INTEGRATOR_TYPE,INT_RK45);//mpc_obj.set( INTEGRATOR_TYPE,INT_RK45);
	mpc_obj.set( GENERATE_TEST_FILE, YES);
	mpc_obj.set( GENERATE_MAKE_FILE, YES);
	mpc_obj.set( GENERATE_MATLAB_INTERFACE, YES);
	mpc_obj.set( GENERATE_SIMULINK_INTERFACE, YES);

 	mpc_obj.set(CG_USE_OPENMP, YES); 
	mpc_obj.set( USE_SINGLE_PRECISION, YES);   
	// mpc_obj.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
	// mpc_obj.set(FIX_INITIAL_STATE, YES);
	if (mpc_obj.exportCode( "acado_mpc_export" ) != SUCCESSFUL_RETURN)
	{
 		exit( EXIT_FAILURE );
	}

	mpc_obj.printDimensionsQP( );
	if(EXIT_SUCCESS)
	{
		ROS_INFO_STREAM("gaoling...");
	}
}

// real init state function of mpc controller
void mpc::init_mpc_fun()
{
	//first of all, i need to get the feedback data
	ros::spinOnce();

	//call the initializeSolver function
	acado_initializeSolver();
	//init x0 , same as input.x0
	acadoVariables.x0[0]  = hover_position.x;//init position x
	acadoVariables.x0[1]  = hover_position.y;//init position y
	acadoVariables.x0[2]  = hover_position.z;//init position z

	acadoVariables.x0[3]  = hover_attitude.x;//init ang x
	acadoVariables.x0[4]  = hover_attitude.y;//init ang y
	acadoVariables.x0[5]  = hover_attitude.z;//init ang z

	acadoVariables.x0[6]  = 0;//main_velocity.x;//init vel x
	acadoVariables.x0[7]  = 0;//main_velocity.y;//init vel y
	acadoVariables.x0[8]  = 0;//main_velocity.z;//init vel z

	acadoVariables.x0[9]  = 0;//main_body_rates.x; //init ang_vel x
	acadoVariables.x0[10] = 0;//main_body_rates.y;//init ang_vel y
	acadoVariables.x0[11] = 0;//main_body_rates.z;//init ang_vel z
	//init x
	for(int i=0;i<NX ; i++)
	{
		for (int j = 0; j < N + 1; j++)
		{
			//first row second col
			acadoVariables.x[i+j*NX]=acadoVariables.x0[i];
		}
	}
	//init u0 
	double u0[NU]; //vector of control 
	u0[0]=0;
	u0[1]=0;
	u0[2]=-62.35;
	u0[3]=0;
	u0[4]=0;
	u0[5]=0;
	// init u
	for (int i = 0; i < NU; i++)
	{
		for (int j = 0; j < N; j++)
		{
			acadoVariables.u[i+j*NU]=u0[i];
		}
	}

	//Initialize the measurements/reference.
	double ref[NY]; 
	ref[0] = hover_position.x;
	ref[1] = hover_position.y;
	ref[2] = hover_position.z;
	ref[3] = hover_attitude.x;
	ref[4] = hover_attitude.y;
	ref[5] = hover_attitude.z;
	ref[6] = 0;//vel_x
	ref[7] = 0;//vel_y
	ref[8] = 0;//vel_z
	ref[9] = 0;//vel_roll
	ref[10] = 0;//vel_pitch
	ref[11] = 0;//vel_yaw
	ref[12] = 0;//vel_y
	ref[13] = 0;//vel_z
	ref[14] = 0;//vel_roll
	ref[15] = 0;//vel_pitch
	ref[16] = 0;//vel_yaw
	for (int i = 0; i < NY; i++)
	{
		for (int j = 0; j < N; j++)
		{
			acadoVariables.y[i+j*NY]=ref[i];
		}
	}

	//Initialize the referenceN
	acadoVariables.yN[0] = hover_position.x;
	acadoVariables.yN[1] = hover_position.y;
	acadoVariables.yN[2] = hover_position.z;
	acadoVariables.yN[3] = hover_attitude.x;
	acadoVariables.yN[4] = hover_attitude.y;
	acadoVariables.yN[5] = hover_attitude.z;

	acadoVariables.yN[6] = 0;
	acadoVariables.yN[7] = 0;
	acadoVariables.yN[8] = 0;
	acadoVariables.yN[9] = 0;
	acadoVariables.yN[10] = 0;
	acadoVariables.yN[11] = 0;
	acado_preparationStep();
}
//the solver of mpc controller,need to be called at every loop
void mpc::mpc_solver()
{

	// acadoVariables.yN[0] = hover_position.x;
	// acadoVariables.yN[1] = hover_position.y;
	// acadoVariables.yN[2] = acadoVariables.yN[2] -0.2*0.012;
	// acadoVariables.yN[3] = hover_attitude.x;
	// acadoVariables.yN[4] = hover_attitude.y;
	// acadoVariables.yN[5] = hover_attitude.z;
	// if(acadoVariables.yN[0]>=hover_position.x+0.8)
	// {
	// 	acadoVariables.yN[0]=hover_position.x+0.8;
	// }
	// if(acadoVariables.yN[2]>=hover_position.z-0.8)
	// {
	// 	acadoVariables.yN[2]=hover_position.z-0.8;
	// }
	acado_printDifferentialVariables();
	acado_printControlVariables();

	acado_feedbackStep();
	acado_shiftStates(2, 0, 0);
	acado_shiftControls(0);
	acado_shiftStates(2, 0, 0);
	acado_shiftControls(0);
	acado_shiftStates(2, 0, 0);
	acado_shiftControls(0);

	acado_preparationStep();
	
}
//the distributor of the whole thrust and torque
void mpc::distributor(Eigen::Vector3f thu,Eigen::Vector3f tor)
{
	//first of all, calc the ratation matrix
	r_mat = euler_to_rotation_mat(main_eular_angles);

	Eigen::Matrix3f I;
	I(0,0)=I(1,1)=I(2,2)=1;
	I(0,1)=I(0,2)=I(1,0)=I(1,2)=I(2,0)=I(2,1)=0;

	Eigen::MatrixXf B(6, 9);
	B.block<3,3>(0,0)=I;
	B.block<3,3>(0,3)=I;
	B.block<3,3>(0,6)=I;
	B.block<3,3>(3,0)=pos_s1;
	B.block<3,3>(3,3)=pos_s2;
	B.block<3,3>(3,6)=pos_s3;

	Eigen::MatrixXf B_calc(6, 6);
	B_calc = B*(B.transpose());
	//get vector of U
	Eigen::VectorXf U(6);
	U.head(3)=thrust_u;
	U.tail<3>()=torques_u;

	Eigen::MatrixXf inter1(3, 6);
	inter1.block<3,3>(0,0)=I;
	inter1.block<3,3>(0,3)=pos_s1.transpose();

	Eigen::MatrixXf inter2(3, 6);
	inter2.block<3,3>(0,0)=I;
	inter2.block<3,3>(0,3)=pos_s2.transpose();

	Eigen::MatrixXf inter3(3, 6);
	inter3.block<3,3>(0,0)=I;
	inter3.block<3,3>(0,3)=pos_s3.transpose();

	lamda1=inter1*(B_calc.inverse()*U);
	lamda2=inter2*(B_calc.inverse()*U);
	lamda3=inter3*(B_calc.inverse()*U);

	//allocation function
	double psi_cmd1=nominal_euler_angles.z+node1_yaw.data;//this line is right
	double psi_cmd2=nominal_euler_angles.z+node2_yaw.data;//this line is right
	double psi_cmd3=nominal_euler_angles.z+node3_yaw.data;//this line is right
	Eigen::Vector4f thu_att1=alloc(lamda1,psi_cmd1);
	Eigen::Vector4f thu_att2=alloc(lamda2,psi_cmd2);
	Eigen::Vector4f thu_att3=alloc(lamda3,psi_cmd3);

	//push data to message

	thu1.data = thu_att1(0);
	ang1.x    = thu_att1(1);
	ang1.y    = thu_att1(2);
	ang1.z    = thu_att1(3);

	thu2.data = thu_att2(0);
	ang2.x    = thu_att2(1);
	ang2.y    = thu_att2(2);
	ang2.z    = thu_att2(3);

	thu3.data = thu_att3(0);
	ang3.x    = thu_att3(1);
	ang3.y    = thu_att3(2);
	ang3.z    = thu_att3(3);
	//I need to check the 

}

//alloc function
Eigen::Vector4f mpc::alloc(Eigen::Vector3f f,double psi_cmd)
{
	Eigen::Vector4f thu_att;
	Eigen::Vector3f fw;
	fw=r_mat*f;

	thu_att(0)=sqrt(fw(0)*fw(0)+fw(1)*fw(1)+fw(2)*fw(2));
	thu_att(1)=asin((fw(1)*cos(psi_cmd)-fw(0)*sin(psi_cmd))/thu_att(0));
    thu_att(2)=asin(-(fw(0)*cos(psi_cmd)+fw(1)*sin(psi_cmd))/sqrt((fw(0)*cos(psi_cmd)+fw(1)*sin(psi_cmd))*(fw(0)*cos(psi_cmd)+fw(1)*sin(psi_cmd))+ fw(2)*fw(2)));
	thu_att(3)=psi_cmd;
	return thu_att;
}

Eigen::Matrix3f mpc::euler_to_rotation_mat(geometry_msgs::Point angle)
{
	Eigen::Matrix3f mat;
	mat(0,0)=cos(angle.y)*cos(angle.z);
	mat(0,1)=sin(angle.x)*sin(angle.y)*cos(angle.z)-cos(angle.x)*sin(angle.z);
	mat(0,2)=cos(angle.z)*sin(angle.y)*cos(angle.x)+sin(angle.z)*sin(angle.x);
	mat(1,0)=cos(angle.y)*sin(angle.z);
	mat(1,1)=sin(angle.z)*sin(angle.y)*sin(angle.x)+cos(angle.z)*cos(angle.x);
	mat(1,2)=sin(angle.z)*sin(angle.y)*cos(angle.x)-cos(angle.z)*sin(angle.x);
	mat(2,0)=-sin(angle.y);
	mat(2,1)=sin(angle.x)*cos(angle.y);
	mat(2,2)=cos(angle.x)*cos(angle.y);
	//ROS_INFO_STREAM(mat);
	return mat;
}

//callback funtion define list
//setpoint position, chk 
void mpc::nominal_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_position = *msg;
	// ROS_INFO_STREAM("nominal_position: ");
	// ROS_INFO_STREAM(nominal_position);
}
//setpoint attitude, eular angles, chk
void mpc::nominal_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_euler_angles = *msg;
	// ROS_INFO_STREAM("nominal_euler_angles: ");
	// ROS_INFO_STREAM(nominal_euler_angles);
}
//position back, chk
void mpc::main_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	main_position = *msg;
	// ROS_INFO_STREAM("main_position: ");
	// ROS_INFO_STREAM(main_position);
}
//velocity back, chk
void mpc::main_velocity_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	main_velocity = *msg;
	// ROS_INFO_STREAM("main_velocity: ");
	// ROS_INFO_STREAM(main_velocity);
}
//attitude back, chk
void mpc::main_eular_angles_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	geometry_msgs::Point angles;
	angles = *msg;

	main_eular_angles.x = angles.x;
	main_eular_angles.y = angles.y;
	main_eular_angles.z = angles.z;
	
	// ROS_INFO_STREAM("main_eular_angles: ");
	// ROS_INFO_STREAM(main_eular_angles);
}
//body rates back,chk
void mpc::main_body_rates_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	main_body_rates = *msg;
	// ROS_INFO_STREAM("main_body_rates: ");
	// ROS_INFO_STREAM(main_body_rates);
}
//init attitude,chk
void mpc::init_euler_angles_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_euler_angles = *msg;
	// ROS_INFO_STREAM("init_euler_angles: ");
	// ROS_INFO_STREAM(init_euler_angles);
}
//init position,chk
void mpc::init_pos_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_position = *msg;
	// ROS_INFO_STREAM("init_position: ");
	// ROS_INFO_STREAM(init_position);
}
//init body rate,chk
void mpc::init_body_rates_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_body_rate = *msg;
	// ROS_INFO_STREAM("init_body_rate: ");
	// ROS_INFO_STREAM(init_body_rate);
}
//init velocity 
void mpc::init_velocity_cmd_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	init_velocity = *msg;
	// ROS_INFO_STREAM("init_velocity: ");
	// ROS_INFO_STREAM(init_velocity);
}

//mode switch callback function
void mpc::mode_switch_sub_cb(const std_msgs::Bool::ConstPtr& msg)
{
	mode_switch = *msg;
}

//control start chk
void mpc::control_start_sub_att_cb(const std_msgs::Bool::ConstPtr& msg)
{
	start_pub_att = *msg;
}
//psi_bias 1 chk
void mpc::psi_bias_1_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node1_yaw = *msg;
}
//psi_bias 2 chk
void mpc::psi_bias_2_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node2_yaw = *msg;
}
//psi_bias 3 chk
void mpc::psi_bias_3_sub_cb(const std_msgs::Float64::ConstPtr& msg)
{
	node3_yaw = *msg;
}
//output function 
void mpc::output_publish(geometry_msgs::Point ang1,geometry_msgs::Point ang2,geometry_msgs::Point ang3,std_msgs::Float64 thu1,std_msgs::Float64 thu2,std_msgs::Float64 thu3)
{
    //publish message ouput
	angle1_pub.publish(ang1) ;
	angle2_pub.publish(ang2) ;
	angle3_pub.publish(ang3) ;
	thrust1_pub.publish(thu1);
	thrust2_pub.publish(thu2);
	thrust3_pub.publish(thu3);
}

//get input value and publish and print the input value
void mpc::get_input()
{
	// if ((-45 <= acadoVariables.u[0] <= 45)&&
	// (-45 <= acadoVariables.u[1] <= 45)&&
	// (-90 <= acadoVariables.u[2] <= 0)&&
	// (-10 <= acadoVariables.u[3] <= 10)&&
	// (-10 <= acadoVariables.u[4] <= 10)&&
	// (-10 <= acadoVariables.u[5] <= 10))
	fisrt_order_filter(in_thrust,in_torque);
	{
	in_thrust[0] =std::max(l_thrust_x,std::min(u_thrust_x,(double)acadoVariables.u[0]));
	in_thrust[1] =std::max(l_thrust_y,std::min(u_thrust_y,(double)acadoVariables.u[1]));
	in_thrust[2] =std::max(l_thrust_z,std::min(u_thrust_z,(double)acadoVariables.u[2]));
	in_torque[0] =std::max(l_torque_x,std::min(u_torque_x,(double)acadoVariables.u[3]));
	in_torque[1] =std::max(l_torque_y,std::min(u_torque_y,(double)acadoVariables.u[4]));
	in_torque[2] =std::max(l_torque_z,std::min(u_torque_z,(double)acadoVariables.u[5]));
	
	// thrust_u[0] =acadoVariables.u[0];
	// thrust_u[1] =acadoVariables.u[1];
	// thrust_u[2] =acadoVariables.u[2];
	// torques_u[0]=acadoVariables.u[3];
	// torques_u[1]=acadoVariables.u[4];
	// torques_u[2]=acadoVariables.u[5];
	}
	

	ROS_INFO_STREAM("gaoling and kaidi ");
	// ROS_INFO_STREAM("u:"<<thrust_u[0]<<" "<<thrust_u[1]<<" "<<thrust_u[2]
	// <<" "<<torques_u[0]<<" "<<torques_u[1]<<" "<<torques_u[2]);
	distributor(thrust_u,torques_u);
	output_publish(ang1,ang2,ang3,thu1,thu2,thu3);

	// ROS_INFO_STREAM("thu1:"<<thu1.data);
	// ROS_INFO_STREAM("ang1.x:"<<ang1.x);
	// ROS_INFO_STREAM("ang1.y:"<<ang1.y);
	// ROS_INFO_STREAM("ang1.z:"<<ang1.z);

	// ROS_INFO_STREAM("thu2:"<<thu2.data);
	// ROS_INFO_STREAM("ang2.x:"<<ang2.x);
	// ROS_INFO_STREAM("ang2.y:"<<ang2.y);
	// ROS_INFO_STREAM("ang2.z:"<<ang2.z);

	// ROS_INFO_STREAM("thu3:"<<thu3.data);
	// ROS_INFO_STREAM("ang3.x:"<<ang3.x);
	// ROS_INFO_STREAM("ang3.y:"<<ang3.y);
	// ROS_INFO_STREAM("ang3.z:"<<ang3.z);

}

//get state value and print them
void mpc::get_state()
{
	//get states value
	for (int i = 0; i < NX; i++)
	{
		states_var[i] = acadoVariables.x0[i];
	}
	//print states value
	ROS_INFO_STREAM("states:"<<states_var[0]<<" "<<states_var[1]<<" "<<states_var[2]
	<<" "<<states_var[3]<<" "<<states_var[4]<<" "<<states_var[5]
	<<" "<<states_var[6]<<" "<<states_var[7]<<" "<<states_var[8]
	<<" "<<states_var[9]<<" "<<states_var[10]<<" "<<states_var[11]
	);
}
//refresh the state and reference value
void mpc::update(
	geometry_msgs::Point position, //state position
	geometry_msgs::Point attitude, //state attitude
	geometry_msgs::Point vel_pos,  //state velocity
	geometry_msgs::Point vel_ang,  //state velocity of euler angle
	geometry_msgs::Point ref_pos,  //reference position
	geometry_msgs::Point ref_att,  //reference attitude
	geometry_msgs::Point ref_vel_pos, //reference velocity
	geometry_msgs::Point ref_vel_ang) //reference velocity of euler angle
{
	acadoVariables.x0[0]  = main_position.x;    //init position x
	acadoVariables.x0[1]  = main_position.y;    //init position y
	acadoVariables.x0[2]  = main_position.z;    //init position z
	//acadoVariables.x0[2]  = -2;
	acadoVariables.x0[3]  = main_eular_angles.x;//init ang x
	acadoVariables.x0[4]  = main_eular_angles.y;//init ang y
	acadoVariables.x0[5]  = main_eular_angles.z;//init ang z

	acadoVariables.x0[6]  = main_velocity.x;    //init vel x
	acadoVariables.x0[7]  = main_velocity.y;    //init vel y
	acadoVariables.x0[8]  = main_velocity.z;    //init vel z

	acadoVariables.x0[9]  = main_body_rates.x;  //init ang_vel x
	acadoVariables.x0[10] = main_body_rates.y;  //init ang_vel y
	acadoVariables.x0[11] = main_body_rates.z;  //init ang_vel z

	ROS_INFO_STREAM("ref:"<<hover_position.x<<" "<<hover_position.y<<" "<<hover_position.z
	<<" "<<hover_attitude.x<<" "<<hover_attitude.y<<" "<<hover_attitude.z);	

	// ROS_INFO_STREAM(hover_position.x);
	// ROS_INFO_STREAM(hover_position.y);
	// ROS_INFO_STREAM(hover_position.z);
	// ROS_INFO_STREAM(hover_attitude.x);
	// ROS_INFO_STREAM(hover_attitude.y);
	// ROS_INFO_STREAM(hover_attitude.z);
	// ROS_INFO_STREAM("+++++++++++++++");
}
//filter first-order function
void mpc::fisrt_order_filter(Eigen::Vector3f in_data_thu,Eigen::Vector3f in_data_tor)
{
	thrust_u[0] = a_thu_x*in_data_thu[0]+(1-a_thu_x)*thrust_u[0];
	thrust_u[1] = a_thu_y*in_data_thu[1]+(1-a_thu_y)*thrust_u[1];
	thrust_u[2] = a_thu_z*in_data_thu[2]+(1-a_thu_z)*thrust_u[2];

	torques_u[0]= a_tor_x*in_data_tor[0]+(1-a_tor_x)*torques_u[0];
	torques_u[1]= a_tor_y*in_data_tor[1]+(1-a_tor_y)*torques_u[1];
	torques_u[2]= a_tor_z*in_data_tor[2]+(1-a_tor_z)*torques_u[2];
}
//timer callback function
void mpc::calc_cb(const ros::TimerEvent&)
{
	if(start_pub_att.data)
	{
		if(mode_switch.data)
		{
			// distributor(thrust_u,torques_u);
			// output_publish(ang1,ang2,ang3,thu1,thu2,thu3);
		}
		else
		{	
			// get callback position and attitude as the init position and attitude for mpc controller
			hover_position.x = main_position.x;
			hover_position.y = main_position.y;
			hover_position.z = main_position.z;
			hover_attitude.x = main_eular_angles.x;
			hover_attitude.y = main_eular_angles.y;
			hover_attitude.z = main_eular_angles.z;
			// ROS_INFO_STREAM(hover_position.x);
			// ROS_INFO_STREAM(hover_position.y);
			// ROS_INFO_STREAM(hover_position.z);
			// ROS_INFO_STREAM(hover_attitude.x);
			// ROS_INFO_STREAM(hover_attitude.y);
			// ROS_INFO_STREAM(hover_attitude.z);
			// ROS_INFO_STREAM("+++++++++++++++");
		}
	}
}

//main function
int main(int argc, char **argv)
{
    ros::init(argc,argv,"mpc");
    ros::NodeHandle nh;
    mpc mpc_node(&nh);

	mpc_node.ref_n[2]=mpc_node.hover_position.z;
	while (ros::ok())
	{
		if(mpc_node.start_pub_att.data)//if start_pub_att == true, then start the mpc controller
		{
			if(mpc_node.mode_switch.data)//start mpc controller
			{
				if(mpc_node.init_fun_switch==0)//init judge 
				{
					ROS_INFO_STREAM("you only look once..");
					mpc_node.init_mpc_fun();//init mpc at the first time
				}
				//recore section
				mpc_node.init_fun_switch++;//count running times
				//mpc mark to 1 imply that mpc controller running
				mpc_node.mark.data = 1;//mark the time of mpc controller start
				mpc_node.mpc_mark_pub.publish(mpc_node.mark);
				
				//get the input and publish and print the input value
				mpc_node.get_input();
				//get the state and print the states value
				mpc_node.get_state();
				//solver and parpare for next step
				mpc_node.mpc_solver();
				//update state and reference
				mpc_node.update(
					mpc_node.main_position,
					mpc_node.main_eular_angles,
					mpc_node.main_velocity,
					mpc_node.main_body_rates,
					mpc_node.r_pos,
					mpc_node.r_att,
					mpc_node.r_vel_pos,
					mpc_node.r_vel_ang);

				mpc_node.count_num_test++;
				ROS_INFO_STREAM("times:"<<mpc_node.count_num_test);
				if(mpc_node.count_num_test>1000)
				{
					//ros::shutdown();// close the ros node
				}
			}
			else
			{
				//if out the mpc controller then mark set to 0
				mpc_node.mark.data = 1;
			}
		}
		ros::spinOnce();
	}
    return 0;
}
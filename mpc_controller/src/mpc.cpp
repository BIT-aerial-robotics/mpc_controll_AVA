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
    //mpc_state_function();
	// init_mpc_fun();//real init mpc function

	//kaidi wang 10.14, stop timer
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

	// const int N  = 10;
	// const int Ni = 4;
	// const double Ts = 0.01;
	const double t_start = 0.0;
	const double t_end = 1.5;
	const double dt = 0.05;
	const int N = round((t_end-t_start)/dt);
	//init weight as a identity matrix
	DMatrix W = eye<double>(h.getDim());
	DMatrix WN = eye<double>(hN.getDim());
	//position
	W(0,0) = 60;
	W(1,1) = 10;
	W(2,2) = 50;
	//angle
	W(3,3) =12;
	W(4,4) =10;
	W(5,5) =10;
	//pos vel
	W(6,6) = 25;
	W(7,7) = 20;
	W(8,8) = 3;
	//ang vel
	W(9,9)   = 22;
	W(10,10) = 20;
	W(11,11) = 20;
	// //acc
	// W(12,12) = 0.1;
	// W(13,13) = 0.1;
	// W(14,14) = 0.1;
	// //ang acc
	// W(15,15) = 0.1;
	// W(16,16) = 0.1;
	// W(17,17) = 0.1;

	// WN matrix 
	WN(0,0)= 5;
	WN(1,1)= 3;
	WN(2,2)= 8;
	WN(3,3)= 1;
	WN(4,4)= 1;
	WN(5,5)= 1;

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
	ocp.subjectTo( -100 <= pos_x <= 100 );
	ocp.subjectTo( -100 <= pos_y <= 100 );
	ocp.subjectTo( -100 <= pos_z <= 100 );
	//angle
	ocp.subjectTo( -PI/3 <= ang_roll <= PI/3  );
	ocp.subjectTo( -PI/3 <= ang_pitch <= PI/3 );
	ocp.subjectTo( -2*PI <= ang_yaw <= 2*PI   );
	//position vel 
	ocp.subjectTo( -1 <= pos_vel_x <= 1 );
	ocp.subjectTo( -1 <= pos_vel_y <= 1 );
	ocp.subjectTo( -1 <= pos_vel_z <= 1 );
	//angle vel
	ocp.subjectTo( -0.5 <= ang_vel_roll  <= 0.5 );
	ocp.subjectTo( -0.5 <= ang_vel_pitch <= 0.5 );
	ocp.subjectTo( -0.5 <= ang_vel_yaw   <= 0.5 );

	ocp.subjectTo( -5 <= u_thu_x <= 5 );
	ocp.subjectTo( -5 <= u_thu_y <= 5 );
	ocp.subjectTo( -64 <= u_thu_z <= 64 );
	ocp.subjectTo( -1 <= u_tor_x <= 1);
	ocp.subjectTo( -1 <= u_tor_y <= 1 );
	ocp.subjectTo( -1 <= u_tor_z <= 1 );
	//init subject of control and state
	//init position
	ocp.subjectTo(AT_START, pos_x==init_position.x);
	ocp.subjectTo(AT_START, pos_y==init_position.y);
	ocp.subjectTo(AT_START, pos_z==init_position.z);	
	//init attitude
	ocp.subjectTo(AT_START, ang_roll  ==init_euler_angles.x);
	ocp.subjectTo(AT_START, ang_pitch ==init_euler_angles.y);
	ocp.subjectTo(AT_START, ang_yaw   ==init_euler_angles.z);
	//init linear velocity
	ocp.subjectTo(AT_START, pos_vel_x==init_velocity.x);
	ocp.subjectTo(AT_START, pos_vel_y==init_velocity.y);
	ocp.subjectTo(AT_START, pos_vel_z==init_velocity.z);	
	//init body rate
	ocp.subjectTo(AT_START, ang_vel_roll  ==init_body_rate.x);
	ocp.subjectTo(AT_START, ang_vel_pitch ==init_body_rate.y);
	ocp.subjectTo(AT_START, ang_vel_yaw   ==init_body_rate.z);	

	//SETTING UP THE MPC CONTROLLER:
	OCPexport mpc_obj( ocp);
	mpc_obj.set( HESSIAN_APPROXIMATION,GAUSS_NEWTON);
	mpc_obj.set( DISCRETIZATION_TYPE,  MULTIPLE_SHOOTING );
	mpc_obj.set( QP_SOLVER, QP_QPOASES  ); 
	mpc_obj.set( NUM_INTEGRATOR_STEPS, N); //mpc_obj.set( NUM_INTEGRATOR_STEPS, N * Ni);
	mpc_obj.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2);//mpc_obj.set( SPARSE_QP_SOLUTION, FULL_CONDENSING);
	mpc_obj.set( HOTSTART_QP, YES);        
	mpc_obj.set( INTEGRATOR_TYPE,INT_IRK_GL4);//mpc_obj.set( INTEGRATOR_TYPE,INT_RK45);
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
	/* Some temporary variables. */
	//int    i, iter;
	//acado_timer t;
	/* Initialize the solver. */
	//acado_initializeSolver();
	// mpc.set(NUM_INTEGRATOR_STEPS,20);
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
		//init all pos x
		// acadoVariables.x[i]      =acadoVariables.x0[i];//1
		// acadoVariables.x[i+1*NX] =acadoVariables.x0[i];//2
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//3
		// acadoVariables.x[i+3*NX] =acadoVariables.x0[i];//4
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//5
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//6
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//7
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//8
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//9
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//10
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//11
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//12
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//13
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//14
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//15
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//16
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//17
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//18
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//19
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//20
		// acadoVariables.x[i+2*NX] =acadoVariables.x0[i];//21
		for (int j = 0; j < N + 1; j++)
		{
			//first row second col
			acadoVariables.x[i+j*NX]=acadoVariables.x0[i];
		}
	}
	//init u0 
	double u0[NU]; //vector of control 
	// for (int i = 0; i < NU; i++)
	// {
	// 	/* code */
	// 	u0[i]=0;
	// }	
	u0[1]=0;
	u0[2]=0;
	u0[3]=-62.35;
	u0[4]=0;
	u0[5]=0;
	u0[6]=0;
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

	// ref[0] = 1;//x
	// ref[1] = 2;//y
	// ref[2] = +8;//y
	// ref[3] = 0.5;//ang_x
	// ref[4] = 0.1;//ang_y
	// ref[5] = 0.2;//ang_z

	ref[6] = 0;//vel_x
	ref[7] = 0;//vel_y
	ref[8] = 0;//vel_z
	ref[9] = 0;//vel_roll
	ref[10] = 0;//vel_pitch
	ref[11] = 0;//vel_yaw
	// initialize the reference remain
	// for (int i = 6; i < NY; i++)
	// {
	// 	ref[i] = 0;
	// }
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
	acadoVariables.yN[2] = hover_position.z+2;
	acadoVariables.yN[3] = hover_attitude.x;
	acadoVariables.yN[4] = hover_attitude.y;
	acadoVariables.yN[5] = hover_attitude.z;
	// acadoVariables.yN[0] = 1;
	// acadoVariables.yN[1] = 2;
	// acadoVariables.yN[2] = +8;
	// acadoVariables.yN[3] = 0.5;
	// acadoVariables.yN[4] = 0.1;
	// acadoVariables.yN[5] = 0.2;
	//ROS_INFO_STREAM("yN:"<<acadoVariables.yN[0]);
	acado_preparationStep();
}
//the solver of mpc controller,need to be called at every loop
void mpc::mpc_solver()
{
	// ROS_INFO_STREAM("x0:");
	// ROS_INFO_STREAM(
	// 	" "<<acadoVariables.x0[0]<<
	// 	" "<<acadoVariables.x0[1]<<
	// 	" "<<acadoVariables.x0[2]<<
	// 	" "<<acadoVariables.x0[3]<<
	// 	" "<<acadoVariables.x0[4]<<
	// 	" "<<acadoVariables.x0[5]<<
	// 	" "<<acadoVariables.x0[6]<<
	// 	" "<<acadoVariables.x0[7]<<
	// 	" "<<acadoVariables.x0[8]<<
	// 	" "<<acadoVariables.x0[9]<<
	// 	" "<<acadoVariables.x0[10]<<
	// 	" "<<acadoVariables.x0[11]
	// );
	
	{
		acado_feedbackStep();
		acado_printDifferentialVariables();
    	acado_printControlVariables();
		thrust_u[0] =acadoVariables.u[0];
		thrust_u[1] =acadoVariables.u[1];
		thrust_u[2] =acadoVariables.u[2];
		torques_u[0]=acadoVariables.u[3];
		torques_u[1]=acadoVariables.u[4];
		torques_u[2]=acadoVariables.u[5];

		// distributor(thrust_u,torques_u);
		// output_publish(ang1,ang2,ang3,thu1,thu2,thu3);
		
		acado_shiftStates(2, 0, 0); 
		acado_shiftControls(0); 
	}

	ros::spinOnce();
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

	
	ref_n[0] = hover_position.x;
	ref_n[1] = hover_position.y;
	ref_n[2] = ref_n[2]+0.1*0.012;
	if(ref_n[2]>=hover_position.z+2)
	{
		ref_n[2]=hover_position.z+2;
	}
	ref_n[3] = hover_attitude.x;
	ref_n[4] = hover_attitude.y;
	ref_n[5] = hover_attitude.z;

	// initialize the reference remain
	for (int i = 6; i < NY; i++)
	{
		ref_n[i] = 0;
	}

	//ROS_INFO_STREAM("acadoVariables.y:");
	for (int i = 0; i < NY; i++)
	{
		for (int j = 0; j < N; j++)
		{
			acadoVariables.y[i+j*NY]=ref_n[i];
			// ROS_INFO_STREAM(" "<<i+j*NY<<":"<<acadoVariables.y[i+j*NY]);
		}
	}
	// acadoVariables.yN[0] = 1.1;
	// acadoVariables.yN[1] = 2.1;
	// acadoVariables.yN[2] = +8.1;
	// acadoVariables.yN[3] = 0.5;
	// acadoVariables.yN[4] = 0.1;
	// acadoVariables.yN[5] = 0.2;

	acado_preparationStep();
	count_num_test++;
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

//timer callback function
void mpc::calc_cb(const ros::TimerEvent&)
{


	if(start_pub_att.data)
	{
		if(mode_switch.data)
		{
			distributor(thrust_u,torques_u);
			output_publish(ang1,ang2,ang3,thu1,thu2,thu3);
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
			ROS_INFO_STREAM(hover_position.x);
			ROS_INFO_STREAM(hover_position.y);
			ROS_INFO_STREAM(hover_position.z);
			ROS_INFO_STREAM(hover_attitude.x);
			ROS_INFO_STREAM(hover_attitude.y);
			ROS_INFO_STREAM(hover_attitude.z);
			ROS_INFO_STREAM("+++++++++++++++");
		}
	}
/*	
	if(start_pub_att.data)//if start_pub_att == true, then start the mpc controller
	{
		if(mode_switch.data)
		{
			mpc solver function
			mpc_solver();
			display thrust_u and torques_u
			ROS_INFO_STREAM("thrust:"<<thrust_u);
			ROS_INFO_STREAM("torque:"<<torques_u);
			
			distributor of thrust and torque
			distributor(thrust_u,torques_u);
			output_publish(ang1,ang2,ang3,thu1,thu2,thu3);

			display thu1,thu2,thu3, ang1,ang2,ang3
			ROS_INFO_STREAM("thu1:"<<thu1.data);
			ROS_INFO_STREAM("thu2:"<<thu2.data);
			ROS_INFO_STREAM("thu3:"<<thu3.data);

			ROS_INFO_STREAM("ang1.x:"<<ang1.x);
			ROS_INFO_STREAM("ang1.y:"<<ang1.y);
			ROS_INFO_STREAM("ang1.z:"<<ang1.z);

			ROS_INFO_STREAM("ang2.x:"<<ang2.x);
			ROS_INFO_STREAM("ang2.y:"<<ang2.y);
			ROS_INFO_STREAM("ang2.z:"<<ang2.z);

			ROS_INFO_STREAM("ang3.x:"<<ang3.x);
			ROS_INFO_STREAM("ang3.y:"<<ang3.y);
			ROS_INFO_STREAM("ang3.z:"<<ang3.z);

			ROS_INFO_STREAM("..........................");
		}
	}
	*/
}

//main function
int main(int argc, char **argv)
{
    ros::init(argc,argv,"mpc");
    ros::NodeHandle nh;
    mpc mpc_node(&nh);
	//ros::spin();
	mpc_node.ref_n[2]=mpc_node.hover_position.z;
	while (ros::ok())
	{
		// if (mpc_node.count_num_test>80000)
		// {
		// 	/* code */
		// 	mpc_node.start_pub_att.data = false;
		// 	mpc_node.mark.data = 0;
		// }
		if(mpc_node.start_pub_att.data)//if start_pub_att == true, then start the mpc controller
		{
			if(mpc_node.mode_switch.data)
			{
				if(mpc_node.init_fun_switch==1)
				{
					//real init mpc function, only need to be call once a time
					ROS_INFO_STREAM("you only look once..");
					mpc_node.init_mpc_fun();
					
				}
				mpc_node.init_fun_switch++;

				//mpc mark to 1 imply that mpc controller running
				mpc_node.mark.data = 1;
				mpc_node.mpc_mark_pub.publish(mpc_node.mark);
				
				mpc_node.mpc_solver();
				ROS_INFO_STREAM("thrust:"<<mpc_node.thrust_u);
				ROS_INFO_STREAM("torque:"<<mpc_node.torques_u);	
				//display thu1,thu2,thu3, ang1,ang2,ang3
				// ROS_INFO_STREAM("thu1:"<<mpc_node.thu1.data);
				// ROS_INFO_STREAM("thu2:"<<mpc_node.thu2.data);
				// ROS_INFO_STREAM("thu3:"<<mpc_node.thu3.data);

				// ROS_INFO_STREAM("ang1.x:"<<mpc_node.ang1.x);
				// ROS_INFO_STREAM("ang1.y:"<<mpc_node.ang1.y);
				// ROS_INFO_STREAM("ang1.z:"<<mpc_node.ang1.z);

				// ROS_INFO_STREAM("ang2.x:"<<mpc_node.ang2.x);
				// ROS_INFO_STREAM("ang2.y:"<<mpc_node.ang2.y);
				// ROS_INFO_STREAM("ang2.z:"<<mpc_node.ang2.z);

				// ROS_INFO_STREAM("ang3.x:"<<mpc_node.ang3.x);
				// ROS_INFO_STREAM("ang3.y:"<<mpc_node.ang3.y);
				// ROS_INFO_STREAM("ang3.z:"<<mpc_node.ang3.z);
				// ROS_INFO_STREAM("..................................");
			}
		}
		ros::spinOnce();
	}
    return 0;
}

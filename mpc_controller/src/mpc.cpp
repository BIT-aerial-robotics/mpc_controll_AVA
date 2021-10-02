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
    init_subscriber();//init subscriber list handle
	init_publisher(); //init publisher list handle

	//init nominal_position and nominal_euler_angles
	nominal_position.x = init_position.x;
	nominal_position.y = init_position.y;
	nominal_position.z = init_position.z;
	nominal_euler_angles.x = init_euler_angles.x;
	nominal_euler_angles.y = init_euler_angles.y;
	nominal_euler_angles.z = init_euler_angles.z;

	//mpc_state_function();//called once mpc state function
	init_mpc_fun();//real init mpc function
	//acado_reference_states_ << acadoVariables.y;
	//init_mpc();
    //timer used to publish state, should be at least for some minimal frequency
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
	s(0,2)=y;
	s(1,0)=z;
	s(1,2)=-x;
	s(2,0)=-y;
	s(2,1)=x;
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
	f << dot(ang_roll) == ang_vel[0];//angle vel
	f << dot(ang_pitch) == ang_vel[1];
	f << dot(ang_yaw) == ang_vel[2];
	f << dot(pos_vel_x) == acc[0];
	f << dot(pos_vel_y) == acc[1];
	f << dot(pos_vel_z) == acc[2];
	f << dot(ang_vel_roll) == ang_acc[0];
	f << dot(ang_vel_pitch) == ang_acc[1];
	f << dot(ang_vel_yaw) == ang_acc[2];

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
	//acc
	h << acc[0];
	h << acc[1];
	h << acc[2];
	//ang_acc
	h << ang_acc[0];
	h << ang_acc[1];
	h << ang_acc[2];
	//the last state function
	hN<< pos_x;
	hN<< pos_y;
	hN<< pos_z;
	hN<< ang_roll;
	hN<< ang_pitch;
	hN<< ang_yaw;

	const int N  = 10;
	const int Ni = 4;
	const double Ts = 0.1;
	//init weight as a identity matrix
	DMatrix W = eye<double>( h.getDim() );
	DMatrix WN = eye<double>( hN.getDim() );
	W(0,0) = 1;
	W(1,1) = 1;
	W(2,2) = 1;
	//angle
	W(3,3) = 1;
	W(4,4) = 1;
	W(5,5) = 1;
	//pos vel
	W(6,6) = 0.1;
	W(7,7) = 0.1;
	W(8,8) = 0.1;
	//ang vel
	W(9,9) = 0.1;
	W(10,10) = 0.1;
	W(11,11) = 0.1;
	//acc
	W(12,12) = 0.1;
	W(13,13) = 0.1;
	W(14,14) = 0.1;
	//ang acc
	W(15,15) = 0.1;
	W(16,16) = 0.1;
	W(17,17) = 0.1;

	// WN matrix 
	WN(0,0)= 1;
	WN(1,1)= 1;
	WN(2,2)= 1;
	WN(3,3)= 1;
	WN(4,4)= 1;
	WN(5,5)= 1;

	// WN *=10;
	// Q(15,15) = 1;
	// Q(16,16) = 1;
	// Q(17,17) = 1;
	// ROS_INFO_STREAM("Q:"<<W);
	// ROS_INFO_STREAM("Qn:"<<WN);

	OCP ocp(0,1,25);//what is this line mean
	ocp.minimizeLSQ(W,h);
	ocp.minimizeLSQEndTerm(WN,hN);
	//constraints from dynamic model
	ocp.subjectTo( f );
//position
	ocp.subjectTo( -10 <= pos_x <= 10 );
	ocp.subjectTo( -10 <= pos_y <= 10 );
	ocp.subjectTo( -10 <= pos_z <= 10 );
	//angle
	ocp.subjectTo( -PI/6 <= ang_roll <= PI/6 );
	ocp.subjectTo( -PI/6 <= ang_pitch <= PI/6 );
	ocp.subjectTo( -PI/3 <= ang_yaw <= PI/3 );
	//position vel 
	ocp.subjectTo( -1 <= pos_vel_x <= 1 );
	ocp.subjectTo( -1 <= pos_vel_y <= 1 );
	ocp.subjectTo( -1 <= pos_vel_z <= 1 );
	//angle vel
	ocp.subjectTo( -0.2 <= ang_vel_roll  <= 0.2 );
	ocp.subjectTo( -0.2 <= ang_vel_pitch <= 0.2 );
	ocp.subjectTo( -0.2 <= ang_vel_yaw   <= 0.2 );


	ocp.subjectTo( -100 <= u_thu_x <= 100 );
	ocp.subjectTo( -100 <= u_thu_y <= 100 );
	ocp.subjectTo( -100 <= u_thu_z <= 100 );
	ocp.subjectTo( -10 <= u_tor_x <= 10 );
	ocp.subjectTo( -10 <= u_tor_y <= 10 );
	ocp.subjectTo( -10 <= u_tor_z <= 10 );
	//init subject of control and state
	//init position
	ocp.subjectTo(AT_START, pos_x==init_position.x);
	ocp.subjectTo(AT_START, pos_y==init_position.y);
	ocp.subjectTo(AT_START, pos_z==init_position.z);
	//init linear velocity
	ocp.subjectTo(AT_START, pos_vel_x==init_velocity.x);
	ocp.subjectTo(AT_START, pos_vel_y==init_velocity.y);
	ocp.subjectTo(AT_START, pos_vel_z==init_velocity.z);	
	//init attitude
	ocp.subjectTo(AT_START, ang_roll  ==init_euler_angles.x);
	ocp.subjectTo(AT_START, ang_pitch ==init_euler_angles.y);
	ocp.subjectTo(AT_START, ang_yaw   ==init_euler_angles.z);	
	//init body rate
	ocp.subjectTo(AT_START, ang_vel_roll  ==init_body_rate.x);
	ocp.subjectTo(AT_START, ang_vel_pitch ==init_body_rate.y);
	ocp.subjectTo(AT_START, ang_vel_yaw   ==init_body_rate.z);	

	//SETTING UP THE MPC CONTROLLER:
	OCPexport mpc_obj( ocp);
	mpc_obj.set( HESSIAN_APPROXIMATION,GAUSS_NEWTON);
    mpc_obj.set( DISCRETIZATION_TYPE,  MULTIPLE_SHOOTING );
	mpc_obj.set( QP_SOLVER, QP_QPOASES  ); 
	mpc_obj.set( NUM_INTEGRATOR_STEPS, N * Ni);
	mpc_obj.set( SPARSE_QP_SOLUTION, FULL_CONDENSING);
	mpc_obj.set( HOTSTART_QP, NO);        
	mpc_obj.set( INTEGRATOR_TYPE,INT_RK45);
	mpc_obj.set( GENERATE_TEST_FILE, YES);
	mpc_obj.set( GENERATE_MAKE_FILE, YES);
	mpc_obj.set( GENERATE_MATLAB_INTERFACE, YES);
	mpc_obj.set( GENERATE_SIMULINK_INTERFACE, YES);
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

//init solver of mpc controller
void mpc::init_mpc()
{
	int    i, iter;
	acado_timer t;
	/* Initialize the solver. */
	acado_initializeSolver();

/* MPC: initialize the current state feedback. */
//#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < NX; ++i) 
	{
		acadoVariables.x0[ i ] = 0.1;
	}

//#endif
	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  
	{
		// initialize state, which get by callback function
		acadoVariables.x[ i ] = 0.0;
	}
	for (i = 0; i < NU * N; ++i) 
	{
		acadoVariables.u[ i ] = 0.0;
	} 

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)
	{
		acadoVariables.y[ i ] = 0.0;
	}  
	for (i = 0; i < NYN; ++i)
	{
		acadoVariables.yN[ i ] = 0.0;
	}


	if( VERBOSE ) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );
}

// real init state function of mpc controller
void mpc::init_mpc_fun()
{
	//init x0 , same as input.x0
	acadoVariables.x0[0]  = init_position.x;//init position x
	acadoVariables.x0[1]  = init_position.y;//init position y
	acadoVariables.x0[2]  = init_position.z;//init position z

	acadoVariables.x0[3]  = init_euler_angles.x;//init ang x
	acadoVariables.x0[4]  = init_euler_angles.y;//init ang y
	acadoVariables.x0[5]  = init_euler_angles.z;//init ang z

	acadoVariables.x0[6]  = init_velocity.x;//init vel x
	acadoVariables.x0[7]  = init_velocity.y;//init vel y
	acadoVariables.x0[8]  = init_velocity.z;//init vel z

	acadoVariables.x0[9]  = init_body_rate.x; //init ang_vel x
	acadoVariables.x0[10] = init_body_rate.y;//init ang_vel y
	acadoVariables.x0[11] = init_body_rate.z;//init ang_vel z
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
	for (int i = 0; i < NU; i++)
	{
		/* code */
		u0[i]=0;
	}	
	// u0[0] = nominal_position.x;
	// u0[1] = nominal_position.y;
	// u0[2] = nominal_position.z;
	// u0[3] = nominal_euler_angles.x;
	// u0[4] = nominal_euler_angles.y;
	// u0[5] = nominal_euler_angles.z;

	// init u
	for (int i = 0; i < NU; i++)
	{
		for (int j = 0; i < N; i++)
		{
			acadoVariables.u[i+j*NU]=u0[i];
		}
	}
	
	//Initialize the measurements/reference.
	double ref[NY]; 
	ref[0] = nominal_position.x;
	ref[1] = nominal_position.y;
	ref[2] = nominal_position.z;
	ref[3] = nominal_euler_angles.x;
	ref[4] = nominal_euler_angles.y;
	ref[5] = nominal_euler_angles.z;
	// initialize the reference remain
	for (int i = 6; i < NY; i++)
	{
		ref[0] = 0;
	}
	for (int i = 0; i < NY; i++)
	{
		for (int j = 0; i < N; i++)
		{
			acadoVariables.u[i+j*NY]=ref[i];
		}
	}

	//Initialize the referenceN
	acadoVariables.yN[0] = nominal_position.x;
	acadoVariables.yN[1] = nominal_position.y;
	acadoVariables.yN[2] = nominal_position.z;
	acadoVariables.yN[3] = nominal_euler_angles.x;
	acadoVariables.yN[4] = nominal_euler_angles.y;
	acadoVariables.yN[5] = nominal_euler_angles.z;
	// acadoVariables.yN[0] = 0;
	// acadoVariables.yN[1] = 0;
	// acadoVariables.yN[2] = -2;
	// acadoVariables.yN[3] = 0;
	// acadoVariables.yN[4] = 0;
	// acadoVariables.yN[5] = 0;
	//ROS_INFO_STREAM("yN:"<<acadoVariables.yN[0]);
}
//the solver of mpc controller,need to be called at every loop
void mpc::mpc_solver()
{
	acadoVariables.x0[0]  = main_position.x;//init position x
	acadoVariables.x0[1]  = main_position.y;//init position y
	acadoVariables.x0[2]  = main_position.z;//init position z

	acadoVariables.x0[3]  = main_eular_angles.x;//init ang x
	acadoVariables.x0[4]  = main_eular_angles.y;//init ang y
	acadoVariables.x0[5]  = main_eular_angles.z;//init ang z

	acadoVariables.x0[6]  = main_velocity.x;//init vel x
	acadoVariables.x0[7]  = main_velocity.y;//init vel y
	acadoVariables.x0[8]  = main_velocity.z;//init vel z

	acadoVariables.x0[9]  = main_body_rates.x; //init ang_vel x
	acadoVariables.x0[10] = main_body_rates.y;//init ang_vel y
	acadoVariables.x0[11] = main_body_rates.z;//init ang_vel z

	//reference of route
	double ref[NY]; 
	ref[0] = nominal_position.x;
	ref[1] = nominal_position.y;
	ref[2] = nominal_position.z;
	ref[3] = nominal_euler_angles.x;
	ref[4] = nominal_euler_angles.y;
	ref[5] = nominal_euler_angles.z;
	// initialize the reference remain
	for (int i = 6; i < NY; i++)
	{
		ref[0] = 0;
	}
	for (int i = 0; i < NY; i++)
	{
		for (int j = 0; i < N; i++)
		{
			acadoVariables.u[i+j*NY]=ref[i];
		}
	}
	acado_feedbackStep( );
	/* Optional: shift the initialization (look at acado_common.h). */
	//acado_shiftStates(2, 0, 0); 
	//acado_shiftControls(0); 
	// thrust_u[0] =acadoVariables.u[0];
	// thrust_u[1] =acadoVariables.u[1];
	// thrust_u[2] =-acadoVariables.u[2];
	// torques_u[0]=acadoVariables.u[3];
	// torques_u[1]=acadoVariables.u[4];
	// torques_u[2]=acadoVariables.u[5];

	thrust_u[0] =0.549493;
	thrust_u[1] =-0.923843;
	thrust_u[2] =-625365;
	torques_u[0]=0;
	torques_u[1]=0;
	torques_u[2]=0;

	// ROS_INFO_STREAM("x:"<<thrust_u[0]);
	// ROS_INFO_STREAM("y:"<<thrust_u[1]);
	// ROS_INFO_STREAM("z:"<<thrust_u[2]);

	/* Prepare for the next step. */
	acado_preparationStep();

	acado_printDifferentialVariables();
	acado_printControlVariables();
}
//the distributor of the whole thrust and torque
void mpc::distributor(Eigen::Vector3f thu,Eigen::Vector3f tor)
{
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

//callback funtion define list
//setpoint position, chk 
void mpc::nominal_position_sub_cb(const geometry_msgs::Point::ConstPtr& msg)
{
	nominal_position = *msg;
	//ROS_INFO_STREAM("nominal_position: ");
	//ROS_INFO_STREAM(nominal_position);
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
	main_eular_angles = *msg;
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
	angle1_pub.publish(ang1);
	angle2_pub.publish(ang2);
	angle3_pub.publish(ang3);
	thrust1_pub.publish(thu1);
	thrust2_pub.publish(thu2);
	thrust3_pub.publish(thu3);
}

//timer callback function
void mpc::calc_cb(const ros::TimerEvent&)
{
    //ROS_INFO_STREAM("gaoling. ");
	if(start_pub_att.data)//if start_pub_att == true, then start the mpc controller
	{
		mpc_solver();
		distributor(thrust_u,torques_u);
		output_publish(ang1,ang2,ang3,thu1,thu2,thu3);
	}
}

//main function
int main(int argc, char **argv)
{
    ros::init(argc,argv,"mpc");
    ros::NodeHandle nh;

    mpc mpc_node(&nh);
    ros::spin();
    return 0;
}

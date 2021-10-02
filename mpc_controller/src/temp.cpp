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
	f << dot(pos_x) == pos_vel[0];//position vel
	f << dot(pos_y) == pos_vel[1];
	f << dot(pos_z) == pos_vel[2];
	f << dot(ang_roll) == ang_vel[0];//angle vel
	f << dot(ang_pitch) == ang_vel[1];
	f << dot(ang_yaw) == ang_vel[2];
	f << dot(pos_vel_x) == acc[0];//acc
	f << dot(pos_vel_y) == acc[1];
	f << dot(pos_vel_z) == acc[2];
	f << dot(ang_vel_roll) == ang_acc[0];//ang acc
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
	//position
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
	WN(1,1)= 1;
	WN(2,2)= 1;
	WN(3,3)= 1;
	WN(4,4)= 1;
	WN(5,5)= 1;
	WN(6,6)= 1;

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
	// //init position
	// ocp.subjectTo(AT_START, pos_x==init_position.x);
	// ocp.subjectTo(AT_START, pos_y==init_position.y);
	// ocp.subjectTo(AT_START, pos_z==init_position.z);
	// //init linear velocity
	// ocp.subjectTo(AT_START, pos_vel_x==init_velocity.x);
	// ocp.subjectTo(AT_START, pos_vel_y==init_velocity.y);
	// ocp.subjectTo(AT_START, pos_vel_z==init_velocity.z);	
	// //init attitude
	// ocp.subjectTo(AT_START, ang_roll  ==init_euler_angles.x);
	// ocp.subjectTo(AT_START, ang_pitch ==init_euler_angles.y);
	// ocp.subjectTo(AT_START, ang_yaw   ==init_euler_angles.z);	
	// //init body rate
	// ocp.subjectTo(AT_START, ang_vel_roll  ==init_body_rate.x);
	// ocp.subjectTo(AT_START, ang_vel_pitch ==init_body_rate.y);
	// ocp.subjectTo(AT_START, ang_vel_yaw   ==init_body_rate.z);	

	//SETTING UP THE MPC CONTROLLER:
	OCPexport mpc_obj( ocp);
	mpc_obj.set( HESSIAN_APPROXIMATION,GAUSS_NEWTON);
    mpc_obj.set( DISCRETIZATION_TYPE,  MULTIPLE_SHOOTING );
	mpc_obj.set( QP_SOLVER,QP_QPOASES  ); 
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
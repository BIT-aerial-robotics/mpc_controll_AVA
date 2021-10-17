#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include <stdio.h>

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
#define VERBOSE     1         /* Show iterations: 1, silent: 0.  */
/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

int main(int argc, char **argv)
{
    USING_NAMESPACE_ACADO
    ros::init(argc,argv,"test_mpc");
    ros::NodeHandle n;
	/*
    // INTRODUCE THE VARIABLES:
    // -------------------------
	DifferentialState xB;
	DifferentialState xW;
	DifferentialState vB;
	DifferentialState vW;

	Control R;
	Control F;

	double mB = 350.0;
	double mW = 50.0;
	double kS = 20000.0;
	double kT = 200000.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;

	f << dot(xB) == vB;
	f << dot(xW) == vW;
	f << dot(vB) == ( -kS*xB + kS*xW + F ) / mB;
	f << dot(vW) == (  kS*xB - (kT+kS)*xW + kT*R - F ) / mW;


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;

    h << xB;
    h << xW;
	h << vB;
    h << vW;

    DMatrix Q(4,4);
    Q.setIdentity();
	Q(0,0) = 10.0;
	Q(1,1) = 10.0;

    DVector r(4);
    r.setAll( 0.0 );


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double t_start = 0.0;
    const double t_end   = 1.0;

    OCP ocp( t_start, t_end, 20 );

    ocp.minimizeLSQ( Q, h, r );

	ocp.subjectTo( f );

	ocp.subjectTo( -500.0 <= F <= 500.0 );
	ocp.subjectTo( R == 0.0 );



    // SETTING UP THE (SIMULATED) PROCESS:
    // -----------------------------------
	OutputFcn identity;
	DynamicSystem dynamicSystem( f,identity );

	Process process( dynamicSystem,INT_RK45 );

    // SETTING UP THE MPC CONTROLLER:
    // ------------------------------
	RealTimeAlgorithm alg( ocp,0.05 );
	alg.set( MAX_NUM_ITERATIONS, 2 );
	
	StaticReferenceTrajectory zeroReference;

	Controller controller( alg,zeroReference );

    // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
    // ----------------------------------------------------------
	SimulationEnvironment sim( 0.0,3.0,process,controller );

	DVector x0(4);
	x0(0) = 0.01;
	x0(1) = 0.0;
	x0(2) = 0.0;
	x0(3) = 0.0;
    ROS_INFO_STREAM("gaoling optimal solution");
	if (sim.init( x0 ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );
	if (sim.run( ) != SUCCESSFUL_RETURN)
		exit( EXIT_FAILURE );

    // ...AND PLOT THE RESULTS
    // ----------------------------------------------------------
	VariablesGrid sampledProcessOutput;
	sim.getSampledProcessOutput( sampledProcessOutput );

	VariablesGrid feedbackControl;
	sim.getFeedbackControl( feedbackControl );

	GnuplotWindow window;
	window.addSubplot( sampledProcessOutput(0), "Body Position [m]" );
	window.addSubplot( sampledProcessOutput(1), "Wheel Position [m]" );
	window.addSubplot( sampledProcessOutput(2), "Body Velocity [m/s]" );
	window.addSubplot( sampledProcessOutput(3), "Wheel Velocity [m/s]" );
	window.addSubplot( feedbackControl(1),      "Damping Force [N]" );
	window.addSubplot( feedbackControl(0),      "Road Excitation [m]" );
	window.plot( );
	*/
	
	// Variables:
// 	DifferentialState   p    ;  // the trolley position
// 	DifferentialState   v    ;  // the trolley velocity 
// 	DifferentialState   phi  ;  // the excitation angle
// 	DifferentialState   omega;  // the angular velocity
// 	Control             a    ;  // the acc. of the trolley
// 	const double     g = 9.81;  // the gravitational constant 
// 	const double     b = 0.20;  // the friction coefficient
// 	// Model equations:
// 	DifferentialEquation f; 
// 	f << dot( p ) == v;
// 	f << dot( v ) == a;
// 	f << dot( phi ) == omega;
// 	f << dot( omega ) == -g * sin(phi) - a * cos(phi) - b * omega;
// 	// Reference functions and weighting matrices:
// 	Function h, hN;
// 	h << p << v << phi << omega << a;
// 	hN << p << v << phi << omega;
// 	// Provide defined weighting matrices:
// 	DMatrix W = eye<double>( h.getDim() );
// 	DMatrix WN = eye<double>( hN.getDim() );
// 	WN *= 5;
// 	// Or provide sparsity patterns for the weighting matrices
// //	BMatrix W = eye<bool>( h.getDim() );
// //	BMatrix WN = eye<bool>( hN.getDim() );
// 	//
// 	// Optimal Control Problem
// 	//
// 	OCP ocp(0.0, 3.0, 10);
// 	ocp.subjectTo( f );
// 	ocp.minimizeLSQ(W, h);
// 	ocp.minimizeLSQEndTerm(WN, hN);
// 	ocp.subjectTo( -1.0 <= a <= 1.0 );
// 	ocp.subjectTo( -0.5 <= v <= 1.5 );
// 	// Export the code:
// 	OCPexport mpc( ocp );
// 	mpc.set( HESSIAN_APPROXIMATION,       GAUSS_NEWTON    );
// 	mpc.set( DISCRETIZATION_TYPE,         SINGLE_SHOOTING );
// 	mpc.set( INTEGRATOR_TYPE,             INT_RK4         );
// 	mpc.set( NUM_INTEGRATOR_STEPS,        30              );
// 	mpc.set( QP_SOLVER,                   QP_QPOASES      );
// // 	mpc.set( HOTSTART_QP,                 YES             );
// // 	mpc.set( LEVENBERG_MARQUARDT,         1.0e-4          );
// 	mpc.set( GENERATE_TEST_FILE,          YES             );
// 	mpc.set( GENERATE_MAKE_FILE,          YES             );
// 	mpc.set( GENERATE_MATLAB_INTERFACE,   YES             );
// 	mpc.set( GENERATE_SIMULINK_INTERFACE, YES             );
// // 	mpc.set( USE_SINGLE_PRECISION,        YES             );
// 	if (mpc.exportCode( "getting_started_export" ) != SUCCESSFUL_RETURN)
// 		exit( EXIT_FAILURE );
// 	mpc.printDimensionsQP( );
	
	/* Some temporary variables. */
	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = 0.0;

	/* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = 0.1;
#endif

	if( VERBOSE ) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );

	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{
	
        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		//if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

		/* Optional: shift the initialization (look at acado_common.h). */
        acado_shiftStates(2, 0, 0); 
		acado_shiftControls( 0 ); 

		ROS_INFO_STREAM("control:"<<acadoVariables.u[0]);
		//acado_printControlVariables();
		ROS_INFO_STREAM("x:");
		acado_printDifferentialVariables();
		ROS_INFO_STREAM("x0:");
		ROS_INFO_STREAM(
			" "<<acadoVariables.x0[0]<<
			" "<<acadoVariables.x0[1]<<
			" "<<acadoVariables.x0[2]<<
			" "<<acadoVariables.x0[3]
		);

		ROS_INFO_STREAM("y:");
		for (int i = 0; i < ACADO_N; i++)
		{
			for (int j = 0; j < ACADO_NY; j++)
			{
				printf("\t%e", acadoVariables.y[i * ACADO_NY + j]);
			}
			printf("\n");
		}
		
		ROS_INFO_STREAM("yN:");
		ROS_INFO_STREAM(
			" "<<acadoVariables.yN[0]<<
			" "<<acadoVariables.yN[1]<<
			" "<<acadoVariables.yN[2]<<
			" "<<acadoVariables.yN[3]
		);

		ROS_INFO_STREAM("states:");
		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 5; j++)
			{
				printf("\t%e", acadoWorkspace.state[i * 5 + j]);
			}
			printf("\n");
		}
		// ROS_INFO_STREAM(
		// 	" "<<acadoWorkspace.state[0]<<
		// 	" "<<acadoWorkspace.state[1]<<
		// 	" "<<acadoWorkspace.state[2]<<
		// 	" "<<acadoWorkspace.state[3]<<
		// 	" "<<acadoWorkspace.state[4]<<
		// 	" "<<acadoWorkspace.state[5]<<
		// 	" "<<acadoWorkspace.state[6]<<
		// 	" "<<acadoWorkspace.state[7]<<
		// 	" "<<acadoWorkspace.state[8]<<
		// 	" "<<acadoWorkspace.state[9]<<
		// 	" "<<acadoWorkspace.state[10]<<
		// 	" "<<acadoWorkspace.state[11]<<
		// 	" "<<acadoWorkspace.state[12]<<
		// 	" "<<acadoWorkspace.state[13]<<
		// 	" "<<acadoWorkspace.state[14]<<
		// 	" "<<acadoWorkspace.state[15]<<
		// 	" "<<acadoWorkspace.state[16]<<
		// 	" "<<acadoWorkspace.state[17]<<
		// 	" "<<acadoWorkspace.state[18]<<
		// 	" "<<acadoWorkspace.state[19]<<
		// 	" "<<acadoWorkspace.state[20]<<
		// 	" "<<acadoWorkspace.state[21]<<
		// 	" "<<acadoWorkspace.state[22]<<
		// 	" "<<acadoWorkspace.state[23]<<
		// 	" "<<acadoWorkspace.state[24]
		// );
		// real_t evGx[ 160 ];this variable may be the evaulate backbround
		// for (i = 0; i < ACADO_N; ++i)
		// {
		// 	for (j = 0; j < ACADO_NU; ++j)
		// 		printf("\t%e", acadoVariables.u[i * ACADO_NU + j]);
		// 	printf("\n");
		// }
		// for (int i = 0; i < count; i++)
		// {
		// 	/* code */
		// }
		
		/* Prepare for the next step. */
		acado_preparationStep();
	}
	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if( !VERBOSE )
	printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	// acado_printDifferentialVariables();
	acado_printControlVariables();
	ros::Rate rate(30.0);

	while (ros::ok())
    {

		/* Perform the feedback step. */
		// acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		// if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );
		// ++iter;
		/* Optional: shift the initialization (look at acado_common.h). */
        /* acado_shiftStates(2, 0, 0); */
		/* acado_shiftControls( 0 ); */

		/* Prepare for the next step. */
		// acado_preparationStep();
		// acado_printDifferentialVariables();
	 	// acado_printControlVariables();
        ///ROS_INFO_STREAM("gaoling optimal solution");
        ros::spinOnce();
		//rate.sleep();
    }
    
    return 0;
}
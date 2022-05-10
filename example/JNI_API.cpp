#ifdef BUILDJNI
#if BUILDJNI

#include <PVTP/Utilities.hpp>
#include <PVTP/JNI_API.hpp>
#include <PVTP/ScenarioUser.hpp>
#include <PVTP/ScenarioEvaluator.hpp>

using namespace PVTP;
using namespace SCIMP_Scenario;

/*
 * Class:     PVTPlanner_JNI_API
 * Method:    GetNaiveBottleneckPT_Obstacles
 * Signature: (DDDDDDDDDDDDDD[[D)[[D
 */
JNIEXPORT jobjectArray JNICALL Java_PVTPlanner_JNI_1API_GetNaiveBottleneckPT_1Obstacles
(JNIEnv *env,
 jclass JNI_APIClass,
 jdouble v_i_min,
 jdouble v_i_max,
 jdouble v_f_min,
 jdouble v_f_max,
 jdouble x_limit,
 jdouble t_limit,
 jdouble v_min,
 jdouble v_max,
 jdouble a_min,
 jdouble a_max,
 jdouble epsilon,
 jdouble cur_vel,
 jdouble u,
 jdouble time_horizon,
 jobjectArray J_O
 ) {

	// if errors are generated, make sure we have enough information to do
	// useful debugging
	std::cerr.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
	
	// java-friendly storage for PT Obstacles
	jobjectArray O_return = NULL;
	
	// build constraints
	try {
		
		Constraints c( (double)x_limit,
					  (double)t_limit,
					  (double)v_min,
					  (double)v_max,
					  (double)a_min,
					  (double)a_max,
					  (double)epsilon );
		
		// read in obstacle information
		jsize len = env->GetArrayLength( J_O );
		
		// storage for obstacles
		std::vector< std::vector<double> > C_O( len );
		for ( jsize i=0; i<len; i++ ) {
			jdoubleArray dblArr = (jdoubleArray)env->GetObjectArrayElement( J_O, i );
			jsize dblArr_len = env->GetArrayLength( dblArr );
			for ( jsize j=0; j<dblArr_len; j++ ) {
				jdouble el;
				env->GetDoubleArrayRegion( dblArr, j, 1, &el );
				C_O.at(i).push_back( (double)el );
			}
		}
		
		// build PVT obstacles
		PVT_ObstacleSet O( C_O, c );
		
		// if there are no obstacles, return
		if ( O.obstacles->empty() ) {
			return O_return;
		}
		
		// determine set of important obstacles
		std::set<PVT_Obstacle*> inCollision;
		Evaluator::naiveSafetyTest( inCollision, (double)cur_vel, (double)u, (double)time_horizon, O, c );
		
		// storage for java-friendly obstacle set
		jclass dblArrCls = env->FindClass("[D");
		if (dblArrCls == NULL) {
			std::cerr << "JNI: Failed to find Java double array class. (1)" << std::endl;
			return O_return;
		}
		O_return = (jobjectArray)env->NewObjectArray( (jsize)(inCollision.size()),
													 dblArrCls,
													 NULL );
		
		// add bottleneck obstacles
		size_t i = 0;
		std::set<PVT_Obstacle*>::iterator it;
		for ( it=inCollision.begin(); it!=inCollision.end(); it++ ) {
			PVT_Obstacle& o = **it;
			jdoubleArray o_return = env->NewDoubleArray( 4 );
			
			jdouble min_x = (jdouble)o.getMinPathCoord();
			jdouble max_x = (jdouble)o.getMaxPathCoord();
			jdouble min_t = (jdouble)o.getMinTimeCoord();
			jdouble max_t = (jdouble)o.getMaxTimeCoord();
			
			env->SetDoubleArrayRegion( o_return, 0, 1, &min_x );
			env->SetDoubleArrayRegion( o_return, 1, 1, &max_x );
			env->SetDoubleArrayRegion( o_return, 2, 1, &min_t );
			env->SetDoubleArrayRegion( o_return, 3, 1, &max_t );
			
			env->SetObjectArrayElement( O_return, i++, o_return );
		}
		
		return O_return;
		
	} catch ( int e ) {
		
		Constraints::exceptionMessage( e );
		return O_return;
	}
}

/*
 * Class:     PVTPlanner_JNI_API
 * Method:    GetImportantPT_Obstacles
 * Signature: (DDDDDDDDDDD[[D)[[D
 */
JNIEXPORT jobjectArray JNICALL Java_PVTPlanner_JNI_1API_GetBottleneckPT_1Obstacles
(JNIEnv *env,
 jclass JNI_APIClass,
 jdouble v_i_min,
 jdouble v_i_max,
 jdouble v_f_min,
 jdouble v_f_max,
 jdouble x_limit,
 jdouble t_limit,
 jdouble v_min,
 jdouble v_max,
 jdouble a_min,
 jdouble a_max,
 jdouble epsilon,
 jobjectArray J_O
 ) {
	
	// if errors are generated, make sure we have enough information to do
	// useful debugging
	std::cerr.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
	
	// java-friendly storage for PT Obstacles
	jobjectArray O_return = NULL;
	
	// build constraints
	try {
		
		Constraints c( (double)x_limit,
					  (double)t_limit,
					  (double)v_min,
					  (double)v_max,
					  (double)a_min,
					  (double)a_max,
					  (double)epsilon );
		
		// read in obstacle information
		jsize len = env->GetArrayLength( J_O );

		// storage for obstacles
		std::vector< std::vector<double> > C_O( len );
		for ( jsize i=0; i<len; i++ ) {
			jdoubleArray dblArr = (jdoubleArray)env->GetObjectArrayElement( J_O, i );
			jsize dblArr_len = env->GetArrayLength( dblArr );
			for ( jsize j=0; j<dblArr_len; j++ ) {
				jdouble el;
				env->GetDoubleArrayRegion( dblArr, j, 1, &el );
				C_O.at(i).push_back( (double)el );
			}
		}

		// build PVT obstacles
		PVT_ObstacleSet O( C_O, c );

		// if there are no obstacles, return
		if ( O.obstacles->empty() ) {
			return O_return;
		}
		
		// interval of available initial velocities
		Interval V_i( v_i_min, v_i_max );
		
		// interval of acceptable final velocities
		Interval V_f( v_f_min, v_f_max );
		
		// determine set of important obstacles
		std::set<PVT_Obstacle*> inCollision;
		Evaluator::detectBottleneckICS( inCollision, V_i, V_f, O, c );

		// if there are no important obstacles
		if ( inCollision.empty() ) {
			return O_return;
		}
		
		// storage for java-friendly obstacle set
		jclass dblArrCls = env->FindClass("[D");
		if (dblArrCls == NULL) {
			std::cerr << "JNI: Failed to find Java double array class. (2)" << std::endl;
			return O_return;
		}
		O_return = (jobjectArray)env->NewObjectArray( (jsize)(inCollision.size()),
												  dblArrCls,
												  NULL );
		
		// add bottleneck obstacles
		size_t i = 0;
		std::set<PVT_Obstacle*>::iterator it;
		for ( it=inCollision.begin(); it!=inCollision.end(); it++ ) {
			PVT_Obstacle& o = **it;
			jdoubleArray o_return = env->NewDoubleArray( 4 );
			
			jdouble min_x = (jdouble)o.getMinPathCoord();
			jdouble max_x = (jdouble)o.getMaxPathCoord();
			jdouble min_t = (jdouble)o.getMinTimeCoord();
			jdouble max_t = (jdouble)o.getMaxTimeCoord();
			
			env->SetDoubleArrayRegion( o_return, 0, 1, &min_x );
			env->SetDoubleArrayRegion( o_return, 1, 1, &max_x );
			env->SetDoubleArrayRegion( o_return, 2, 1, &min_t );
			env->SetDoubleArrayRegion( o_return, 3, 1, &max_t );
			
			env->SetObjectArrayElement( O_return, i++, o_return );
		}

		return O_return;
		
	} catch ( int e ) {
		
		Constraints::exceptionMessage( e );
		return O_return;
	}
}

/*
 * Class:     PVTPlanner_JNI_API
 * Method:    TestScenario
 * Signature: (DDDDDDDDDDD[[D)I
 */
JNIEXPORT jint JNICALL Java_PVTPlanner_JNI_1API_TestScenario
(JNIEnv *env,
 jclass JNI_APIClass,
 jdouble v_i_min,
 jdouble v_i_max,
 jdouble v_f_min,
 jdouble v_f_max,
 jdouble x_limit,
 jdouble t_limit,
 jdouble v_min,
 jdouble v_max,
 jdouble a_min,
 jdouble a_max,
 jdouble epsilon,
 jobjectArray J_O
 ) {
	
	// if errors are generated, make sure we have enough information to do
	// useful debugging
	std::cerr.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
	
	// build constraints
	try {

		Constraints c( (double)x_limit,
					  (double)t_limit,
					  (double)v_min,
					  (double)v_max,
					  (double)a_min,
					  (double)a_max,
					  (double)epsilon );
		
		// read in obstacle information
		jsize len = env->GetArrayLength( J_O );

		// storage for obstacles
		std::vector< std::vector<double> > C_O( len );
		for ( jsize i=0; i<len; i++ ) {
			jdoubleArray dblArr = (jdoubleArray)env->GetObjectArrayElement( J_O, i );
			jsize dblArr_len = env->GetArrayLength( dblArr );
			for ( jsize j=0; j<dblArr_len; j++ ) {
				jdouble el;
				env->GetDoubleArrayRegion( dblArr, j, 1, &el );
				C_O.at(i).push_back( (double)el );
			}
		}

		// build PVT obstacles
		PVT_ObstacleSet O( C_O, c );
		
		// storage for forward-propagated intervals
		std::vector<PVT_G*> G;
		
		// storage for goal-propagated intervals
		std::vector<PVT_S*> Goal;
		
		// interval of available initial velocities
		Interval V_i( v_i_min, v_i_max );
		
		// interval of acceptable final velocities
		Interval V_f( v_f_min, v_f_max );
		
		// run algorithm
		Planner::Forward( G, Goal, V_i, V_f, O, c );
		
		// this determines reachability
		return Goal.size();
		
	} catch ( int e ) {
		
		Constraints::exceptionMessage( e );
		return false;
	}
}

/*
 * Class:     PVTPlanner_JNI_API
 * Method:    GetOptimalTrajectory
 * Signature: (DDDDDDDDDDDD[[D)[[D
 */
JNIEXPORT jobjectArray JNICALL Java_PVTPlanner_JNI_1API_GetOptimalTrajectory
(JNIEnv *env,
 jclass JNI_APIClass,
 jdouble v_i_min,
 jdouble v_i_max,
 jdouble v_f_min,
 jdouble v_f_max,
 jdouble x_limit,
 jdouble t_limit,
 jdouble v_min,
 jdouble v_max,
 jdouble a_min,
 jdouble a_max,
 jdouble epsilon,
 jobjectArray J_O
 ) {

	jobjectArray route = NULL;
	
	// if errors are generated, make sure we have enough information to do
	// useful debugging
	std::cerr.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
	
	// build constraints
	try {

		Constraints c( (double)x_limit,
					  (double)t_limit,
					  (double)v_min,
					  (double)v_max,
					  (double)a_min,
					  (double)a_max,
					  (double)epsilon );

		// read in obstacle information
		jsize len = env->GetArrayLength( J_O );

		// storage for obstacles
		std::vector< std::vector<double> > C_O( len );
		for ( jsize i=0; i<len; i++ ) {
			jdoubleArray dblArr = (jdoubleArray)env->GetObjectArrayElement( J_O, i );
			jsize dblArr_len = env->GetArrayLength( dblArr );
			for ( jsize j=0; j<dblArr_len; j++ ) {
				jdouble el;
				env->GetDoubleArrayRegion( dblArr, j, 1, &el );
				C_O.at(i).push_back( (double)el );
			}
		}

		// build PVT obstacles
		PVT_ObstacleSet O( C_O, c );
		
		// storage for forward-propagated intervals
		std::vector<PVT_G*> G;
		
		// storage for goal-propagated intervals
		std::vector<PVT_S*> Goal;
		
		// interval of available initial velocities
		Interval V_i( v_i_min, v_i_max );
		
		// interval of acceptable final velocities
		Interval V_f( v_f_min, v_f_max );
		
		// run algorithm
		if ( !Planner::Forward( G, Goal, V_i, V_f, O, c ) ) {
			return route;
		}
		
		// goal unreachable
		if ( Goal.size() == 0 ) {
			return route;
		}

		// storage for trajectory
		std::vector<TrajectorySegment*> T;
		
		// derive optimal trajectory
		if ( !Planner::BuildOptimalTrajectory( T, G, Goal, O, c ) ) {
			return route;
		}
		
		jclass dblArrCls = env->FindClass("[D");
		if (dblArrCls == NULL) {
			std::cerr << "JNI: Failed to find Java double array class. (3)" << std::endl;
			return route;
		}

		// storage for java-friendly trajectory
		route = (jobjectArray)env->NewObjectArray( (jsize)(T.size() + 1),
													dblArrCls,
													NULL );

		for ( size_t i=0; i<T.size(); i++ ) {
			jdoubleArray traj = env->NewDoubleArray( 3 );
			jdouble x = (jdouble)T.at(i)->getInitialState().getPathCoord();
			jdouble t = (jdouble)T.at(i)->getInitialState().getTimeCoord();
			double v_tmp = T.at(i)->getInitialState().getVelocityCoord();
			jdouble v = (jdouble)Maths::clipToZero( v_tmp, Constants::CLIP_TO_ZERO );
			env->SetDoubleArrayRegion( traj, 0, 1, &x );
			env->SetDoubleArrayRegion( traj, 1, 1, &t );
			env->SetDoubleArrayRegion( traj, 2, 1, &v );
			env->SetObjectArrayElement( route, i, traj );
		}
		jdoubleArray traj = env->NewDoubleArray( 3 );
		jdouble x = (jdouble)T.back()->getFinalState().getPathCoord();
		jdouble t = (jdouble)T.back()->getFinalState().getTimeCoord();
		double v_tmp = T.back()->getFinalState().getVelocityCoord();
		jdouble v = (jdouble)Maths::clipToZero( v_tmp, Constants::CLIP_TO_ZERO );
		env->SetDoubleArrayRegion( traj, 0, 1, &x );
		env->SetDoubleArrayRegion( traj, 1, 1, &t );
		env->SetDoubleArrayRegion( traj, 2, 1, &v );
		env->SetObjectArrayElement( route, (jsize)T.size(), traj );
		
		// clean memory
		Utilities::CleanTrajectory( T );
		Utilities::CleanResults( G, Goal );
		
		// return constructed route
		return route;
		
	} catch ( int e ) {

		Constraints::exceptionMessage( e );
		return route;
	}
}

JNIEXPORT jdouble JNICALL Java_PVTPlanner_JNI_1API_GetOptimalControl
(JNIEnv *env,
 jclass JNI_APIClass,
 jdouble _time_step,
 jdouble v_i_min,
 jdouble v_i_max,
 jdouble v_f_min,
 jdouble v_f_max,
 jdouble x_limit,
 jdouble t_limit,
 jdouble v_min,
 jdouble v_max,
 jdouble a_min,
 jdouble a_max,
 jdouble epsilon,
 jobjectArray J_O
 ) {
	
	double time_step = (double)_time_step;
	jdouble control = 2 * a_max;
	
	// if errors are generated, make sure we have enough information to do
	// useful debugging
	std::cerr.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
	
	// build constraints
	try {
		
		Constraints c( (double)x_limit,
					  (double)t_limit,
					  (double)v_min,
					  (double)v_max,
					  (double)a_min,
					  (double)a_max,
					  (double)epsilon );
		
		// read in obstacle information
		jsize len = env->GetArrayLength( J_O );
		
		// storage for obstacles
		std::vector< std::vector<double> > C_O( len );
		for ( jsize i=0; i<len; i++ ) {
			jdoubleArray dblArr = (jdoubleArray)env->GetObjectArrayElement( J_O, i );
			jsize dblArr_len = env->GetArrayLength( dblArr );
			for ( jsize j=0; j<dblArr_len; j++ ) {
				jdouble el;
				env->GetDoubleArrayRegion( dblArr, j, 1, &el );
				C_O.at(i).push_back( (double)el );
			}
		}
		
		// build PVT obstacles
		PVT_ObstacleSet O( C_O, c );
		
		// storage for forward-propagated intervals
		std::vector<PVT_G*> G;
		
		// storage for goal-propagated intervals
		std::vector<PVT_S*> Goal;
		
		// interval of available initial velocities
		Interval V_i( v_i_min, v_i_max );
		
		// interval of acceptable final velocities
		Interval V_f( v_f_min, v_f_max );
		
		// run algorithm
		if ( !Planner::Forward( G, Goal, V_i, V_f, O, c ) ) {
			return control;
		}
		
		// goal unreachable
		if ( Goal.size() == 0 ) {
			return control;
		}
		
		// storage for trajectory
		std::vector<TrajectorySegment*> T;
		
		// derive optimal trajectory
		if ( !Planner::BuildOptimalTrajectory( T, G, Goal, O, c ) ) {
			return control;
		}
		
		// compute optimal executable control
		control = Utilities::getOptimalExecutableControl( T, time_step );
		
		// verify that it's within range; if not, pick the closest one
		if ( !c.validA(control) ) {
			double min_diff = control - c.getAMin();
			double max_diff = control - c.getAMax();
			if ( min_diff < max_diff ) {
				control = c.getAMin();
			} else {
				control = c.getAMax();
			}
		}
		
		// clean memory
		Utilities::CleanTrajectory( T );
		Utilities::CleanResults( G, Goal );
		
		return control;

		
	} catch ( int e ) {
		
		Constraints::exceptionMessage( e );
		return control;
	}
	
}

#endif
#endif

#ifndef PVTP_PLANNER_H
#define PVTP_PLANNER_H

/*! \mainpage Introduction
 *
 * \section PVT Trajectory Planner
 *
 * The PVTP algorithm plans time-optimal trajectories along a known path given
 * a set of path-time obstacles, and velocity and acceleration constraints. This
 * package contains both the planner, and an API for constructing and evaluating
 * scenarios that can then be fed into the planner. Further, the API supports
 * computing a safe control given a scenario and user-specified control
 * according to a pre-defined notion of safety.
 *
 * Because the algorithm is exact and deals with real-valued functions, the
 * discretization errors introduced by floating point representations can cause
 * problems. Further, the algorithm relies on correct propagation of exact
 * values, which allows errors to accumulate. For this reason there is an 
 * epsilon value specified in the Constants.hpp file that allows the algorithm
 * to perform approximate comparisons when necessary. This should allow the
 * algorithm to function normally for most scenarios, but there are inevitably
 * cases where, for one reason or another, accumulated error results in
 * unexpected values. When this happens, the algorithm emits an error message
 * to stderr, and tries to fail gracefully. If an error is encountered that
 * is not attributable to discretization error, it is considered fatal and
 * the algorithm halts.
 *
 * I am always working on improving the robustness of the algorithm, but
 * avoiding all errors simply isn't possible.
 *
 * \section install_sec Installation
 *
 * \subsection step1 Step 1: Set Build Parameters
 * 
 * In Makefile.config are several build parameters that can be set, but probably
 * the only thing you'll need to choose is whether to build static or dynamic
 * libraries. (Note that to use the JNI functionality, dynamic libraries must be
 * built.)
 *
 * \subsection step2 Step 2: Build Library
 *
 * To build the C++ library and scenario API, edit Makefile.config to choose
 * static or shared library, then run make. To build the test scenario file,
 * run:
 *
 * make
 * make scenario_test
 *
 * I've tried to document the code fairly well, especially the scenario code
 * (everything in the SCIMP_Scenario namespace). For more information about the
 * API, and an example of usage, see scenario_test.cpp.
 *
 * There is a README file in PVTP/Matlab that describes the Matlab code.
 */

#include <PVTP/PVT_Point.hpp>
#include <PVTP/TrajectorySegment.hpp>
#include <PVTP/Interval.hpp>
#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/Constants.hpp>
#include <PVTP/Collisions.hpp>
#include <PVTP/PVT_S.hpp>
#include <PVTP/PVT_G.hpp>

namespace PVTP {
	
	/**
	 * This namespace contains API functions specific to the library. Any
	 * function here with a bool return type returns 'true' if the routine
	 * terminated successfully, or 'false' if some unexpected error occurred.
	 *
	 * Note that the return value only indicates whether errors were
	 * encountered, not whether something was successfully found. This means,
	 * for example, that NextReachableSet would return true even if it couldn't
	 * find a reachable set; that would only indicate that it successfully
	 * determined that there is no reachable set.
	 */
	namespace Planner {
		
		/**
		 * The backward propagation routine. This routine utilitizes Forward
		 * to run propagation backwards through the obstacle field towards the
		 * initial point (passed in as p_end). The resulting set of reachable
		 * velocities is intersected with those found by Forward to produce 
		 * the set of velocities at each point for which goal-reachable
		 * trajectories exist.
		 *
		 * It is important to release memory used by the arguments to this
		 * procedure when they are no longer needed. Use the functions provided
		 * in Utilities to do so.
		 */
		bool Backward( std::vector<PVT_G*>& G_intersect,
					  std::vector<PVT_G*>& G_forward,
					  std::vector<PVT_S*>& Goal_forward,
					  Interval& V_f,
					  PVT_ObstacleSet& O_forward,
					  Constraints& c_forward );
		
		/**
		 * The forward propagation routine. This routine takes an initial
		 * PVT point, initial velocity interval, desired final velocity
		 * interval, set of obstacles, and set of constraints, and calculates
		 * the reachable velocities at all obstacle vertices, G, and goal
		 * region, Goal.
		 *
		 * It is important to release memory used by the arguments to this
		 * procedure when they are no longer needed. Use the functions provided
		 * in Utilities to do so.
		 */
		bool Forward( std::vector<PVT_G*>& G,
					 std::vector<PVT_S*>& Goal,
					 Interval& V_i,
					 Interval& V_f,
					 PVT_ObstacleSet& O,
					 Constraints& c,
					 bool do_goal_propagation = true,
					 bool add_origin = true );
		
		/**
		 * The propagation routine. This routine propagates the outbound
		 * velocity set from one point, p1, to the reachable set of another, p2.
		 * The results of the propagation are stored as an element of S.
		 *
		 * This is used in a loop that computes reachable sets at p2 from all
		 * points previous to p2 in time. Once those are computed and stored in
		 * S, the Merge routine computes the final reachable sets at p2.
		 */
		bool Propagate( std::vector<PVT_S*>& S,
					   PVT_Point& p1,
					   PVT_Point& p2,
					   Interval& V_int,
					   PVT_ObstacleSet& O,
					   std::vector<PVT_ObstaclePoint*>& P_t,
					   Constraints& c );
		
		/**
		 * The goal propagation routine. This routine is essentially the same
		 * as the propagation routine, save that it propagates to the goal
		 * region instead of a specified point.
		 */
		bool PropagateGoal( std::vector<PVT_S*>& S_goal,
						   PVT_ObstaclePoint& p1,
						   Interval& V_i,
						   Interval& V_f,
						   PVT_ObstacleSet& O,
						   std::vector<PVT_ObstaclePoint*>& P_t,
						   Constraints& c );
						   
		
		/**
		 * The channel classifier. This routine generates a bit set that
		 * identifies the channel of a trajetory in an obstacle set, and
		 * stores it in hClass. An empty hClass upon return indicates a
		 * trajectory in collision. Classification looks at the set of obstacles
		 * occurring within the time range of the trajectory, and classifies
		 * according to whether a given obstacle occurs before or after the
		 * the trajectory in the path dimension.
		 *
		 * By convention, any trajectory whose time range is devoid of obstacles
		 * belongs to the Constants::H_CLASS_ORIGIN class.
		 *
		 * WARNING: This routine requires P be sorted by time coordinate! The
		 * PVT_ObstaclePoint header file contains a comparator for such sorting.
		 */
		bool Channel( std::vector<char>& hClass,
					   std::vector<TrajectorySegment*>& T,
					   PVT_ObstacleSet& O,
					   std::vector<PVT_ObstaclePoint*>& P,
					   Constraints& c );
		
		/**
		 * This routine determines whether a bit field, B, is the suffix of
		 * another bit field, B_prime.
		 *
		 * This routine is used to merge homotopic classes, where the bit field
		 * is the hClass generated by Homotopic.
		 */
		bool isSuffix( std::vector<char>& B, std::vector<char>& B_prime );
		
		/**
		 * The merge routine. This routine takes sets of reachable velocities
		 * paired with their homotpic classes and merges those that belong to
		 * the same class.
		 *
		 * The merged velocity intervals are stored in V.
		 */
		void Merge( std::vector<Interval*>& V, std::vector<PVT_S*>& S, Constraints& c );
		
		/**
		 * The goal merge routine. This routine is essentially the same as the
		 * merge routine, save that it merges trajectories at the goal region
		 * instead of reachable velocity sets at a point.
		 */
		void MergeGoal( std::vector<PVT_S*>& Goal, std::vector<PVT_S*>& S_goal, Constraints& c );
		
		/**
		 * The next reachable set routine. This routine calculates the reachable
		 * set of velocities at a given point from a given point with an initial
		 * set of velocities. The routine returns two representative trajectories,
		 * one terminating in the maximum attainable velocity, and one the
		 * minimum.
		 *
		 * In Reachable, the first element of the pair is the max velocity,
		 * the second element the min velocity trajectory. It's good practice
		 * to set the 'first' and 'second' members to NULL before calling, and
		 * then testing for NULL afterwards; a NULL indicates not reachable
		 * (assuming the routine terminated correctly, indicated by a return 
		 * value of true).
		 *
		 * Reachable sets are calculated by constructing representative
		 * trajectories of the form PLP, which are curves of at most three
		 * segments, where the P segments are parabolic, and L are linear. P
		 * segments are assumed to have second derivative equal to max/min
		 * acceleration, and neither P nor L segments may not have slope
		 * exceeding max/min velocity. Careful construction of these
		 * trajectories yields representative trajectories originating at p1
		 * and terminating at p2 with max/min reachable velocities.
		 */
		bool NextReachableSet( std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* >& Reachable,
							  PVT_Point& p1,
							  PVT_Point& p2,
							  Interval& V_i,
							  Constraints& c );
		
		/**
		 * The initial velocity routine. Similar to the next reachable set
		 * routine, this routine determine the set of feasible initial
		 * velocities with which a point p2 can be reached from a point p1.
		 */
		bool FindInitialVelocityRange( Interval& V_i, PVT_Point& p1, PVT_Point& p2, Constraints& c );
		
		/**
		 * Given an initial state and a destination PVT point, this routine will
		 * return a representative trajectory whose final velocity is the
		 * maximum achievable velocity given the constraints.
		 */
		bool UpperBoundingStates( std::vector<TrajectorySegment*>& T,
								 PVT_State& s_i,
								 PVT_Point& p,
								 Constraints& c );
		
		
		/**
		 * Given an initial state and a destination PVT point, this routine will
		 * return a representative trajectory whose final velocity is the
		 * minimum achievable velocity given the constraints.
		 */
		bool LowerBoundingStates( std::vector<TrajectorySegment*>& T,
								 PVT_State& s_i,
								 PVT_Point& _p,
								 Constraints& c );
		
		/**
		 * Find upper and lower bounding trajectories for connecting to the goal
		 * region.
		 *
		 * UB = Representative trajectory with maximum final velocity
		 * LB = Representative trajectory with minimum final velocity
		 */
		bool GoalConnect( std::vector<TrajectorySegment*>& UB,
						 std::vector<TrajectorySegment*>& LB,
						 PVT_Point& p1,
						 Interval& V_i,
						 Interval& V_f,
						 PVT_ObstacleSet& O,
						 Constraints& c );
		
		/**
		 * Construct a PLP trajectory to the goal region given an initial point,
		 * initial velocity and final velocity.
		 *
		 * The trajectory is stored in T.
		 */
		bool BuildGoalPLP( std::vector<TrajectorySegment*>& T,
						  PVT_Point& p1,
						  Interval& V_i,
						  double vf,
						  Constraints& c );
		
		/**
		 * Construct a PLP trajectory between two states.
		 *
		 * The trajectory is stored in T.
		 */
		bool BuildPLP( std::vector<TrajectorySegment*>& T,
					  PVT_State& s1,
					  PVT_State& s2,
					  Constraints& c );
		
		/**
		 * Given a set of forward propagated intervals, derive the time-optimal
		 * trajectory.
		 *
		 * The trajectory is stored in T.
		 *
		 * It is important to release memory used by the arguments to this
		 * procedure when they are no longer needed. Use the functions provided
		 * in Utilities to do so.
		 */
		bool BuildOptimalTrajectory( std::vector<TrajectorySegment*>& T,
									std::vector<PVT_G*>& G,
									std::vector<PVT_S*>& Goal,
									PVT_ObstacleSet& O,
									Constraints& c );
		

	} // end Planner namespace

} // end PVTP namespace

#endif

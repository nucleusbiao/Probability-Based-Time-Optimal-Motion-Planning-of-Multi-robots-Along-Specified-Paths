#ifndef PVTP_UTILITIES_H
#define PVTP_UTILITIES_H

#include <string>
#include <vector>
#include <PVTP/TrajectorySegment.hpp>
#include <PVTP/PVT_G.hpp>
#include <PVTP/PVT_S.hpp>
#include <PVTP/Planner.hpp>

//#define SHOW_COMMENTS

#ifdef SHOW_COMMENTS
#include <iostream>
#endif

namespace PVTP {
	
	/**
	 * This namespace contains utility functions specific to the library.
	 */
	namespace Utilities {
		
		/**
		 * Perform an intersection of reachable sets on a point-by-point basis
		 * across G1 and G2. Store the result in G_union. This method is used
		 * during backwards propagation to intersect the sets of forward
		 * reachable intervals with backward reachable intervals.
		 *
		 * Assume that G2 contains a subset of the points in G1, and
		 * that they are ordered in the same way
		 */
		bool IntersectionOfReachableSets( std::vector<PVT_G*>& G_union,
										 std::vector<PVT_G*>& G1,
										 std::vector<PVT_G*>& G2,
										 Constraints& c );
		
		/**
		 * Perform a special union of reachable sets on a point-by-point basis
		 * across G1 and G2. Store the result in G1. This method is used
		 * during backwards propagation to accumulate backwards reachable
		 * intervals
		 *
		 * Assume that G2 contains a subset of the points in G1, and
		 * that they are ordered in the same way
		 */
		void SpecialUnionOfReachableSets( std::vector<PVT_G*>& G1,
										 std::vector<PVT_G*>& G2,
										 Constraints& c );
		
		/**
		 * Given a set of obstacles and a new origin, translate the set, 
		 * then reflect about both axes. This procedure is useful when 
		 * performing backwards propagation: transforming the obstacle field
		 * this way allows the existing Forward algorithm to be used.
		 */
		void TranslateAndMirrorObstacleSet( PVT_ObstacleSet& O_backward,
										   PVT_ObstacleSet& O_forward,
										   PVT_Point& p,
										   Constraints& c );
		
		/**
		 * Given a set of reachable velocities, mirror the set of points and
		 * translate to some other origin. This is used during backwards
		 * propagation to undo the transformation performed by
		 * TranslateAndMirrorObstacleSet
		 */
		void MirrorAndTranslateReachableSets( std::vector<PVT_G*>& G,
											 PVT_Point& p );
		
		/**
		 * Clean both node and goal-reachable information from forward propagation.
		 * Convenience method that calls both CleanG and CleanGoal.
		 */
		void CleanResults( std::vector<PVT_G*>& G, std::vector<PVT_S*>& Goal );
		
		/**
		 * Clean the node results from forward propagation
		 */
		void CleanG( std::vector<PVT_G*>& G );
		
		/**
		 * Clean the goal-reachable information from forwad propagation
		 */
		void CleanGoal( std::vector<PVT_S*>& Goal );
		
		/**
		 * A function to free memory associated with a computed trajectory
		 */
		void CleanTrajectory( std::vector<TrajectorySegment*>& T );
		
		/**
		 * A function to remove essentially duplicate state transitions from a trajectory
		 */
		void SanitizeTrajectory( std::vector<TrajectorySegment*>& T, Constraints& c );
		
		/**
		 * Output information from the results of forward propagation.
		 * Convenience method that calls both DescribeG and DescribeGoal.
		 */
		void DescribeResults( std::vector<PVT_G*>& G,
							 std::vector<PVT_S*>& Goal,
							 bool verbose = false );
		
		/**
		 * Output node information from the results of forward propagation
		 */
		void DescribeG( std::vector<PVT_G*>& G, bool verbose = false );
		
		/**
		 * Output goal information from the results of forward propagation
		 */
		void DescribeGoal( std::vector<PVT_S*>& Goal, bool verbose = false );
		
		/**
		 * Print out a full trajectory
		 */
		void PrintTrajectory( std::vector<TrajectorySegment*>& T );
		
		/**
		 * Print a set of obstacles in [min_x max_x min_t max_t] format
		 */
		void PrintObstacles( PVT_ObstacleSet& obs );
		
		/**
		 * Print a vector of obstacles in [min_x max_x min_t max_t] format
		 */
        void PrintObstacles( std::vector<PVT_Obstacle>& obs );
		
		/**
		 * Print the homotopic signature of a trajectory
		 */
		void PrintSignature( std::vector<char>& hClass );
		
		/**
		 * Construct a velocity interval based on the given representative
		 * trajectories.
		 */
		void ExtractVelocityInterval( Interval& I,
									 std::vector<TrajectorySegment*>& UB,
									 std::vector<TrajectorySegment*>& LB );
		
		/**
		 * Truncate a given value to be within specified bounds
		 */
		double truncateValue( double value, double min, double max );
		
		/**
		 * Given a trajectory and a time into that trajectory, compute the 
		 * velocity at that time.
		 *
		 * For invalid imputs (like an empty trajectory or negative time) this
		 * method return NaN.
		 */
		double trajectoryVelocityAtTime( std::vector<TrajectorySegment*>& T, double time, bool truncate = true );
		
		/**
		 * Given a trajectory and a time into that trajectory, compute the 
		 * displacement at that time.
		 *
		 * For invalid imputs (like an empty trajectory or negative time) this
		 * method return NaN.
		 */
		double trajectoryDisplacementAtTime( std::vector<TrajectorySegment*>& T, double time, bool truncate = true );
		
		/**
		 * Compute an optimal, executable control for a given time step. An executable control
		 * is one with constant acceleration over the time step that minimizes
		 * deviation from the optimal final displacement and velocity. The coefficients C1
		 * and C2 are weights for velocity and displacement, respectively.
		 *
		 * NOTE: This does NOT check whether the control meets constraints
		 * Because the optimization function is linear, truncating the returned
		 * control to the constraint range is the correct way to optimize; therefore,
		 * it is left to the caller (who has access to the constraints object) to do that.
		 */
		double getOptimalExecutableControl( double initial_velocity, double optimal_displacement, double optimal_velocity, double time_step, double C1 = 1., double C2 = 1. );
		
		/**
		 * Overloaded version of above that takes in a trajetory and desired time
		 */
		double getOptimalExecutableControl( std::vector<TrajectorySegment*>& T, double time_step );
		
		/**
		 * Check the executability of a trajectory: a trajectory is executable
		 * if its segment durations are within range of being multiples of the
		 * given time step.
		 */
		bool trajectoryIsExecutable( std::vector<TrajectorySegment*>& T, double time_step, double range );
		
		/**
		 * An adaptation of PHP's explode function
		 * http://www.infernodevelopment.com/perfect-c-string-explode-split
		 */
		void StringExplode( std::string str, std::string separator, std::vector<std::string>& results );
	
	} // end Utilities namespace

} // end PVTP namespace

#endif

#ifndef PVTP_SCENARIO_EVALUATOR_H
#define PVTP_SCENARIO_EVALUATOR_H

#include <set>
#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/TrajectorySegment.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {

	/**
	 * The Evaluator namespace. This namespace contains methods for computing
	 * sets of "important" obstacles in a given scenario that contains
	 * uncertainty. The idea is to look for obstacles correlated with the
	 * navigability of a the scenario. This is done by first padding path-time
	 * obstacles with minimum fore- and aft-ttc values. If an optimal trajectory
	 * is thus found, it is returned, and the scenario is considered "safe."
	 *
	 * If an optimal trajectory is not found, the planner is run again without
	 * the ttc padding. If an optimal trajectory is found, it is returned along
	 * with the set of obstacles whose ttc bounds are violated. These obstacles
	 * are considered "important," and the scenario "risky."
	 *
	 * If an optimal trajectory is still not found, the scenario is considered
	 * "unsafe."
	 */
	namespace Evaluator {
		
		/**
		 * Perform a naive analysis of the safety of a given scenario. This is
		 * done by assuming a fixed control over some time horizon. This method
		 * fills the set of collision PT obstacles, which is empty if no
		 * collision is detected.
		 */
		void naiveSafetyTest( std::set<PVT_Obstacle*>& inCollision,
							 double cur_vel,
							 double u,
							 double time_horizon,
							 PVT_ObstacleSet& O,
							 Constraints& c );
		
		/**
		 * Run the planner on a given set of obstacles and constraints, return
		 * the optimal trajectory (or an empty trajectory, if there is none).
		 */
		bool getOptimalTrajectory( std::vector<TrajectorySegment*>& T,
								  Interval& V_i,
								  Interval& V_f,
								  PVT_ObstacleSet& O,
								  Constraints& c );
		
		/**
		 * This procedure grows the obstacle by the TTC bounds and attempts to
		 * to find an optimal trajectory.
		 *
		 * If the procedure returns false, an unexpected error was encountered,
		 * and the procedure was aborted.
		 *
		 * If T is empty after the procedure returns, no trajectory was found.
		 *
		 * If inCollision is empty after the procedure returns, then T contains
		 * a trajectory that respects TTC bounds.
		 *
		 * If inCollision is not empty, a risky trajectory was found, and
		 * inCollision contains the set of obstacles whose TTC bounds are
		 * violated by the trajectory.
		 */
		bool getOptimalTTC_Tracjectory( double x_fore_padding,
									   double x_aft_padding,
									   double t_fore_padding,
									   double t_aft_padding,
									   std::vector<TrajectorySegment*>& T,
									   std::set<PVT_Obstacle*>& inCollision,
									   Interval& V_i,
									   Interval& V_f,
									   PVT_ObstacleSet& O,
									   Constraints& c );
		
		/**
		 * Detect an impending imminent collision with a set of obstacles, all
		 * of which will be involved in the collision. This is a special case
		 * of a collision imminent state in which the set of obstacles in the
		 * collision path is known exactly. This can happen if the driver is
		 * travelling at non-zero velocity towards one or more obstacles, and
		 * cannot brake in time to avoid hitting 'all' of them, that is, all
		 * trajectory channels are obstructed by exactly the same set of
		 * obstacles. Such a scenario is called a "bottleneck" imminent
		 * collision state.
		 *
		 * inCollision is set to contain the set of bottleneck ICS obstacles.
		 */
		void detectBottleneckICS( std::set<PVT_Obstacle*>& inCollision,
								 Interval& V_i,
								 Interval& V_f,
								 PVT_ObstacleSet& O,
								 Constraints& c );
		
		/**
		 * Given an upper bounding trajectory and a lower bounding trajectory
		 * between two path-time points, this method computes the set of 
		 * obstacles that both intersect.
		 *
		 * inCollision is set to contain the set of obstacles that both UB and
		 * LB intersect.
		 */
		void getIntersectionOfCollisionObstacles( std::set<PVT_Obstacle*>& inCollision,
												 std::vector<TrajectorySegment*>& UB,
												 std::vector<TrajectorySegment*>& LB,
												 PVT_ObstacleSet& O,
												 Constraints& c );
		
		/**
		 * This method computes the intersection of the obstacle sets contained
		 * in inCollisionA and inCollisionB.
		 *
		 * inCollision is set to contain the intersection of inCollisionA and
		 * inCollisionB.
		 */
		void intersectCollisionObstacleSets( std::set<PVT_Obstacle*>& inCollision,
											std::set<PVT_Obstacle*>& inCollisionA,
											std::set<PVT_Obstacle*>& inCollisionB,
											Constraints& c );
		
	} // end Evaluator namespace
	
} // end SCIMP_Scenario namespace

#endif

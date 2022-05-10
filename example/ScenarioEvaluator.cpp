#include <PVTP/ScenarioEvaluator.hpp>
#include <PVTP/Planner.hpp>
#include <PVTP/Utilities.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	namespace Evaluator {
		
		void naiveSafetyTest( std::set<PVT_Obstacle*>& inCollision,
							 double cur_vel,
							 double u,
							 double time_horizon,
							 PVT_ObstacleSet& O,
							 Constraints& c ) {
			
			// compute path and velocity changes
			double x = Maths::motionX_FromV1_T1_T2_A( cur_vel, 0., time_horizon, u, c.getEpsilon() );
			double v = Maths::V2_FromV1_T_A( cur_vel, time_horizon, u, c.getEpsilon() );
			
			// construct trajectory
			PVT_State initial( 0., 0., cur_vel );
			PVT_State final( x, time_horizon, v );
			TrajectorySegment traj_seg( initial, final );
			
			// do collision checking
			Collisions::checkTrajectorySegment( inCollision, traj_seg, O, c, true );
		}
		
		bool getOptimalTrajectory( std::vector<TrajectorySegment*>& T,
								  Interval& V_i,
								  Interval& V_f,
								  PVT_ObstacleSet& O,
								  Constraints& c ) {
			
			// store computed velocity intervals
			std::vector<PVT_G*> G;
			std::vector<PVT_S*> Goal;
			
			// run forward propagation
			if ( !Planner::Forward(G, Goal, V_i, V_f, O, c) ) {
				std::cerr << "SCIMP_Scenario::Evaluator::getOptimalTrajectory: Error running Planner::Forward. Aborting." << std::endl;
				return false;
			}
			
			// run backward propagation
			std::vector<PVT_G*> G_intersect;
			if ( !Planner::Backward(G_intersect, G, Goal, V_i, O, c) ) {
				std::cerr << "SCIMP_Scenario::Evaluator::getOptimalTrajectory: Error running Planner::Backward. Aborting." << std::endl;
				return false;
			}
			
			// get optimal trajectory
			if ( !Planner::BuildOptimalTrajectory(T, G_intersect, Goal, O, c) ) {
				std::cerr << "SCIMP_Scenario::Evaluator::getOptimalTrajectory: Error running Planner::BuildOptimaltrajectory. Aborting." << std::endl;
				return false;
			}

			// clean up
			Utilities::CleanG( G_intersect );
			Utilities::CleanResults( G, Goal );
			
			return true;
		}
		
		bool getOptimalTTC_Tracjectory( double x_fore_padding,
									   double x_aft_padding,
									   double t_fore_padding,
									   double t_aft_padding,
									   std::vector<TrajectorySegment*>& T,
									   std::set<PVT_Obstacle*> inCollision,
									   Interval& V_i,
									   Interval& V_f,
									   PVT_ObstacleSet& O,
									   Constraints& c ) {
			
			// clear out storage
			Utilities::CleanTrajectory( T );
			inCollision.clear();
			
			// first try to find true optimal trajectory
			std::vector<TrajectorySegment*> T_raw;
			if ( !getOptimalTrajectory(T_raw, V_i, V_f, O, c) ) {
				std::cerr << "SCIMP_Scenario::Evaluator::getOptimalTTC_Trajectory: Error running getOptimalTrajectory (1). Aborting." << std::endl;
				return false;
			}
			
			// if T_raw is empty, this scenario is unsafe
			if ( T_raw.empty() ) {
				return true;
			}
			
			// now try for TTC trajectory... first grow obstacles
			O.growObstacles( x_fore_padding, x_aft_padding, t_fore_padding, t_aft_padding );
			
			// run planner again
			if ( !getOptimalTrajectory(T, V_i, V_f, O, c) ) {
				std::cerr << "SCIMP_Scenario::Evaluator::getOptimalTTC_Trajectory: Error running getOptimalTrajectory (2). Aborting." << std::endl;
				O.growObstacles( -x_fore_padding, -x_aft_padding, -t_fore_padding, -t_aft_padding );
				return false;
			}
			
			// if T is not empty, scenario is safe, and T contains TTC-bounded trajectory
			if ( !T.empty() ) {
				O.growObstacles( -x_fore_padding, -x_aft_padding, -t_fore_padding, -t_aft_padding );
				return true;
			}
			
			// at this point, there is a risky trajectory, but no safe trajectory
			// compute set of TTC-bounded obstacles the risky trajectory intersects
			if ( !Collisions::checkTrajectory(inCollision, T_raw, O, c) ) {
				std::cerr << "SCIMP_Scenario::Evaluator::getOptimalTTC_Trajectory: Error running checkTrajectory. Aborting." << std::endl;
				O.growObstacles( -x_fore_padding, -x_aft_padding, -t_fore_padding, -t_aft_padding );
				return false;
			}
			
			// now set T to be T_raw
			T = T_raw;
			
			// undo obstacle growth
			O.growObstacles( -x_fore_padding, -x_aft_padding, -t_fore_padding, -t_aft_padding );
			
			// exit
			return true;
		}
		
		void detectBottleneckICS( std::set<PVT_Obstacle*>& inCollision,
								 Interval& V_i,
								 Interval& V_f,
								 PVT_ObstacleSet& O,
								 Constraints& c ) {
			
			// clear anything in there new
			inCollision.clear();
			
			// trivial case
            if ( O.obstacles.empty() ) {
				return;
			}
			
			// get points
            std::vector<PVT_ObstaclePoint*> P_t;
			O.getAllVertices( P_t, c );
			
			// create origin
			PVT_ObstaclePoint origin( 0., 0., Constants::H_CLASS_ORIGIN, c );
			
			/**
			 * Attempt to connect to all nodes from origin, maintaining an
			 * intersection of the sets of obstacles that were found
			 * to be in collision. Return that set.
			 */

			// check goal connection
			std::vector<TrajectorySegment*> UB;
			std::vector<TrajectorySegment*> LB;
			if ( !Planner::GoalConnect(UB, LB, origin, V_i, V_f, O, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: GoalConnect failed" << std::endl;
				return;
			}
			
			// if the goal cannot be connected to, we're done
			if ( UB.empty() || LB.empty() ) {
				return;
			}
			
			// get goal collision trajectories
			getIntersectionOfCollisionObstacles( inCollision, UB, LB, O, c );
			
			// trivial case
			if ( inCollision.empty() ) {
				return;
			}
			
			// check interstitial points
			for ( size_t i=0; i<P_t.size(); i++ ) {
				
				// check reachability
				std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > BoundingTrajectories;
				BoundingTrajectories.first = NULL;
				BoundingTrajectories.second = NULL;
                if ( !Planner::NextReachableSet(BoundingTrajectories, origin, *P_t[i], V_i, c) ) {
					std::cerr << "ERROR IN Planner::Propagate: NextReachableSet failed" << std::endl;
					return;
				}
				
				// this point cannot be connected to, skip it
				if ( (BoundingTrajectories.first == NULL) || (BoundingTrajectories.second == NULL) ) {
					continue;
				}
				
				// get intersection of collision sets for representative trajectories
				std::set<PVT_Obstacle*> inCollisionInterstitial;
				getIntersectionOfCollisionObstacles( inCollisionInterstitial, *BoundingTrajectories.first, *BoundingTrajectories.second, O, c );
				
				// compute total intersection
				std::set<PVT_Obstacle*> inCollision_tmp( inCollision );
				intersectCollisionObstacleSets( inCollision, inCollision_tmp, inCollisionInterstitial, c );
				
				// trivial case
				if ( inCollision.empty() ) {
					return;
				}
			}
		}
		
		void intersectCollisionObstacleSets( std::set<PVT_Obstacle*>& inCollision,
											std::set<PVT_Obstacle*>& inCollisionA,
											std::set<PVT_Obstacle*>& inCollisionB,
											Constraints& c ) {
			// clear anything in there now
			inCollision.clear();
			
			// compute intersection of collision sets
			std::set<PVT_Obstacle*>::iterator it_i;
			for ( it_i=inCollisionA.begin(); it_i!=inCollisionA.end(); it_i++ ) {
				
				std::set<PVT_Obstacle*>::iterator it_j;
				for ( it_j=inCollisionB.begin(); it_j!=inCollisionB.end(); it_j++ ) {
				
					if ( (*it_i)->equals(**it_j, c) ) {
						
						inCollision.insert( *it_j );
						
					}
					
				}
			}
		}
		
		void getIntersectionOfCollisionObstacles( std::set<PVT_Obstacle*>& inCollision,
												 std::vector<TrajectorySegment*>& UB,
												 std::vector<TrajectorySegment*>& LB,
												 PVT_ObstacleSet& O,
												 Constraints& c ) {
			
			// clear anything in there now
			inCollision.clear();
			
			// check both representative trajectories for collision
			std::set<PVT_Obstacle*> inCollisionUB;
			if ( !Collisions::checkTrajectory(inCollisionUB, UB, O, c, true) ) {
				std::cerr << "ERROR IN Planner::Channel: collision check failed (1)" << std::endl;
				return;
			}
			std::set<PVT_Obstacle*> inCollisionLB;
			if ( !Collisions::checkTrajectory(inCollisionLB, LB, O, c, true) ) {
				std::cerr << "ERROR IN Planner::Channel: collision check failed (2)" << std::endl;
				return;
			}
			
			// trivial case
			if ( inCollisionUB.empty() || inCollisionLB.empty() ) {
				return;
			}
			
			intersectCollisionObstacleSets( inCollision, inCollisionUB, inCollisionLB, c );
		}
		
	} // end Evaluator namespace
	
} // end SCIMP_Scenario namespace

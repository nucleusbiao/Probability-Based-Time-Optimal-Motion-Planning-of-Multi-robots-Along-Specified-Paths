#ifndef PVTP_COLLISIONS_H
#define PVTP_COLLISIONS_H

#include <set>
#include <PVTP/Constants.hpp>
#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/TrajectorySegment.hpp>

//#define SHOW_COMMENTS

#ifdef SHOW_COMMENTS
#include <iostream>
#endif

namespace PVTP {
	
	/**
	 * This namespace contains routines for library-specific collision detection.
	 */
	namespace Collisions {
		
		/**
		 * Check a given trajectory for collision with any obstacle in O, return
		 * a list of obstacles in collision via inCollision.
		 */
		bool checkTrajectory( std::set<PVT_Obstacle*>& inCollision,
							 std::vector<TrajectorySegment*>& T,
							 PVT_ObstacleSet& O,
							 Constraints& c,
							 bool return_obstacles = false );
		
		/**
		 * Check a given trajectory segment for collision with any obstacle in O,
		 * return a list of obstacles in collision via inCollision.
		 *
		 * Currently this procedure assumes rectangular obstacles; future
		 * versions will handle arbitrary polygonal obstacles.
		 */
		bool checkTrajectorySegment( std::set<PVT_Obstacle*>& inCollision,
									TrajectorySegment& T,
									PVT_ObstacleSet& O,
									Constraints& c,
									bool return_obstacles = false );
				
	} // end Collisions namespace

} // end PVTP namespace

#endif
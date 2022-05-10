#ifndef PVTP_OBSTACLE_H
#define PVTP_OBSTACLE_H

#include <iostream>
#include <vector>
#include <PVTP/PVT_ObstaclePt.hpp>

namespace PVTP {
	
	/**
	 * Path-Velocity-Time Obstacle class. This class is for contructing and handling PT
	 * obstacles, which are assumed to be convex and polygonal in PT space.
	 */
	class PVT_Obstacle {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_Obstacle& o);
		
	public:
		
		/**
		 * PVT Obstacle constructor: rectangular obstacle, velocity constraint
		 * intervals on upper-right and lower-left vertices are empty, velocity
		 * constraints on upper-left and lower-right vertices are [-Inf, +Inf],
		 * and constraints on upper-right and lower-left are (empty). The
		 * upper-right corner is unreachable without infinite velocity, and the
		 * lower-left corner is reachable only with zero velocity, in which case
		 * the velocity necessarily connects to the upper-left corner, making a
		 * connection to the lower-left redundant.
		 *
		 * double[] box = { min_x, max_x, min_t, max_t }
		 */
		PVT_Obstacle ( double box[], Constraints& c );
		
		/**
		 * PVT Obstacle constructor: vector-based
		 */
		PVT_Obstacle ( std::vector<double>& box, Constraints& c );
		
		/**
		 * PVT Obstacle copy constructor
		 */
        PVT_Obstacle ( const PVT_Obstacle& obs, const Constraints& c );
		
		/**
		 * Destructor
		 */
		~PVT_Obstacle ();
		
		/**
		 * Translate the obstacle by some offset in path, and some offset in time
		 */
		void translateObstacle( double x_offset, double t_offset );
		
		/**
		 * Put a margin around the obstacle; padding applied equally to both sides.
		 *
		 * NOTE: This assumes rectangular obstacles.
		 */
		void growObstacle( double x_padding, double t_padding );
		
		/**
		 * Put a margin around the obstacle. "fore" refers to the side nearest
		 * the origin, "aft" the side farthest from the origin.
		 *
		 * NOTE: This assumes rectangular obstacles.
		 */
		void growObstacle( double x_fore_padding, double x_aft_padding, double t_fore_padding, double t_aft_padding );
		
		/**
		 * Get minimum path extent of obstacle
		 *
		 * NOTE: This assumes rectangular obstacles.
		 */
		double getMinPathCoord() const;
		
		/**
		 * Get maximum path extent of obstacle
		 *
		 * NOTE: This assumes rectangular obstacles.
		 */
		double getMaxPathCoord() const;
		
		/**
		 * Get minimum time extent of obstacle
		 *
		 * NOTE: This assumes rectangular obstacles.
		 */
		double getMinTimeCoord() const;
		
		/**
		 * Get maximum time extent of obstacle
		 *
		 * NOTE: This assumes rectangular obstacles.
		 */
		double getMaxTimeCoord() const;
		
		/**
		 * Test for equality
		 */
		bool equals( const PVT_Obstacle& other_obstacle, Constraints& c ) const;

		/**
		 * The PT coordinates of the obstacle: These are the vertices of the
		 * obstacle polygon. The ordering of the vertices is clockwise starting
		 * at the vertex with minimum path-coordinate and maximum time-coordinate.
		 */
        std::vector<PVT_ObstaclePoint> vertices;
		
		/**
		 * Initializer
		 */
		void init( double min_x, double max_x, double min_t, double max_t, Constraints& c );
		
	};
	
} // end PVTP namespace

#endif

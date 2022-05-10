#ifndef PVTP_OBSTACLE_POINT_H
#define PVTP_OBSTACLE_POINT_H

#include <PVTP/PVT_Point.hpp>

namespace PVTP {
	
	/**
	 * Path-Time Obstacle Point class. This class contains PVT_Points that are
	 * obstacle vertices. These are PVT_Points that contain an extra bit flag
	 * denoting whether a trajectory must pass above them (0) or below them (1)
	 * in order to remain collision-free and respect monotonicity constraints.
	 */
	class PVT_ObstaclePoint : public PVT_Point {

	public:
		
		/**
		 * PVT Obstacle Point constructor: unspecified interval.
		 */
		PVT_ObstaclePoint ( double p, double t, char type, Constraints& c );
		
		/**
		 * PVT Obstacle Point constructor: specified interval.
		 */
		PVT_ObstaclePoint ( double p, double t, Interval& i, char type, Constraints& c );
		
		/**
		 * PVT Obstacle Point constructor: specified interval.
		 */
		PVT_ObstaclePoint ( double p, double t, double min, double max, char type, Constraints& c );
		
		/**
		 * PVT Obstacle Point constructor: copy
		 */
		PVT_ObstaclePoint ( PVT_Point& p, char type, Constraints& c );

		/**
		 * PVT Obstacle Point constructor: copy
		 */
        PVT_ObstaclePoint ( const PVT_ObstaclePoint& p, const Constraints& c );
		
		/**
		 * Accessor for vertex type
		 */
		char getType() const;
		
		/**
		 * Mutator for vertex type
		 */
		void setType( char type );
		
		/**
		 * If the admissable velocities at this point prevent it from being
		 * feasibly reached at all, return false.
		 */
		bool isReachable( Constraints& c ) const;
		
	private:
		
		/**
		 * Called on initialization to adjust reachable interval based on
		 * whether point coordinates are valid, and set type
		 */
        void init( char type, const Constraints& c );
		
		/**
		 * Whether a trajectory must pass this point from above (0) or below (1)
		 */
		char type;
		
	};

	/**
	 * A function object used as a comparator to sort points by time coordinate.
	 *
	 * Sort order: Ascending.
	 */
	struct PVT_ObstaclePointComparator {
		double epsilon;
        bool operator() (PVT_ObstaclePoint *A, PVT_ObstaclePoint *B) {
            if ( Maths::approxGt(B->getTimeCoord(), A->getTimeCoord(), epsilon) ) {
				return true;
            } else if ( Maths::approxEq(B->getTimeCoord(), A->getTimeCoord(), epsilon) ) {
                if ( B->getPathCoord() > A->getPathCoord() ) {
					return true;
				}
			}
			return false;
		}
	};
	
} // end PVTP namespace

#endif

#ifndef PVTP_G_H
#define PVTP_G_H

#include <PVTP/PVT_ObstaclePt.hpp>

namespace PVTP {
	
	/**
	 * Container for a set of reachable velocity intervals at a given point.
	 * Objects of this class are generated and returned by the Forward algorithm.
	 */
	class PVT_G {
	public:

		/**
		 * Constructor: specified interval and point
		 */
		PVT_G ( PVT_ObstaclePoint& p );
		
		/**
		 * Constructor: specified interval and point
		 */
		PVT_G ( Interval& i, PVT_ObstaclePoint& p );
		
		/**
		 * Destructor
		 */
		~PVT_G ();
		
		/**
		 * Reachable velocity intervals
		 */
		std::vector<Interval*> * V;
		
		/**
		 * Point associated with these intervals
		 */
		PVT_ObstaclePoint * p;

	};
	
	/**
	 * A function object used as a comparator to sort reachable sets by time coordinate.
	 *
	 * Sort order: Ascending.
	 */
	struct PVT_G_Comparator {
		double epsilon;
		bool operator() (PVT_G * A, PVT_G * B) {
			if ( Maths::approxGt(B->p->getTimeCoord(), A->p->getTimeCoord(), epsilon) ) {
				return true;
			} else if ( Maths::approxEq(B->p->getTimeCoord(), A->p->getTimeCoord(), epsilon) ) {
				if ( B->p->getPathCoord() > A->p->getPathCoord() ) {
					return true;
				}
			}
			return false;
		}
	};

} // end PVTP namespace

#endif
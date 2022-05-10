#ifndef PVTP_S_H
#define PVTP_S_H

#include <iostream>
#include <PVTP/Interval.hpp>
#include <PVTP/TrajectorySegment.hpp>

namespace PVTP {
	
	/**
	 * Interval set class. Objects of this class are used by the Forward
	 * algorithm to track and merge goal-reachable velocity intervals at PT
	 * points. The reachable interval is stored as a pair of representative
	 * trajetories representing the maximum and minimum arrival velocities
	 * (and corresponding max/min arrival times).
	 *
	 * When an optimal trajectory is reconstructed, reconstruction begins
	 * by following one of these representative trajectories back to a point
	 * in the obstacle field.
	 */
	class PVT_S {
		
		/**
		 * Overload the output operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_S& s);
		
	public:

		/**
		 * Constructor: Used for intermediate states during propagation when
		 * only reachable interval and homotopic class information need to
		 * be stored
		 */
		PVT_S ( Interval& i );
		
		/**
		 * Constructor: Used for intermediate states during propagation when
		 * only reachable interval and homotopic class information need to
		 * be stored
		 */
		PVT_S ( Interval& i, std::vector<char>& B );

		/**
		 * Constructor: Used for goal connections; for convenience, include
		 * a reference to the obstacle point the interval is for
		 */
		PVT_S ( std::vector<TrajectorySegment*>& UB,
			   std::vector<TrajectorySegment*>& LB,
			   std::vector<char>& B,
			   PVT_ObstaclePoint& p );
		
		/**
		 * Constructor: Copy
		 */
		PVT_S ( PVT_S& S );
		
		/**
		 * Destructor
		 */
		~PVT_S ();
		
		/**
		 * Set the UB to a copy of another
		 */
		void setUB( std::vector<TrajectorySegment*>& UB );
		
		/**
		 * Set the LB to a copy of another
		 */
		void setLB( std::vector<TrajectorySegment*>& LB );
		
		/**
		 * A convenience method for getting the reachable velocity interval
		 * for the point that UB and LB originate at
		 */
		void getReachableInterval( Interval& V ) const;
		
		/**
		 * A reachable velocity interval at the goal
		 */
		Interval * V;
		
		/**
		 * The homotopic classification associated with the reachable interval
		 */
		std::vector<char> * B;
		
		/**
		 * A representative trajectory terminating in the maximum achievable velocity
		 */
		std::vector<TrajectorySegment*> * UB;
		
		/**
		 * A representative trajectory terminating in the maximum achievable velocity
		 */
		std::vector<TrajectorySegment*> * LB;
		
		/**
		 * The point this reachable set corresponds to
		 */
		PVT_ObstaclePoint * p;
	};
	
	/**
	 * A function object used as a comparator for sorting goal intervals.
	 *
	 * Sort order: Ascending.
	 */
	struct PVT_S_Comparator {
		bool operator() (PVT_S* A, PVT_S* B) {
			double a_time = A->UB->back()->getFinalState().getTimeCoord();
			double b_time = B->UB->back()->getFinalState().getTimeCoord();
			return a_time < b_time;
		}
	};

} // end PVTP namespace

#endif
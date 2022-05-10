#ifndef PVTP_STATE_H
#define PVTP_STATE_H

#include <iostream>
#include <PVTP/PVT_Point.hpp>

namespace PVTP {
	
	/**
	 * Path-Velocity-Time State class. Stores a point in PVT space, which is
	 * regarded as a system state by the planner. There are also some utility
	 * functions associated with states.
	 */
	class PVT_State {
		
		/**
		 * Overload the output operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_State& pvt_s);
		
	public:

		/**
		 * PVT State constructor: Duplicate.
		 */
		PVT_State ( PVT_State& s );
		
		/**
		 * PVT State constructor: Specified.
		 */
		PVT_State ( double p, double t, double v );
		
		/**
		 * PVT State constructor: Specified.
		 */
		PVT_State ( PVT_Point& p, double v );
		
		/**
		 * Initialize a PVT State object
		 */
		void init( double p, double t, double v );
		
		/**
		 * Accessor for path coordinate.
		 */
		double getPathCoord() const;
		
		/**
		 * Accessor for time coordinate.
		 */
		double getTimeCoord() const;
		
		/**
		 * Accessor for velocity coordinate.
		 */
		double getVelocityCoord() const;
		
		/**
		 * Fast test for reachability from this state to a given point.
		 */
		bool canReach( PVT_Point& p2, Constraints& c ) const;
		
		/**
		 * Test for state equality
		 */
		bool equals( PVT_State& s2, Constraints& c ) const;
		
		/**
		 * Test for point component: given PVT_Point
		 */
		bool atPoint( PVT_Point& p, Constraints& c ) const;
		
		/**
		 * Test for point component: given doubles
		 */
		bool atPoint( double x, double t, Constraints& c ) const;
		
	private:
		
		/**
		 * The path coordinate in PVT space.
		 */
		double p;
		
		/**
		 * The time coordinate in PVT space.
		 */
		double t;
		
		/**
		 * The velocity coordinate in PVT space.
		 */
		double v;
		
		
	};

} // end PVTP namespace

#endif

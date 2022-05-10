#ifndef PVTP_TRAJECTORY_SEGMENT_H
#define PVTP_TRAJECTORY_SEGMENT_H

#include <iostream>
#include <PVTP/PVT_State.hpp>

namespace PVTP {
	
	/**
	 * Trajectory Segment class. Store an initial PVT state, and a final PVT
	 * state, and provide utility methods for extracting information about the
	 * segment. A "trajectory segment" is defind as a segment of a trajectory
	 * across which constant acceleration is experienced.
	 */
	class TrajectorySegment {
		
		/**
		 * Overload the output operator.
		 */
		friend std::ostream& operator<<(std::ostream& out, const TrajectorySegment& ts);
		
	public:

		/**
		 * TrajectorySegment constructor: Constructed states.
		 */
		TrajectorySegment ( PVT_State* initial, PVT_State* final );
		
		/**
		 * TrajectorySegment constructor: Specified states.
		 */
		TrajectorySegment ( PVT_State& initial, PVT_State& final );
		
		/**
		 * TrajectorySegment constructor: Copy
		 */
		TrajectorySegment ( TrajectorySegment& Ts );

		/**
		 * TrajectorySegment constructor: Specified coordinates.
		 */
		TrajectorySegment ( double p_i, double t_i, double v_i, double p_f, double t_f, double v_f );
		
		/**
		 * Destructor
		 */
		~TrajectorySegment ();
		
		/**
		 * Compute acceleration across this segment
		 */
		double getAcceleration() const;
		
		/**
		 * Get the time duration of this segment
		 */
		double getDuration() const;
		
		/**
		 * Compute whether this is a null transition, that is, whether the 
		 * initial and final states are essentially the same.
		 *
		 * A trajectory segment is a null transition iff intial and final times
		 * are approximately equal.
		 */
		bool isNullTransition( Constraints& c ) const;
		
		/**
		 * Accessor for initial state.
		 */
		PVT_State& getInitialState() const;
		
		/**
		 * Accessor for final state.
		 */
		PVT_State& getFinalState() const;
		
	private:
		
		/**
		 * Initial state of trajectory segment
		 */
		PVT_State * initial;
		
		/**
		 * Final state of trajectory segment
		 */
		PVT_State * final;
		
	};

} // end PVTP namespace

#endif

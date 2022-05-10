#ifndef PVTP_CONTROLLER_H
#define PVTP_CONTROLLER_H

#include <limits>
#include <iostream>
#include <vector>
#include <sys/types.h>

namespace SCIMP_Scenario {
	
	/**
	 * This class acts as a controller for a vehicle object. It provides a
	 * mapping between time and control ("control" being longitudinal control,
	 * acceleration or deceleration), beginning at some fixed point in time.
	 * The unit of time is seconds, and time is tracked on a per-scenario basis;
	 * that is, time 0 occurs for all agents when the scenario begins, and is
	 * measured uniformly across all agents.
	 *
	 * Control is assumed to be constant between any two specified times, so if
	 * control is specified as accelerate 4m/s^2 at time t1, and accelerate
	 * 8m/s^2 at t2, the control for any time between t1 and t2 will be 4m/s^2.
	 * After t2, control becomes 8m/s^2.
	 *
	 * The controls are stored as a sequence of control points, which are
	 * time/control pairs.
	 *
	 * Times must be >= 0.
	 */
	class Controller {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const Controller& controller );
		
	public:
		
		/**
		 * Constructor
		 */
		Controller ();
		
		/**
		 * Copy constructor
		 */
		Controller ( Controller& controller );
		
		/**
		 * Destructor
		 */
		~Controller ();
        void initSet(const Controller& controller );
		/**
		 * Add a time/control pair to the control sequence. If the given time
		 * already exists in the control sequence, it is replaced.
		 */
		void addControl( double time, double control );
		
		/**
		 * Retrieve a control given a time. If a control for the given time is
		 * not explicitly stored in the sequence, return the control from the
		 * next nearest time that does not exceed the given time.
		 */
		double getControl( double time );
		
	private:
		
		/**
		 * Return the control index such that it points to the spot
		 * in the control sequence where a control for the given time
		 * should go.
		 *
		 * NOTE: The returned index points to a new member of the sequence,
		 * that is, a position where a new control point should be created
		 * and inserted. This means it returns -1 if the time begins at the
		 * beginning of the sequence, and control_sequence->size() if it
		 * belongs at the end.
		 *
		 * If the returned index points to an existing member, assume that
		 * insertion occurs between that member and the previous member in the 
		 * sequence; this mirrors std::vector::insert
		 *
		 * If the seek fails, false is returned. Otherwise true.
		 *
		 * This method does not error check the given time.
		 */
		ssize_t seek( double time );
		
		/**
		 * Retrieves the time member of the control point indexed by
		 * control_index
		 */
		double getControlTime() const;
		
		/**
		 * Retrieves the time member of the control point indexed by index
		 */
		double getIndexTime( size_t index ) const;
		
		/**
		 * Retrieves the control member of the control point indexed by
		 * control_index
		 */
		double getControlControl() const;
		
		/**
		 * Retrieves the control member of the control point indexed by index
		 */
		double getIndexControl( size_t index ) const;
		
		/**
		 * The current index into the control sequence
		 */
		size_t control_index;
		
		/**
		 * This vector of pairs is the control sequence, with the first 
		 * member being control time, and the second being control.
		 *
		 * Might be worthwhile switching this to a tree data structure
		 * at some point.
		 */
		std::vector< std::pair<double, double>* > * control_sequence;
		
	};
	
} // end SCIMP_Scenario namespace

#endif

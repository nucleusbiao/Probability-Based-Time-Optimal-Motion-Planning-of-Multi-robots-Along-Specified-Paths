#ifndef PVTP_LINEAR_SEGMENT_H
#define PVTP_LINEAR_SEGMENT_H

#include <PVTP/Constraints.hpp>
#include <PVTP/XY_Point.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	/**
	 * This class describes a linear segment of the path traversed by a vehicle
	 * in a scenario in which the planner is used. It is defined by a start
	 * point and end point in a plane orientated as described in the XY_Point
	 * class.
	 */
	class LinearSegment {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const LinearSegment& lSeg );
		
	public:
		
		/**
		 * Construct a linear segment parameterized by two points, and a
		 * and a direction. Let the direction be from p1 to p2.
		 */
		LinearSegment ( XY_Point& p1, XY_Point& p2 );
		
		/**
		 * Copy constructor
		 */
		LinearSegment ( LinearSegment& lSeg );
		
		/**
		 * Destructor
		 */
		~LinearSegment ();
		
		/**
		 * Accessor for the initial point of this linear segment
		 */
		XY_Point& getInitialPoint() const;
		
		/**
		 * Accessor for the final point of this linear segment
		 */
		XY_Point& getFinalPoint() const;
		
		/**
		 * Accessor for the length of this segment
		 */
		double getLength() const;
		
	private:
		
		/**
		 * Used by the constructors to initialize fields
		 */
		void init( XY_Point& p1, XY_Point& p2 );
		
		/**
		 * The initial point for this linear segment
		 */
		XY_Point * p1;
		
		/**
		 * The final point for this linear segment
		 */
		XY_Point * p2;
		
		/**
		 * The length of this segment
		 */
		double length;

	};

} // end SCIMP_Scenario namespace

#endif
#ifndef PVTP_ELLIPTICAL_SEGMENT_H
#define PVTP_ELLIPTICAL_SEGMENT_H

#include <PVTP/Constraints.hpp>
#include <PVTP/XY_Point.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	/**
	 * This class describes an elliptical segment of the path traversed by a vehicle
	 * in a scenario in which the planner is used.
	 *
	 * IMPORTANT: Elliptical segments are not currently supported.
	 */
	class EllipticalSegment {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const EllipticalSegment& eSeg );
		
	public:
		
		/**
		 * Construct the elliptical segment: The segment is parameterized by the
		 * lengths of the semi-major, A, and semi-minor, B, axes, and the center,
		 * which is specified in world-frame coordinates, where the world-frame
		 * is taken to be centered at the initial position of the
		 * user-controlled vehicle.
		 *
		 * IMPORTANT: The segment is assumed to extend exactly from the tip of
		 * the semi-minor axis to the tip of the semi-major; that is, it is
		 * assumed to be exactly a quarter ellipse.
		 */
		EllipticalSegment ( double A, double B, XY_Point& center );
		
		/**
		 * Destructor
		 */
		~EllipticalSegment ();
		
		/**
		 * Accessor for ellipse center
		 */
		XY_Point& getCenter() const;
		
		/**
		 * Accessor for ellipse semi-major axis length
		 */
		double getA() const;
		
		/**
		 * Accessor for ellipse semi-minor axis length
		 */
		double getB() const;
		
		/**
		 * Accessor for ellipse segment length
		 */
		double getLength() const;
		
	private:
		
		/**
		 * The length of the semi-major axis
		 */
		double A;
		
		/**
		 * The length of the semi-minor axis
		 */
		double B;
		
		/**
		 * The origin in the world frame
		 */
		XY_Point * center;
		
		/**
		 * The length of this segment
		 */
		double length;
		
	};
	
} // end SCIMP_Scenario namespace

#endif

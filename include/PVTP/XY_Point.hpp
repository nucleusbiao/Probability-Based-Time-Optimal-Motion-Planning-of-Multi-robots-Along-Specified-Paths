#ifndef PVTP_XY_POINT_H
#define PVTP_XY_POINT_H

#include <PVTP/PVT_ObstaclePt.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	/**
	 * This class describes a point in the world frame, which is a plane
	 * centered at the initial position of the user-controlled vehicle, and
	 * oriented such that positive Y extends in the direction of forward motion,
	 * and positive X extends in the direction of the driver's forward motion.
	 *
	 * The orientation of "direction of driver's forward motion" is a bit vague,
	 * since the driver's path can curve before settling on a final route
	 * (such as when turning left at an intersection). For now, since we're
	 * currently only considering lane-crossing scenarios, let this direction
	 * be the direction across the lanes.
	 */
	class XY_Point {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const XY_Point& p );
		
	public:
		
		/**
		 * Construct a point in the world frame of the scenario.
		 */
		XY_Point ( double x, double y );
		
		/**
		 * Copy constructor
		 */
		XY_Point ( XY_Point& p );
		
		/**
		 * Accessor for x-coordinate of this point
		 */
		double getX() const;
		
		/**
		 * Accessor for y-coordinate of this point
		 */
		double getY() const;
		
		/**
		 * Mutator to set this point equal to a given point
		 */
		void setPoint( XY_Point& p );
		
	private:
		
		/**
		 * Used by the constructors to initialize variables
		 */
		void init( double x, double y );
		
		/**
		 * The x-coordinate of the point
		 */
		double x;
		
		/**
		 * The y-coordinate of the point
		 */
		double y;
		
	};

} // end SCIMP_Scenario namespace

#endif

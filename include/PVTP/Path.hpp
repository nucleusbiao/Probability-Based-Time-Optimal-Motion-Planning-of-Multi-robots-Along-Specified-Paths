#ifndef PVTP_PATH_H
#define PVTP_PATH_H

#include <PVTP/LinearSegment.hpp>
#include <PVTP/EllipticalSegment.hpp>
#include <PVTP/Path.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	/**
	 * This class describes a path through the world space. The path is 
	 * comprised of a set of segments, which can be either linear or 
	 * elliptical.
	 *
	 * IMPORTANT: Currently, only a single linear segment is supported.
	 */
	class Path {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const Path& path );
		
	public:
        Path();
		/**
		 * Construct a path. For now, assume one linear segment.
		 */
		Path ( XY_Point& p1, XY_Point& p2 );
		
		/**
		 * Copy constructor
		 */
		Path ( Path& path );
		
		/**
		 * Destructor
		 */
		~Path ();
		

         void initSet( const Path& path );
		/**
		 * Add a linear segment to this path
		 */
		void addLinearSegment( XY_Point& p1, XY_Point& p2 );
		
		/**
		 * Add an elliptical segment to this path. It is assumed that p1 and
		 * p2 are terminal points of the semi-major and semi-minor axes. The
		 * convex parameter specifies which direction the ellipse should curve,
		 * that is, whether it is convex or concave from the perspective of
		 * the driver.
		 */
		void addEllipticalSegment( XY_Point& p1, XY_Point& p2, bool convex );
		
		/**
		 * Return a reference to the initial point of this path.
		 */
		XY_Point& getInitialPoint() const;
		
		/**
		 * Return a reference to the final point of this path
		 */
		XY_Point& getFinalPoint() const;
		
		/**
		 * Return the total length of the path
		 */
		double getLength() const;
		
	private:
		
		/**
		 * This vector contains the path segments. Each element is a pair that
		 * can be either a linear segment, or an elliptical segment.
		 *
		 * There's certainly a more elegant way to do this.
		 */
		std::vector< std::pair<LinearSegment*, EllipticalSegment*>* > * segments;
		
	};
	
} // end SCIMP_Scenario namespace

#endif

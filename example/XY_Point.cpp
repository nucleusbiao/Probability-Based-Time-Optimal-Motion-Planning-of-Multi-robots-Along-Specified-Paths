#include <PVTP/XY_Point.hpp>

namespace SCIMP_Scenario {
	
	XY_Point::XY_Point ( double x, double y ) {
		this->init( x, y );
	}
	
	XY_Point::XY_Point ( XY_Point& p ) {
		this->init( p.x, p.y );
	}
	
	double XY_Point::getX() const {
		return this->x;
	}
	
	double XY_Point::getY() const {
		return this->y;
	}
	
	void XY_Point::init( double x, double y ) {
		this->x = x;
		this->y = y;
	}
	
	void XY_Point::setPoint( XY_Point& p ) {
		this->init( p.x, p.y );
	}
	
	std::ostream& operator<<( std::ostream& out, const XY_Point& p ) {
		return out << "XY_Point: [" << p.getX() << ", " << p.getY() << "]";
	}

} // end SCIMP_Scenario namespace
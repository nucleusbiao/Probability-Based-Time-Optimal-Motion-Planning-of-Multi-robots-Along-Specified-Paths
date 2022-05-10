#include <PVTP/Maths.hpp>
#include <PVTP/EllipticalSegment.hpp>

namespace SCIMP_Scenario {
	
	EllipticalSegment::EllipticalSegment ( double A, double B, XY_Point& center ) {
		this->A = A;
		this->B = B;
		this->center = new XY_Point( center );
		
		// calculate approx. length
		double perimeter = Maths::approxEllipsePerimeter( fabs(this->A), fabs(this->B), 0. );
		this->length = perimeter / 4.0;
	}
	
	EllipticalSegment::~EllipticalSegment() {
		delete( this->center );
	}
	
	XY_Point& EllipticalSegment::getCenter() const {
		return *this->center;
	}

	double EllipticalSegment::getA() const {
		return this->A;
	}

	double EllipticalSegment::getB() const {
		return this->B;
	}
	
	double EllipticalSegment::getLength() const {
		return this->length;
	}
	
	std::ostream& operator<<( std::ostream& out, const EllipticalSegment& eSeg ) {
		return out << "EllipticalSegment: [A: " << eSeg.getA() << ", B: " << eSeg.getB() << ", Center: " << eSeg.getCenter() << "]";
	}
	
} // end SCIMP_Scenario namespace
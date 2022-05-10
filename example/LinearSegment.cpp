#include <PVTP/Maths.hpp>
#include <PVTP/LinearSegment.hpp>

namespace SCIMP_Scenario {
	
	LinearSegment::LinearSegment ( XY_Point& p1, XY_Point& p2 ) {
		this->init( p1, p2 );
	}
	
	LinearSegment::LinearSegment ( LinearSegment& lSeg ) {
		this->init( lSeg.getInitialPoint(), lSeg.getFinalPoint() );
	}
	
	void LinearSegment::init( XY_Point& p1, XY_Point& p2 ) {
		this->p1 = new XY_Point( p1 );
		this->p2 = new XY_Point( p2 );
		
		// calculate length
		std::pair<double, double> point1( p1.getX(), p1.getY() );
		std::pair<double, double> point2( p2.getX(), p2.getY() );
		this->length = Maths::euclideanDistance( point1, point2, 0. );
	}
	
	LinearSegment::~LinearSegment () {
		delete( this->p1 );
		delete( this->p2 );
	}
	
	XY_Point& LinearSegment::getInitialPoint() const {
		return *this->p1;
	}
	
	XY_Point& LinearSegment::getFinalPoint() const {
		return *this->p2;
	}
	
	double LinearSegment::getLength() const {
		return this->length;
	}
	
	std::ostream& operator<<( std::ostream& out, const LinearSegment& lSeg ) {
		return out << "LinearSegment: " << lSeg.getInitialPoint() << " -> " << lSeg.getFinalPoint();
	}
	
} // end SCIMP_Scenario namespace
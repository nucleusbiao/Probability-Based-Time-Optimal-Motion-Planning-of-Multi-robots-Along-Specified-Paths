#include <PVTP/PVT_Point.hpp>

namespace PVTP {

	PVT_Point::PVT_Point ( double p, double t ) {
		this->init( p, t, -std::numeric_limits<double>::max(), std::numeric_limits<double>::max() );
	}
	
	PVT_Point::PVT_Point ( double p, double t, Interval& i ) {
		this->init( p, t, i );
	}
	
	PVT_Point::PVT_Point ( double p, double t, double min, double max ) {
		this->init( p, t, min, max );
	}
	
    PVT_Point::PVT_Point ( const PVT_Point& p ) {
		this->init( p.getPathCoord(), p.getTimeCoord(), *p.i );
	}
	
	PVT_Point::~PVT_Point () {
		delete( this->i );
	}
	
	void PVT_Point::setCoords( double p, double t) {
		// enforce a positive sign on zero
		if ( p == 0. ) {
			p = 0.;
		}
		if ( t == 0. ) {
			t = 0.;
		}
		this->p = p;
		this->t = t;
	}
	
	void PVT_Point::translate( double p, double t ) {
		this->setCoords( this->p + p, this->t + t );
	}
	
	void PVT_Point::init( double p, double t, Interval& i ) {
		this->init( p, t, i.getMin(), i.getMax() );
	}
	
	void PVT_Point::init( double p, double t, double min, double max ) {
		this->setCoords( p, t );
		this->i = new Interval( min, max );
	}
	
	bool PVT_Point::canSee( PVT_Point& p2, Constraints& c ) const {
		double epsilon = c.getEpsilon();
		return Maths::approxLt( this->getTimeCoord(), p2.getTimeCoord(), epsilon )
		&& Maths::approxLe( this->getPathCoord(), p2.getPathCoord(), epsilon );
	}
	
	bool PVT_Point::isViewable( Constraints& c ) const {
		double epsilon = c.getEpsilon();
		return Maths::approxLe( 0., this->getTimeCoord(), epsilon )
		&& Maths::approxLe( 0., this->getPathCoord(), epsilon );
	}
	
	double PVT_Point::getPathCoord() const {
		return this->p;
	}

	double PVT_Point::getTimeCoord() const {
		return this->t;
	}
	
    bool PVT_Point::equals( const PVT_Point& pvt_p, Constraints& c ) const {
		double epsilon = c.getEpsilon();
		return Maths::approxEq( this->getPathCoord(), pvt_p.getPathCoord(), epsilon )
		&& Maths::approxEq( this->getTimeCoord(), pvt_p.getTimeCoord(), epsilon );
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_Point& pvt_p) {
		if ( pvt_p.i == NULL ) {
			Interval *tmp = new Interval();
			return out << "(" << pvt_p.getPathCoord() << ", " << pvt_p.getTimeCoord() << ", " << *tmp << ")";
			delete( tmp );
			tmp = NULL;
		}
		return out << "(" << pvt_p.getPathCoord() << ", " << pvt_p.getTimeCoord() << ", " << *(pvt_p.i) << ")";
	}
	
} // end PVTP namespace

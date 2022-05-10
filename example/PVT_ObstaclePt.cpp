#include <PVTP/PVT_ObstaclePt.hpp>

namespace PVTP {
	
	PVT_ObstaclePoint::PVT_ObstaclePoint ( double p, double t, char type, Constraints& c ) : PVT_Point( p, t ) {
		this->init( type, c );
	}

	PVT_ObstaclePoint::PVT_ObstaclePoint ( double p, double t, Interval& i, char type, Constraints& c ) : PVT_Point ( p, t, i ) {
		this->setType( type );
	}
	
	PVT_ObstaclePoint::PVT_ObstaclePoint ( double p, double t, double min, double max, char type, Constraints& c ) : PVT_Point ( p, t, min, max ) {
		this->init( type, c );
	}

	PVT_ObstaclePoint::PVT_ObstaclePoint ( PVT_Point& p, char type, Constraints& c ) : PVT_Point( p ) {
		this->init( type, c );
	}
	
    PVT_ObstaclePoint::PVT_ObstaclePoint ( const PVT_ObstaclePoint& p, const Constraints& c ) : PVT_Point( p ) {
		this->init( p.getType(), c );
	}
	
	char PVT_ObstaclePoint::getType() const {
		return this->type;
	}
	
	void PVT_ObstaclePoint::setType( char type ) {
		this->type = type;
	}
	
    void PVT_ObstaclePoint::init( char type, const Constraints& c ) {
		this->setType( type );
		
		bool valid_x = c.validX( this->getPathCoord() );
		bool valid_t = c.validT( this->getTimeCoord() );
		if ( !valid_x || !valid_t ) {
			this->i->setEmpty( true );
		}
	}
	
	bool PVT_ObstaclePoint::isReachable( Constraints& c ) const {
		if ( !this->isViewable(c) ) {
			return false;
		}
		if ( this->i == NULL ) {
			return true;
		}
		Interval velocities( c.getVMin(), c.getVMax() );
		velocities.intersect( *this->i, c );
		if ( velocities.isEmpty() ) {
			return false;
		}
		return true;
	}
	
} // end PVTP namespace

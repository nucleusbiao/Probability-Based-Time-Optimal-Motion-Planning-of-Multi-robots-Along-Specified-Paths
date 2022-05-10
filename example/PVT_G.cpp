#include <PVTP/PVT_G.hpp>

namespace PVTP {

	PVT_G::PVT_G ( PVT_ObstaclePoint& p ) {
		this->V = new std::vector<Interval*>();
		this->p = new PVT_ObstaclePoint( p );
	}
	
	PVT_G::PVT_G ( Interval& i, PVT_ObstaclePoint& p ) {
		this->V = new std::vector<Interval*>();
		this->V->push_back( new Interval(i) );
		this->p = new PVT_ObstaclePoint( p );
	}
	
	PVT_G::~PVT_G () {
		for ( size_t i=0; i<this->V->size(); i++ ) {
			delete( this->V->at(i) );
		}
		delete( this->V );
		delete( this->p );
	}
} // end PVTP namespace
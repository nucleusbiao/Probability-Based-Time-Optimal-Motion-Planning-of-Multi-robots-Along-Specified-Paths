#include <PVTP/Utilities.hpp>
#include <PVTP/PVT_S.hpp>

namespace PVTP {
	
	PVT_S::PVT_S ( Interval& i ) {
		this->V = new Interval( i );
		this->B = new std::vector<char>();
		this->UB = NULL;
		this->LB = NULL;
		this->p = NULL;
	}

	PVT_S::PVT_S ( Interval& i, std::vector<char>& B ) {
		this->V = new Interval( i );
		this->B = new std::vector<char>( B );
		this->UB = NULL;
		this->LB = NULL;
		this->p = NULL;
	}
	
	PVT_S::PVT_S ( std::vector<TrajectorySegment*>& UB,
				  std::vector<TrajectorySegment*>& LB,
				  std::vector<char>& B,
				  PVT_ObstaclePoint& p ) {
		
		this->B = new std::vector<char>( B );
		
		this->UB = new std::vector<TrajectorySegment*>( UB );
		if ( &UB != &LB ) {
			this->LB = new std::vector<TrajectorySegment*>( LB );
		} else {
			this->LB = new std::vector<TrajectorySegment*>( LB.size() );
			for ( size_t i=0; i<LB.size(); i++ ) {
				this->LB->at(i) = new TrajectorySegment( *LB.at(i) );
			}
		}
		
		double v_min = LB.back()->getFinalState().getVelocityCoord();
		double v_max = UB.back()->getFinalState().getVelocityCoord();
		this->V = new Interval( v_min, v_max );
		
		this->p = new PVT_ObstaclePoint( p );
	}
	
	PVT_S::PVT_S ( PVT_S& S ) {
		this->B = new std::vector<char>( *S.B );
		this->V = new Interval( *S.V );
		if ( S.UB != NULL ) {
			this->UB = new std::vector<TrajectorySegment*>( S.UB->size(), NULL );
			for ( size_t i=0; i<S.UB->size(); i++ ) {
				this->UB->at(i) = new TrajectorySegment( *S.UB->at(i) );
			}
		} else {
			this->UB = NULL;
		}
		if ( S.LB != NULL ) {
			this->LB = new std::vector<TrajectorySegment*>( S.LB->size(), NULL );
			for ( size_t i=0; i<S.LB->size(); i++ ) {
				this->LB->at(i) = new TrajectorySegment( *S.LB->at(i) );
			}
		} else {
			this->LB = NULL;
		}
		this->p = new PVT_ObstaclePoint( *S.p );
	}
	
	PVT_S::~PVT_S () {
		if ( this->UB != NULL ) {
			Utilities::CleanTrajectory( *this->UB );
			delete( this->UB );
		}
		if ( this->LB != NULL ) {
			Utilities::CleanTrajectory( *this->LB );
			delete( this->LB );
		}
		if ( this->p != NULL ) {
			delete( this->p );
		}
		this->B->clear();
		delete( this->B );
		delete( this->V );
	}
	
	void PVT_S::getReachableInterval( Interval& V ) const {
		if ( (this->UB == NULL) && (this->LB == NULL) ) {
			V.setEmpty( true );
			return;
		}
		if ( this->UB == NULL ) {
			double v = this->LB->at(0)->getInitialState().getVelocityCoord();
			V.init( v, v );
			return;
		}
		if ( this->LB == NULL ) {
			double v = this->UB->at(0)->getInitialState().getVelocityCoord();
			V.init( v, v );
			return;
		}
		double v1 = this->LB->at(0)->getInitialState().getVelocityCoord();
		double v2 = this->UB->at(0)->getInitialState().getVelocityCoord();
		if ( v1 > v2 ) {
			V.init( v2, v1 );
			return;
		}
		V.init( v1, v2 );
	}
	
	void PVT_S::setUB( std::vector<TrajectorySegment*>& UB ) {
		if ( this->UB != NULL ) {
			Utilities::CleanTrajectory( *this->UB );
			this->UB->resize( UB.size() );
		} else {
			this->UB = new std::vector<TrajectorySegment*>( UB.size(), NULL );
		}
		for ( size_t i=0; i<UB.size(); i++ ) {
			this->UB->at(i) = new TrajectorySegment( *UB.at(i) );
		}
	}
	
	void PVT_S::setLB( std::vector<TrajectorySegment*>& LB ) {
		if ( this->LB != NULL ) {
			Utilities::CleanTrajectory( *this->LB );
			this->LB->resize( LB.size() );
		} else {
			this->LB = new std::vector<TrajectorySegment*>( LB.size(), NULL );
		}
		for ( size_t i=0; i<LB.size(); i++ ) {
			this->LB->at(i) = new TrajectorySegment( *LB.at(i) );
		}
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_S& s) {
		out << *s.V << " ";
		for ( size_t i=0; i<s.B->size(); i++ ) {
			out << ((s.B->at(i)==0)?"0":"1") << " ";
		}
		return out;
	}
	
} // end PVTP namespace
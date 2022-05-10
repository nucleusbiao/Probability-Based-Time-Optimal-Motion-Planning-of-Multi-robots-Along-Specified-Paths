#include <PVTP/Maths.hpp>
#include <PVTP/TrajectorySegment.hpp>

namespace PVTP {

	TrajectorySegment::TrajectorySegment ( PVT_State* initial, PVT_State* final ) {
		this->initial = new PVT_State( *initial );
		this->final = new PVT_State( *final );
	}
	
	TrajectorySegment::TrajectorySegment ( PVT_State& initial, PVT_State& final ) {
		this->initial = new PVT_State( initial );
		this->final = new PVT_State( final );		
	}
	
	TrajectorySegment::TrajectorySegment ( double p_i, double t_i, double v_i, double p_f, double t_f, double v_f ) {
		this->initial = new PVT_State( p_i, t_i, v_i );
		this->final = new PVT_State( p_f, t_f, v_f );
	}
	
	TrajectorySegment::TrajectorySegment ( TrajectorySegment& Ts ) {
		this->initial = new PVT_State( *Ts.initial );
		this->final = new PVT_State( *Ts.final );
	}
	
	TrajectorySegment::~TrajectorySegment () {
		delete( this->initial );
		delete( this->final );
	}
	
	double TrajectorySegment::getAcceleration() const {
		double v1 = this->getInitialState().getVelocityCoord();
		double v2 = this->getFinalState().getVelocityCoord();
		double t1 = this->getInitialState().getTimeCoord();
		double t2 = this->getFinalState().getTimeCoord();
		return Maths::A_FromV1_V2_T1_T2( v1, v2, t1, t2, 0. );
	}
	
	double TrajectorySegment::getDuration() const {
		double t1 = this->getInitialState().getTimeCoord();
		double t2 = this->getFinalState().getTimeCoord();
		return t2 - t1;
	}
	
	bool TrajectorySegment::isNullTransition( Constraints& c ) const {
		double epsilon = c.getEpsilon();
		return Maths::approxEq( this->getInitialState().getTimeCoord(), this->getFinalState().getTimeCoord(), epsilon );
	}
	
	PVT_State& TrajectorySegment::getInitialState() const {
		return *this->initial;
	}
	
	PVT_State& TrajectorySegment::getFinalState() const {
		return *this->final;
	}
	
	std::ostream& operator<<(std::ostream& out, const TrajectorySegment& traj) {
		return out << traj.getInitialState() << " -> " << traj.getFinalState();
	}
	
} // end PVTP namespace

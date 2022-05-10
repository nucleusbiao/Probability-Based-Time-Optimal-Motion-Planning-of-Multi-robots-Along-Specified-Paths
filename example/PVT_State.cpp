#include <PVTP/Constants.hpp>
#include <PVTP/PVT_State.hpp>
#include <PVTP/Maths.hpp>

namespace PVTP {

	PVT_State::PVT_State ( PVT_State& s ) {
		this->init( s.getPathCoord(), s.getTimeCoord(), s.getVelocityCoord() );
	}
	
	PVT_State::PVT_State ( double p, double t, double v ) {
		this->init( p, t, v );
	}
	
	PVT_State::PVT_State ( PVT_Point& p, double v ) {
		this->init( p.getPathCoord(), p.getTimeCoord(), v );
	}
	
	void PVT_State::init( double p, double t, double v ) {
		this->p = p;
		this->t = t;
		this->v = v;		
	}
	
	double PVT_State::getPathCoord() const {
		return this->p;
	}
	
	double PVT_State::getTimeCoord() const {
		return this->t;
	}
	
	double PVT_State::getVelocityCoord() const {
		return this->v;
	}
	
	bool PVT_State::canReach( PVT_Point& p2, Constraints& c ) const {
		
		double epsilon = c.getEpsilon();
		
		// monotonicity in x
		double x1 = this->getPathCoord();
		double x2 = p2.getPathCoord();
		double delta_x = x2 - x1;
		if ( !c.validX(delta_x) ) {
			return false;
		}
		
		// montonicity in t
		double t1 = this->getTimeCoord();
		double t2 = p2.getTimeCoord();
		double delta_t = t2 - t1;
		if ( !c.validT(delta_t) ) {
			return false;
		}
		
		// no change in position, trivially reachable
		bool delta_x_zero = Maths::approxEq(delta_x, 0., epsilon);
		bool delta_t_zero = Maths::approxEq(delta_t, 0., epsilon);
		if ( delta_x_zero && delta_t_zero ) {
			return true;
		}
		
		// instantaneous change in position, unreachable
		if ( delta_t_zero && !delta_x_zero ) {
			return false;
		}
		
		// average velocity
		double v_avg = delta_x / delta_t;
		if ( !c.validV(v_avg) ) {
			return false;
		}

		// deceleration
		double v1 = this->getVelocityCoord();
		if ( Maths::approxGt(v1, v_avg, epsilon) ) {
			double t_inf = Maths::T_FromV1_V2_A( v1, 0., c.getAMin(), epsilon );
			double x_inf = Maths::motionX_FromV1_T1_T2_A( v1, 0., t_inf, c.getAMin(), epsilon );
			if ( Maths::approxGt(x_inf, c.getXLimit(), epsilon) && Maths::approxLt(t_inf, c.getTLimit(), epsilon) ) {
				return false;
			}
		}
		
		// acceleration
		double a_req = Maths::motionA_FromV1_X1_X2_T1_T2( this->getVelocityCoord(), x1, x2, t1, t2, epsilon );
		if ( !c.validA( a_req ) ) {
#ifdef SHOW_COMMENTS
			std::cout << "a_req: " << a_req << " [" << c.getAMin() << ", " << c.getAMax() << "]" << std::endl;
#endif
			return false;
		}
		
		// p2 is reachable from s1
		return true;
	}
	
	bool PVT_State::atPoint( PVT_Point& p, Constraints& c ) const {
		return this->atPoint( p.getPathCoord(), p.getTimeCoord(), c );
	}
	
	bool PVT_State::atPoint( double x, double t, Constraints& c ) const {
		return Maths::approxEq( this->getPathCoord(), x, c.getEpsilon() )
		&& Maths::approxEq( this->getTimeCoord(), t, c.getEpsilon() );
	}
	
	bool PVT_State::equals( PVT_State& s2, Constraints& c ) const {
		double epsilon = c.getEpsilon();
		return Maths::approxEq( this->getPathCoord(), s2.getPathCoord(), epsilon )
		&& Maths::approxEq( this->getTimeCoord(), s2.getTimeCoord(), epsilon )
		&& Maths::approxEq( this->getVelocityCoord(), s2.getVelocityCoord(), epsilon );
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_State& pvt_s) {
		return out << "(" << pvt_s.getPathCoord() << ", " << pvt_s.getTimeCoord() << ", " << pvt_s.getVelocityCoord() << ")";
	}
} // end PVTP namespace
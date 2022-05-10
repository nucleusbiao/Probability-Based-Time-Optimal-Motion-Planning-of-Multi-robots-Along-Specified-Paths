#include <PVTP/Utilities.hpp>
#include <PVTP/Vehicle.hpp>

namespace SCIMP_Scenario {

	Vehicle::Vehicle ( double width, double length, double min_vel, double max_vel, double min_acc, double max_acc ) {
		this->width = width;
		this->length = length;
		this->min_velocity = min_vel;
		this->max_velocity = max_vel;
		this->min_acceleration = min_acc;
		this->max_acceleration = max_acc;
		this->position = 0.;
	}
    Vehicle::Vehicle ( ) {  }
	Vehicle::Vehicle ( Vehicle& v ) {
		this->width = v.getWidth();
		this->length = v.getLength();
		this->min_velocity = v.getMinimumVelocity();
		this->max_velocity = v.getMaximumVelocity();
		this->min_acceleration = v.getMinimumAcceleration();
		this->max_acceleration = v.getMaximumAcceleration();
		this->position = v.getPosition();
		
		this->velocity = v.velocity;
		this->acceleration = v.acceleration;
	}

    void Vehicle::initSet(const  Vehicle& v ) {
        this->width = v.getWidth();
        this->length = v.getLength();
        this->min_velocity = v.getMinimumVelocity();
        this->max_velocity = v.getMaximumVelocity();
        this->min_acceleration = v.getMinimumAcceleration();
        this->max_acceleration = v.getMaximumAcceleration();
        this->position = v.getPosition();

        this->velocity = v.velocity;
        this->acceleration = v.acceleration;
    }
	double Vehicle::applyControl( double control, double time_step ) {

		// initialize these to NaN to detect errors
		double new_velocity = std::numeric_limits<double>::quiet_NaN();
		double distance_travelled = std::numeric_limits<double>::quiet_NaN();
		
		// calculate the effect of the controls
		this->calculateControlEffects( new_velocity,
									  distance_travelled,
									  control,
									  time_step );
		
		// error detection
		if ( Maths::isNaN(new_velocity) || Maths::isNaN(distance_travelled) ) {
			std::cerr << "Error computing control effects. Control: " << control << ", time_step: " << time_step << std::endl;
			return distance_travelled;
		}
		
		// the set velocity method will truncate new_velocity if necessary,
		// no need to do it here
		this->setVelocity( new_velocity );
		
		// increment position
		this->incrementPosition( distance_travelled );
		
		return distance_travelled;
	}
	
	void Vehicle::calculateControlEffects( double& new_velocity,
										  double& distance_travelled,
										  double control,
										  double time_step ) {
		if ( time_step < 0. ) {
			return;
		}
		
		control = Utilities::truncateValue( control,
										   this->getMinimumAcceleration(),
										   this->getMaximumAcceleration() );
		
		double current_velocity = this->getVelocity();
		new_velocity = Maths::V2_FromV1_T_A( current_velocity, time_step, control, 0. );
		
		// if the new velocity exceeds bounds, take this into account when
		// computing distance traversed
		double max_v = this->getMaximumVelocity();
		double min_v = this->getMinimumVelocity();
		if ( new_velocity > max_v ) {
			
			// time to reach max velocity
			double time_to_max_v = Maths::T_FromV1_V2_A( current_velocity,
														max_v,
														control,
														0. );
			// time at max velocity
			double time_at_max_v = time_step - time_to_max_v;
			
			// compute distance piecewise
			double d1 = Maths::motionX_FromV1_T1_T2_A( current_velocity,
													  0.,
													  time_to_max_v,
													  control,
													  0. );
			double d2 = Maths::motionX_FromV1_T1_T2_A( max_v,
													  0.,
													  time_at_max_v,
													  0.,
													  0. );
			
			new_velocity = max_v;
			distance_travelled = d1 + d2;
		} else if ( new_velocity < min_v ) {
			
			// time to reach min velocity
			double time_to_min_v = Maths::T_FromV1_V2_A( current_velocity,
														min_v,
														control,
														0. );
			
			// compute distance just for this segment
			double d1 = Maths::motionX_FromV1_T1_T2_A( current_velocity,
													  0.,
													  time_to_min_v,
													  control,
													  0. );
			
			new_velocity = min_v;
			distance_travelled = d1;
			
		} else {
			distance_travelled = Maths::motionX_FromV1_T1_T2_A( current_velocity,
															   0.,
															   time_step,
															   control,
															   0. );
		}
	}
	
	double Vehicle::getWidth() const {
		return this->width;
	}
	
	double Vehicle::getLength() const {
		return this->length;
	}
	
	double Vehicle::getVelocity() const {
		return this->velocity;
	}
	
	void Vehicle::setVelocity( double velocity ) {
		this->velocity = Utilities::truncateValue( velocity, this->min_velocity, this->max_velocity );
	}
	
	double Vehicle::getAcceleration() const {
		return this->acceleration;
	}
	
	void Vehicle::setAcceleration( double acceleration ) {
		this->acceleration = Utilities::truncateValue( acceleration, this->min_acceleration, this->max_acceleration );
	}
	
	double Vehicle::getPosition() const {
		return this->position;
	}
	
	void Vehicle::setPosition( double p ) {
		this->position = p;
	}
	
	void Vehicle::incrementPosition( double inc ) {
		this->position = this->position + inc;
	}
	
	double Vehicle::getMinimumVelocity() const {
		return this->min_velocity;
	}
	
	double Vehicle::getMaximumVelocity() const {
		return this->max_velocity;
	}
	
	double Vehicle::getMinimumAcceleration() const {
		return this->min_acceleration;
	}
	
	double Vehicle::getMaximumAcceleration() const {
		return this->max_acceleration;
	}
	
	std::ostream& operator<<( std::ostream& out, const Vehicle& vehicle ) {
		out << "position: " << vehicle.getPosition() << ", width: " << vehicle.getWidth() << ", length: " << vehicle.getLength();
		out << ", velocity: " << vehicle.getVelocity() << ", acceleration: " << vehicle.getAcceleration();
		out << ", velocity range: [" << vehicle.getMinimumVelocity() << ", " << vehicle.getMaximumVelocity();
		out << "], acceleration range: [" << vehicle.getMinimumAcceleration() << ", " << vehicle.getMaximumAcceleration() << "]" << std::endl;
		return out;
	}
	
} // end SCIMP_Scenario namespace

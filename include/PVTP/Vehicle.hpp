#ifndef PVTP_VEHICLE_H
#define PVTP_VEHICLE_H

#include <PVTP/XY_Point.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	/**
	 * This class describes a vehicle in a scenario. Vehicles are assumed
	 * to be rectangular. All units are meters, seconds, or some combination
	 * thereof.
	 *
	 * The vehicle's position coordinate is located in the middle of its
	 * rear side, which approximates the midpoint of the rear axle.
	 */
	class Vehicle {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const Vehicle& vehicle );
		
	public:
        Vehicle ();
		/**
		 * Constructor: specify vehicle dimensions, and dynamic constraints
		 */
		Vehicle ( double width, double length, double min_vel, double max_vel, double min_acc, double max_acc );
		
		/**
		 * Copy constructor
		 */
        Vehicle ( Vehicle& v );
		

        void initSet( const Vehicle& v );
		/**
		 * Apply a control directly to the vehicle for the specified time.
		 * The distance traversed over time is returned. If invalid arguments
		 * are given (like negative time), NaN is returned.
		 */
		double applyControl( double control, double time_step );
		
		/**
		 * Calculate the effects of applying a control; because we have hard
		 * constraints on velocity, there is a piecewise, instead of linear,
		 * relationship, so this is a convenience method for dealing with it.
		 */
		void calculateControlEffects( double& new_velocity,
									 double& distance_travelled,
									 double control,
									 double time_step );
		
		/**
		 * Accessor for vehicle width
		 */
		double getWidth() const;
		
		/**
		 * Accessor for vehicle length
		 */
		double getLength() const;
		
		/**
		 * Accessor for vehicle's current velocity
		 */
		double getVelocity() const;
		
		/**
		 * Mutator for vehicle's current velocity
		 */
		void setVelocity( double velocity );
		
		/**
		 * Accessor for vehicle's current acceleration
		 */
		double getAcceleration() const;
		
		/**
		 * Mutator for vehicle's current acceleration
		 */
		void setAcceleration( double acceleration );
		
		/**
		 * Accessor for vehicle's current position along a path
		 */
		double getPosition() const;
		
		/**
		 * Mutator for vehicle's current position along a path
		 */
		void setPosition( double p );
		
		/**
		 * Mutator for vehicle's current position along a path
		 */
		void incrementPosition( double inc );
		
		/**
		 * Accessor for this vehicle's minimum velocity
		 */
		double getMinimumVelocity() const;
		
		/**
		 * Accessor for this vehicle's maximum velocity
		 */
		double getMaximumVelocity() const;
		
		/**
		 * Accessor for this vehicle's minimum acceleration
		 */
		double getMinimumAcceleration() const;
		
		/**
		 * Accessor for this vehicle's maximum acceleration
		 */
		double getMaximumAcceleration() const;
		
	private:
		
		/**
		 * The vehicle's width
		 */
		double width;
		
		/**
		 * The vehicle's length
		 */
		double length;
		
		/**
		 * The vehicle's current velocity
		 */
		double velocity;
		
		/**
		 * The vehicle's minimum achievable velocity
		 */
		double min_velocity;
		
		/**
		 * The vehicle's maximum achievable velocity
		 */
		double max_velocity;
		
		/**
		 * The vehicle's current acceleration
		 */
		double acceleration;
		
		/**
		 * The vehicle's minimum achievable acceleration
		 */
		double min_acceleration;
		
		/**
		 * The vehicle's maximum achievable acceleration
		 */
		double max_acceleration;
		
		/**
		 * The vehicle's current position along a path
		 */
		double position;
		
	};

} // end SCIMP_Scenario namespace

#endif

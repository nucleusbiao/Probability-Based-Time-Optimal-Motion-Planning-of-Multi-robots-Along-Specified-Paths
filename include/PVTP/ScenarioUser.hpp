#ifndef PVTP_SCENARIO_USER_H
#define PVTP_SCENARIO_USER_H

#include <PVTP/Constraints.hpp>
#include <PVTP/SCIMP_Controller.hpp>
#include <PVTP/Scenario.hpp>

using namespace PVTP;

/**
 * This namespace contains the scenario API that can be used to interact
 * with the planner.
 *
 * A scenario consists of a user-controlled vehicle, and a (possibly
 * empty) set of obstacles (other vehicles). These two things are described by
 * the ScenarioUser and ScenarioObstacle classes, respectively.
 *
 * The ScenarioUser class consists of a Vehicle object and a Path object, which
 * respectively describe the characteristics of the user's vehicle (such as
 * length, width, velocity, position, etc.), and the path that the vehicle
 * follows. Because this is the user-controlled vehicle, it also contains
 * a representation of the scenario (the other vehicles and their paths), and
 * and a SCIMP controller which attempts to prevent the user from executing
 * unsafe controls.
 *
 * The ScenarioObstacle class also contains a Vehicle and a Path. Instead of
 * a SCIMP controller, it contains an actual controller that defines how the
 * vehicle is controlled over time. (In this context, control is limited to
 * longitudinal control: acceleration or deceleration.)
 */
namespace SCIMP_Scenario {

	/**
	 * This class is a container for the scenario user. The scenario user
	 * is the vehicle under user control, and a path that that vehicle
	 * follows. This is sort of the "master object" that drives simulations.
	 * Each applied control is assumed to be an advancement of one time
	 * step in the simulation.
	 */
	class ScenarioUser {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const ScenarioUser& scenario_user );
		
	public:
		
		/**
		 * Constructor
		 */
		ScenarioUser ( Vehicle& vehicle,
					  Path& path,
					  double final_vel_min,
					  double fina_vel_max,
					  double time_step,
					  double alpha,
					  Constraints& c );
        ScenarioUser();
		/**
		 * Copy constructor
		 */
        ScenarioUser(const ScenarioUser& user );
		
		/**
		 * Destructor
		 */
        ~ScenarioUser();

        void init( Vehicle& vehicle, Path& path,double final_vel_min,double final_vel_max,double time_step,double alpha,Constraints& c ) ;
		/**
		 * Evaluate a scenario: this constructs the set of path-time 
		 * obstacles and prepares the scenario to be simulated. Call this
		 * after all scenario obstacles have been added, or any time more
		 * are added.
		 */
		void initializeScenario();
        void generatePositionSeq(double timeStep);
		/**
		 * Send a control for direct execution
		 */
		void applyUnfilteredControl( double control, double time_step );
		
		/**
		 * Send a control to the vehicle for execution for the specified time.
		 * This control will be filtered by the SCIMP controller. The control
		 * that is actually executed is returned.
		 */
        bool applyFilteredControl( double control, double time_step, bool ignore_scimp = false );
		
		/**
		 * Same as above, but also lets you look at raw planner control
		 */
        bool applyFilteredControl( double& planner_control, double control, double time_step, bool ignore_scimp = false );
		
		/**
		 * Computes a filtered control, but does not apply it; boolean returns
		 * whether one could be found.
		 */
        bool computeFilteredControl( double& filtered_control, double& planner_control, double user_control, double time_step,bool& successFlag, bool ignore_scimp = false );
		
		/**
		 * Return a reference to the final velocity interval
		 */
        const Interval& getFinalVelocityInterval() const;
		
		/**
		 * Return a reference to the user's vehicle
		 */
        const Vehicle& getVehicle() const;
		
		/**
		 * Return a reference to the user's path
		 */
        const Path& getPath() const;
		
		/**
		 * Return a reference to the user's scenario
		 */
        const Scenario& getScenario() const;
		
		/**
		 * Return a reference to the user's constraints object
		 */
        const Constraints& getConstraints() const;
		
		/**
		 * Return a reference to the SCIMP controller
		 */
        const SCIMP_Controller& getSCIMPController() const;

		/**
		 * Whether or not the path has been traversed.
		 */
		bool pathTraversed() const;
		
		/**
		 * Whether or not the time limit has been reached
		 */
		bool timeLimitReached() const;
		
		/**
		 * Setter for current time
		 */
		bool setCurrentTime( double time );
		
		/**
		 * Getter for current time
		 */
		double getCurrentTime() const;
		
		/**
		 * Getter for default time step
		 */
		double getTimeStep() const;
		
		/**
		 * Set the state of the user vehicle
		 */
		bool setVehicleState( double path_position, double velocity );
		

		/**
		 * Whether or not the scenario has been initialized
		 */
		bool initialized;
		
		/**
		 * The alpha safety level for this scenario user
		 */
		double alpha;
		
		/**
		 * The set of constraints for this scenario user
		 */
        Constraints constraints;
		
		/**
		 * The user's vehicle
		 */
        Vehicle vehicle;
		
		/**
		 * The user's path
		 */
        Path path;
		
		/**
		 * Accumulator that tracks the current total time elapsed
		 */
		double current_time;
		
		/**
		 * Default time step for scenario
		 */
		double time_step;
		
		/**
		 * The desired final velocity range for the end of the path
		 */
        Interval final_velocity_interval;
		
		/**
		 * The scenario, which is the user's representation of the world
		 */
        Scenario scenario;
		
		/**
		 * The SCIMP controller that sits between the user and the vehicle
		 */
        SCIMP_Controller scimp_controller;
        std::vector<double> positionSeq;
	};
	
} // end SCIMP_Scenario namespace

#endif

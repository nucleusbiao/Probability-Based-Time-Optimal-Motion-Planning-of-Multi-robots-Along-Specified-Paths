#ifndef PVTP_SCENARIO_OBSTACLE_H
#define PVTP_SCENARIO_OBSTACLE_H

#include <PVTP/Vehicle.hpp>
#include <PVTP/Path.hpp>
#include <PVTP/Controller.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	/**
	 * This class is a container for a scenario obstacle. A scenario obstacle
	 * is a vehicle, a path that vehicle follows, and a controller that 
	 * defines how the vehicle is controlled over time. (Here we're limited
	 * to longitudinal controls, acceleration and deceleration.)
	 */
	class ScenarioObstacle {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const ScenarioObstacle& scenario_obstacle );
		
	public:
        ScenarioObstacle();
		/**
		 * Constructor: set vehicle, path, and controller
		 */
		ScenarioObstacle ( Vehicle& vehicle, Path& path, Controller& controller );
		
		/**
		 * Copy constructor
		 */
        ScenarioObstacle ( const ScenarioObstacle& scenario_obstacle );

		/**
		 * Destructor
		 */
		~ScenarioObstacle ();
		
		/**
		 * Return the minimum extent of this obstacle on the x axis at a given
		 * time.
		 *
		 * NOTE: The time parameter is currently unused since a perpendicular
		 * path is assumed. Eventually it will be used in conjunction with the
		 * controller to compute the position for an arbitrary path.
		 */
		double getMinX( double time ) const;
		
		/**
		 * Return the maximum extent of this obstacle on the x axis at a given
		 * time.
		 *
		 * NOTE: The time parameter is currently unused since a perpendicular
		 * path is assumed. Eventually it will be used in conjunction with the
		 * controller to compute the position for an arbitrary path.
		 */
		double getMaxX( double time ) const;
		
		/**
		 * Return a reference to this scenario obstacle's vehicle
		 */
        const Vehicle& getVehicle() const;
		
		/**
		 * Return a reference to this scenario obstacle's path
		 */
        const Path& getPath() const;
		
		/**
		 * Return a reference to this scenario obstacle's controller
		 */
        const Controller& getController() const;
		
	private:
		
		/**
		 * Used by the constructors to initialize fields
		 */
        void init(const Vehicle& vehicle, const Path& path,const Controller& controller );
		
		/**
		 * The vehicle belonging to this scenario obstacle
		 */
        Vehicle vehicle;
		
		/**
		 * The path belonging to this scenario obstacle
		 */
        Path path;
		
		/**
		 * The controller beloning to this scenario
		 */
        Controller controller;

	};

} // end SCIMP_Scenario namespace

#endif

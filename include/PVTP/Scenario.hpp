#ifndef PVTP_SCENARIO_H
#define PVTP_SCENARIO_H

#include <PVTP/Interval.hpp>
#include <PVTP/Constraints.hpp>
#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/ScenarioObstacle.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	/**
	 * This class will contain scenarios, which are sets of obstacle vehicles,
	 * a driver vehicle, and paths and controllers for all vehicles.
	 *
	 * LIMITATIONS: The API currently only supports scenarios in which
	 * the user-controlled vehicle follows a straight-line path, and obstacle
	 * vehicles follow straight-line paths that cross the user-controlled vehicle's
	 * path perpendicularly; so, scenarios that have the driver driving straight
	 * across lanes of traffic. I am working on extending the scenario to handle
	 * elliptical as well as linear paths, and arbitrary angles of intersection.
	 *
	 * SETUP: A scenario consists of a user-controlled vehicle, and a (possibly
	 * empty) set of obstacle vehicles. These two things are described by the
	 * ScenarioUser and ScenarioObstacle classes, respectively. The ScenarioUser
	 * class, in turn, consists of a Vehicle object and a Path object, which
	 * respectively describe the characteristics of the user's vehicle (such as
	 * length, width, velocity, position, etc.), and the path that the vehicle
	 * follows. The ScenarioObstacle class also contains those things, and
	 * additionally contains a controller that defines how the vehicle is 
	 * controlled over time. (In this context, control is limited to longitudinal
	 * control: acceleration or deceleration.) The ScenarioUser class does
	 * not contain a controller, as it user-controlled.
	 */
	class Scenario {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<( std::ostream& out, const Scenario& scenario );
		
	public:
        Scenario ();
		/**
		 * Constructor: Build a scenario for the given vehicle and path
		 */
		Scenario ( Vehicle& vehicle, Path& path );
		
		/**
		 * Constructor: copy constructor
		 */
       Scenario (const Scenario& scenario );
		
		/**
		 * Destructor
		 */
		~Scenario ();
		

        void initSet( Vehicle& vehicle, Path& path);
        void initSet(const Scenario& scenario);
		/**
		 * Add an obstacle to the scenario
		 */
        void addObstacle(ScenarioObstacle& obstacle );
         void addObstacle(const ScenarioObstacle& obstacle );
		/**
		 * Given the scenario, compute the path-time obstacles, and store
		 * them in the obstacle set.
		 *
		 * IMPORTANT: This assumes linear paths for both obstacles and user,
		 * and it assumes that those paths are perpendicular. This is a
		 * simplification for now, future revisions will allow arbitrary configurations.
		 */
		bool computePathTimeObstacles( PVT_ObstacleSet& obstacle_set, Constraints& c ) const;
		
	private:
		
		/**
		 * Measurements of the user vehicle used to construct path-time obstacles
		 */
		double user_half_width;
		double user_length;
		double path_min_x;
		double path_max_x;
		double path_y;
		
		/**
		 * The set of scenario obstacle
		 */
        std::vector<ScenarioObstacle> obstacles;
		
	};
	
	/**
	 * The following constants define tag and attribute names for XML input
	 */
	namespace IO {
	
		/**
		 * The name of the root tag
		 */
		static std::string XmlRootNode = "scenario";
		static std::string XmlScenarioDistanceUnitsAttr = "distanceUnits";
		static std::string XmlScenarioTimeUnitsAttr = "timeUnits";
		static std::string XmlScenarioDirectionAttr = "direction";
		
		/**
		 * Following are attribute names of the constraints tag, which specify individual constraints
		 */
		static std::string XmlConstraintsAlphaAttr = "alpha";
		static std::string XmlConstraintsTimeLimitAttr = "timeLimit";
		static std::string XmlConstraintsTimeStepAttr = "defaultTimeStep";
		static std::string XmlConstraintsMinFinalVelAttr = "minFinalVelocity";
		static std::string XmlConstraintsMaxFinalVelAttr = "maxFinalVelocity";
		static std::string XmlConstraintsMinVelAttr = "minVelocity";
		static std::string XmlConstraintsMaxVelAttr = "maxVelocity";
		static std::string XmlConstraintsMinAccAttr = "minAcceleration";
		static std::string XmlConstraintsMaxAccAttr = "maxAcceleration";
		static std::string XmlConstraintsEpsilonAttr = "epsilon";
		
		/**
		 * The name of the vehicles tag
		 */
		static std::string XmlVehiclesNode = "vehicles";
		
		/**
		 * The name of the vehicle tag
		 */
		static std::string XmlVehicleNode = "vehicle";
		
		/**
		 * The name of the path tag
		 */
		static std::string XmlPathNode = "path";
		
		/**
		 * Following are path tag attributes
		 */
		static std::string XmlPathTypeAttr = "type";
		static std::string XmlPathStartXAttr = "startX";
		static std::string XmlPathEndXAttr = "endX";
		static std::string XmlPathStartYAttr = "startY";
		static std::string XmlPathEndYAttr = "endY";
		
		/**
		 * Following are the tag names of vehicle types
		 */
		static std::string XmlUserVehicleNode = "user";
		static std::string XmlSimulatorVehiclesNode = "simulator";
		
		/**
		 * Following are vehicle attributes
		 */
		static std::string XmlVehicleLengthAttr = "length";
		static std::string XmlVehicleVelAttr = "velocity";
		static std::string XmlVehicleAccAttr = "acceleration";
		static std::string XmlVehicleWidthAttr = "width";

	} // end IO namespace

} // end SCIMP_Scenario namespace

#endif

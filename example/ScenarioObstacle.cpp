#include <PVTP/ScenarioObstacle.hpp>

namespace SCIMP_Scenario {

	ScenarioObstacle::ScenarioObstacle ( Vehicle& vehicle, Path& path, Controller& controller ) {
		this->init( vehicle, path, controller );
	}
	
    ScenarioObstacle::ScenarioObstacle ( const ScenarioObstacle& scenario_obstacle ) {
		this->init( scenario_obstacle.getVehicle(), scenario_obstacle.getPath(), scenario_obstacle.getController() );
	}
	
    ScenarioObstacle::ScenarioObstacle () {}
    ScenarioObstacle::~ScenarioObstacle () {}
    void ScenarioObstacle::init(const Vehicle& vehicle, const Path& path,const Controller& controller ) {
        this->vehicle.initSet( vehicle );
        this->path.initSet(path );
        this->controller.initSet(controller );
		
		// position the vehicle at the beginning of the path
        this->vehicle.setPosition( 0. );
	}
	
	double ScenarioObstacle::getMinX( double time ) const {
        double vehicle_half_width = this->vehicle.getWidth() / 2.;
        return this->path.getInitialPoint().getX() - vehicle_half_width;
	}
	
	double ScenarioObstacle::getMaxX( double time ) const {
        double vehicle_half_width = this->vehicle.getWidth() / 2.;
        return this->path.getInitialPoint().getX() + vehicle_half_width;
	}
	
    const Vehicle& ScenarioObstacle::getVehicle() const {
        return this->vehicle;
	}
	
    const Path& ScenarioObstacle::getPath() const {
        return this->path;
	}
	
    const Controller& ScenarioObstacle::getController() const {
        return this->controller;
	}
	
	std::ostream& operator<<( std::ostream& out, const ScenarioObstacle& scenario_obstacle ) {
		out << "Scenario Obstacle:" << std::endl;
        out << "Vehicle: " << scenario_obstacle.vehicle << std::endl;
        out << "Path: " << scenario_obstacle.path << std::endl;
        out << "Controller: " << scenario_obstacle.controller << std::endl;
		return out;
	}
	
} // end SCIMP_Scenario namespace

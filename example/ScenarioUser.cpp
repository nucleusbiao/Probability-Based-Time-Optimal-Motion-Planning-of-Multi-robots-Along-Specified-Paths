#include <PVTP/Maths.hpp>
#include <PVTP/Utilities.hpp>
#include <PVTP/ScenarioUser.hpp>

namespace SCIMP_Scenario {

ScenarioUser::ScenarioUser ( Vehicle& vehicle,
                             Path& path,
                             double final_vel_min,
                             double final_vel_max,
                             double time_step,
                             double alpha,
                             Constraints& c ) {
    this->alpha = Utilities::truncateValue( alpha, 0., 1. );
    this->time_step = time_step;
    this->constraints.initSet( c );
    this->current_time = 0.;
    this->vehicle.initSet(vehicle);
    this->path.initSet(path);
    this->scenario.initSet( vehicle, path );
    this->final_velocity_interval.initSet( final_vel_min, final_vel_max );
    this->initialized = false;
    this->positionSeq.clear();
    //this->scimp_controller = NULL;
}

ScenarioUser::ScenarioUser ( const ScenarioUser& user ) {
    this->alpha = user.alpha;
    this->constraints.initSet( user.getConstraints() );
    this->current_time = user.getCurrentTime();
    this->vehicle.initSet( user.getVehicle() );
    this->path.initSet( user.getPath());
    this->scenario.initSet(user.scenario);
    this->final_velocity_interval.initSet(user.getFinalVelocityInterval());
    this->initialized = user.initialized;
    this->scimp_controller.initSet(user.scimp_controller);
    this->positionSeq.assign(user.positionSeq.begin(),user.positionSeq.end());
    /*if ( this->initialized ) {
            this->initializeScenario();
        }*/
}

ScenarioUser::ScenarioUser::~ScenarioUser(){
}
void ScenarioUser::init( Vehicle& vehicle,
                         Path& path,
                         double final_vel_min,
                         double final_vel_max,
                         double time_step,
                         double alpha,
                         Constraints& c ) {
    this->alpha = Utilities::truncateValue( alpha, 0., 1. );
    this->time_step = time_step;
    this->constraints.initSet( c );
    this->current_time = 0.;
    this->vehicle.initSet(vehicle);
    this->path.initSet(path);
    this->scenario.initSet( vehicle, path );
    this->final_velocity_interval.initSet( final_vel_min, final_vel_max );
    this->initialized = false;
    this->positionSeq.clear();
}

void ScenarioUser::initializeScenario() {
    this->scimp_controller.initSet( this->scenario, this->vehicle, this->alpha, this->constraints );
    this->initialized = true;
}

const SCIMP_Controller& ScenarioUser::getSCIMPController() const {
    return (this->scimp_controller);
}

const Interval& ScenarioUser::getFinalVelocityInterval() const {
    return (this->final_velocity_interval);
}

bool ScenarioUser::setCurrentTime( double time ) {
    if ( !this->constraints.validT(time) ) {
        std::cerr << "ScenarioUser::setCurrentTime: Tried to set an invalid time: " << time << ", ignoring." << std::endl;
        return false;
    }
    this->current_time = time;
    return true;
}

double ScenarioUser::getCurrentTime() const {
    return this->current_time;
}

bool ScenarioUser::pathTraversed() const {
    return this->vehicle.getPosition() >= this->constraints.getXLimit();
}

bool ScenarioUser::timeLimitReached() const {
    return this->current_time >= this->constraints.getTLimit();
}

void ScenarioUser::applyUnfilteredControl( double control, double time_step ) {

    // apply control
    this->vehicle.applyControl( control, time_step );

    // accumulate time
    this->current_time = this->current_time + time_step;

}
/*
void ScenarioUser::generatePositionSeq(double timeStep) {
    ScenarioUser temp(*this);
    this->positionSeq.clear();
    double user_control = temp.vehicle.getMaximumAcceleration();
    this->positionSeq.push_back(0);
    while(!temp.pathTraversed()){
        temp.applyFilteredControl(user_control, timeStep);
        this->positionSeq.push_back(temp.vehicle.getPosition());
    }
}*/

bool ScenarioUser::applyFilteredControl( double user_control, double time_step, bool ignore_scimp ) {
    double dummy = -1.;
    return this->applyFilteredControl( dummy, user_control, time_step, ignore_scimp );
}

bool ScenarioUser::computeFilteredControl( double& filtered_control, double& planner_control, double user_control, double time_step,bool& successFlag, bool ignore_scimp ) {

    // force scenario to be initialized first
    if ( !this->initialized ) {
        std::cerr << "ERROR: ScenarioUser::applyFilteredControl: Attempted to apply control to uninitialized scenario. Remember to run ScenarioUser::initializeScenario() first." << std::endl;
        return std::numeric_limits<double>::quiet_NaN();
    }

    // make sure control is within bounds
    user_control = Utilities::truncateValue( user_control,
                                             this->vehicle.getMinimumAcceleration(),
                                             this->vehicle.getMaximumAcceleration() );

    // apply scimp filter to this control
    double current_velocity = this->vehicle.getVelocity();
    double current_path_position = this->vehicle.getPosition();
    double current_time = this->current_time;

    return this->scimp_controller.filterUserControl( this->final_velocity_interval,
                                                     filtered_control,
                                                     planner_control,
                                                     user_control,
                                                     current_velocity,
                                                     time_step,
                                                     current_path_position,
                                                     current_time,
                                                     successFlag,
                                                     ignore_scimp );

}

bool ScenarioUser::applyFilteredControl( double& planner_control, double user_control, double time_step, bool ignore_scimp) {

    // compute filtered control
    double filtered_control;
    bool successFlag=true;
    this->computeFilteredControl( filtered_control, planner_control, user_control, time_step,successFlag, ignore_scimp);

    // apply the filtered control without further filtering
    this->applyUnfilteredControl( filtered_control, time_step );
    this->scimp_controller.vehicle.initSet(this->vehicle);
    if(successFlag)
        return true;
    else {
        return false;
    }
}

const Vehicle& ScenarioUser::getVehicle() const {
    return this->vehicle;
}

const Path& ScenarioUser::getPath() const {
    return this->path;
}

const Scenario& ScenarioUser::getScenario() const {
    return this->scenario;
}

const Constraints& ScenarioUser::getConstraints() const {
    return this->constraints;
}

double ScenarioUser::getTimeStep() const {
    return this->time_step;
}

bool ScenarioUser::setVehicleState( double path_position, double velocity ) {
    // verify path position and velocity
    if ( !this->constraints.validX(path_position) ) {
        std::cerr << "ScenarioUser::setVehicleState: Tried to set an invalid path position: " << path_position << ", limit is: " << this->constraints.getXLimit() << "." << std::endl;
        return false;
    }
    if ( !this->constraints.validV(velocity) ) {
        std::cerr << "ScenarioUser::setVehicleState: Tried to set an invalid velocity: " << velocity << ", bounds are: [" << this->constraints.getVMin() << ", " << this->constraints.getVMax() << "]." << std::endl;
        return false;
    }
    this->scimp_controller.vehicle.setPosition( path_position );
    this->scimp_controller.vehicle.setVelocity( velocity );
    return true;
}

std::ostream& operator<<( std::ostream& out, const ScenarioUser& scenario_user ) {
    out << "Scenario User:" << std::endl;
    out << "Vehicle: " << scenario_user.getVehicle() << std::endl;
    out << "Path: " << scenario_user.getPath() << std::endl;
    out << "Scenario: " << scenario_user.getScenario() << std::endl;
    out << "Final velocity interval: " << scenario_user.final_velocity_interval << std::endl;
    if ( &(scenario_user.scimp_controller) != NULL ) {
        out << "SCIMP Controller: " << scenario_user.scimp_controller << std::endl;
    }
    out << "Constraints: " << scenario_user.constraints << std::endl;
    return out;
}

} // end SCIMP_Scenario namespace

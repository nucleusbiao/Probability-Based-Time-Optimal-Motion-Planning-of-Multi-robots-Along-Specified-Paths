#include <set>
#include <PVTP/Utilities.hpp>
#include <PVTP/Planner.hpp>
#include <PVTP/SCIMP_Controller.hpp>

namespace SCIMP_Scenario {

SCIMP_Controller::SCIMP_Controller ( Scenario& scenario, Vehicle& vehicle, double alpha, Constraints& c ) {
    this->alpha = alpha;
    this->current_path_translation = 0.;
    this->current_time_translation = 0.;
    this->constraints .initSet( c );
    this->vehicle .initSet(vehicle) ;
    this->obstacle_set.clear();
    scenario.computePathTimeObstacles( this->obstacle_set, c );
}
SCIMP_Controller::SCIMP_Controller ( ) {}
const PVT_ObstacleSet& SCIMP_Controller::getPVTObstacleSet() const {
    return (this->obstacle_set);
}
SCIMP_Controller::~SCIMP_Controller(){}

void SCIMP_Controller::initSet( Scenario& scenario, Vehicle& vehicle, double alpha, Constraints& c){
    this->alpha = alpha;
    this->current_path_translation = 0.;
    this->current_time_translation = 0.;
    this->constraints.initSet(c);
    this->obstacle_set.clear();
    this->vehicle.initSet(vehicle);
    scenario.computePathTimeObstacles( this->obstacle_set, c );
}
void SCIMP_Controller::initSet(const SCIMP_Controller& sc){
    this->alpha=sc.alpha;
    this->current_path_translation =sc.current_path_translation;
    this->current_time_translation = sc.current_time_translation;
    this->constraints .initSet( sc.constraints );
    this->obstacle_set.initSet(sc.obstacle_set,this->constraints);
    this->vehicle .initSet(sc.vehicle) ;
}

bool SCIMP_Controller::addPVTObstacleSet(double tu,double td,double pu,double pd){
    std::vector< std::vector<double> > boxes;
    std::vector<double> box;
    box.push_back( pu );
    box.push_back( pd );
    box.push_back( tu );
    box.push_back( td );

    // add box to obstacle set
    boxes.push_back( box );
    (this-> obstacle_set).add(boxes,this->constraints);
    return true;
}
/**
     * Given a user's control, position on the path, and current time, this
     * analyzes the control for safety, either passing it through, if it's
     * safe, or adjusting it minimally to make it safer.
     */
bool SCIMP_Controller::filterUserControl( Interval& final_velocity_interval,
                                          double& filtered_control,
                                          double& planner_control,
                                          double user_control,
                                          double current_velocity,
                                          double time_step,
                                          double current_path_position,
                                          double current_time,
                                          bool& successFlag,
                                          bool ignore_scimp) {
    successFlag=true;
    // this is how many steps to look ahead
    double time_step_multiplier = 3.;
    time_step = time_step_multiplier * time_step;

    // compute user's change in path and velocity position one time step into the future
    double new_user_position_delta = std::numeric_limits<double>::quiet_NaN();
    double new_user_velocity = std::numeric_limits<double>::quiet_NaN();
    this->vehicle.calculateControlEffects( new_user_velocity,
                                           new_user_position_delta,
                                           user_control,
                                           time_step );
    if ( Maths::isNaN(new_user_velocity) || Maths::isNaN(new_user_position_delta) ) {
        std::cerr << "SCIMP_Controller::filterUserControl: Error computing lookahead offsets. Reverting to user control for this time step." << std::endl;
        filtered_control = user_control;
        planner_control = user_control;
        return false;
    }

    //std::cout<<"new_user_position_delta:"<<new_user_position_delta<<std::endl;
    //std::cout<<"new_user_velocity:"<<new_user_velocity<<std::endl;
    // compute relative offset from last translation
    double x_offset = current_path_position - this->current_path_translation;
    double t_offset = current_time - this->current_time_translation;
    //std::cout<<" x_offset:"<<x_offset<<std::endl;
    //std::cout<<" t_offset:"<<t_offset<<std::endl;
    // compute lookahead constraints
    double new_x_limit = this->constraints.getXLimit() - (x_offset + new_user_position_delta);
    double new_t_limit = this->constraints.getTLimit() - (t_offset + time_step);
   // std::cout<<" new_x_limit:"<<new_x_limit<<std::endl;
    //std::cout<<" new_t_limit:"<<new_t_limit<<std::endl;
    // translate obstacles one time step into the future
    this->obstacle_set.translateObstacles( -(x_offset + new_user_position_delta),
                                           -(t_offset + time_step) );

    // if these lookahead limits aren't valid, it means we're right near the
    // end of the path, so just pass through user control
    if ( (new_t_limit < 0.) || (new_x_limit < 0.) ) {
        filtered_control = user_control;
        planner_control = user_control;
        return false;
    }

    // update path and time constraints for one time step into the future
    this->constraints.setXLimit( new_x_limit );
    this->constraints.setTLimit( new_t_limit );

    // update current translation
    this->current_path_translation = current_path_position + new_user_position_delta;
    this->current_time_translation = current_time + time_step;

    // run the planner on this scenario
    std::vector<PVT_G*> G_lookahead;
    std::vector<PVT_S*> Goal_lookahead;
    Interval initial_velocity_interval( new_user_velocity, new_user_velocity );
    if ( !Planner::Forward(G_lookahead,
                           Goal_lookahead,
                           initial_velocity_interval,
                           final_velocity_interval,
                           this->obstacle_set,
                           this->constraints) ) {
        std::cerr << "SCIMP_Controller::filterUserControl: The planner encountered an error during lookahead. Reverting to user control for this time step." << std::endl;
        filtered_control = user_control;
        planner_control = user_control;
        return false;
    }

    // undo lookahead offsets
    this->current_path_translation = this->current_path_translation - new_user_position_delta;
    this->current_time_translation = this->current_time_translation - time_step;
    this->constraints.setXLimit( this->constraints.getXLimit() + new_user_position_delta );
    this->constraints.setTLimit( this->constraints.getTLimit() + time_step );
    initial_velocity_interval.init( current_velocity, current_velocity );

    // translate obstacles back to current time step
    this->obstacle_set.translateObstacles( new_user_position_delta,
                                           time_step );

    // if there exist goal-reachable trajectories, pass the user control through
    if ( !Goal_lookahead.empty() ) {
        Utilities::CleanResults( G_lookahead, Goal_lookahead );
        filtered_control = user_control;
        planner_control = user_control;
        return true;
    }

    // goal reachable trajectories do not exist after executing user control,
    // run the planner on current configuration
    std::vector<PVT_G*> G;
    std::vector<PVT_S*> Goal;
    if ( !Planner::Forward(G,
                           Goal,
                           initial_velocity_interval,
                           final_velocity_interval,
                           this->obstacle_set,
                           this->constraints) ) {
        std::cerr << "SCIMP_Controller::filterUserControl: The planner encountered an error during interference. Reverting to user control for this time step." << std::endl;
        planner_control = user_control;
        filtered_control = user_control;
        return false;
    }

    // if there's still no goal-reachable trajectory, fail out, return the user's control
    if ( Goal.empty() ) {
        filtered_control = user_control;
        planner_control = user_control;
        return false;
    }

    /**
         * At this point, the user's control is unsafe, but there exist safe
         * trajectories. We now find one that attempts to minimize the
         * difference from the user's desired control.
         */

    // first, the easy case: if G contains only one vertex, that vertex must
    // be the origin, which means there are no obstacles,
    // and we can connect directly to the goal
    if ( G.size() == 1 ) {
        Utilities::SanitizeTrajectory( *Goal.at(0)->UB, this->constraints );

        double system_time_step = time_step / time_step_multiplier;
        double v = Utilities::trajectoryVelocityAtTime( *Goal.at(0)->UB, system_time_step, true );
        planner_control = (v - current_velocity) / system_time_step;

        filtered_control = planner_control;
        return true;
    }

    // run Backward to get the feasible set of intervals
    std::vector<PVT_G*> G_intersect;
    if ( !Planner::Backward(G_intersect,
                            G,
                            Goal,
                            initial_velocity_interval,
                            this->obstacle_set,
                            this->constraints) ) {
        std::cerr << "SCIMP_Controller::filterUserControl: The planner encountered an error during backward propagation (1). Reverting to user control for this time step." << std::endl;
        filtered_control = user_control;
        planner_control = user_control;
        return false;
    }

    // sanity check
    if ( G_intersect.empty() ) {
        std::cerr << "SCIMP_Controller::filterUserControl: The planner encountered an error during backward propagation (2). Reverting to user control for this time step." << std::endl;
        filtered_control = user_control;
        planner_control = user_control;
        return false;
    }

    /**
         * BEGIN: If needed, compute raw planner control
         */
    if ( planner_control >= 0. ) {

        std::vector<TrajectorySegment*> T;
        if ( !Planner::BuildOptimalTrajectory(T, G, Goal, this->obstacle_set, this->constraints) ) {
            std::cerr << "Failure building optimal trajectory. (1)" << std::endl;
            filtered_control = user_control;
            planner_control = user_control;
            return false;
        }
        if ( T.empty() ) {
            std::cerr << "Failure building optimal trajectory. (2)" << std::endl;
            filtered_control = user_control;
            planner_control = user_control;
            return false;
        }
        Utilities::SanitizeTrajectory( T, this->constraints );

        double system_time_step = time_step / time_step_multiplier;
        double v = Utilities::trajectoryVelocityAtTime( T, system_time_step );
        planner_control = (v - current_velocity) / system_time_step;

        Utilities::CleanTrajectory( T );

        if ( ignore_scimp ) {
            filtered_control = planner_control;
            return true;
        }
    }
    /**
         * END: Raw planner control
         */

    /**
         * G_intersect now contains only those reachable velocities at obstacle
         * vertices that contain goal-reachable trajectories. Build a collision-free
         * representative trajectories to each immediately reachable vertex. Let
         * the nearest safe control be one of these representative
         * trajectories, so see which one has a velocity after one time step
         * closest to what the user desired.
         */

    // build a collision-free trajectory to all immediately reachable nodes
    double acc_diff_min = std::numeric_limits<double>::max();
    double scimp_control = user_control;
    PVT_Point origin( 0., 0. );
    bool found_scimp_control = false;
    for ( size_t i=0; i<G_intersect.size(); i++ ) {

        PVT_ObstaclePoint * vertex = G_intersect.at(i)->p;

        // build representative trajectories
        std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > BoundingTrajectories;
        BoundingTrajectories.first = NULL;
        BoundingTrajectories.second = NULL;
        if ( !Planner::NextReachableSet(BoundingTrajectories, origin, *vertex, initial_velocity_interval, this->constraints) ) {
            std::cerr << "SCIMP_Controller::filterUserControl: NextReachableSet failed. Reverting to user control for this time step." << std::endl;
            filtered_control = user_control;
            return false;
        }

        // if vertex isn't immediately reachable, skip
        if ( (BoundingTrajectories.first == NULL) || (BoundingTrajectories.second == NULL) ) {
            continue;
        }

        // determine whether the trajectories are safely executable given the time step
        // this is relatively cheap, so do it first
        // NOTE: This appears to break it... more testing needed
        bool first_exectuable = Utilities::trajectoryIsExecutable( *BoundingTrajectories.first, time_step, Constants::DURATION_SAFE_REMAINDER );
        bool second_exectuable = Utilities::trajectoryIsExecutable( *BoundingTrajectories.second, time_step, Constants::DURATION_SAFE_REMAINDER );

        // if neither is executable, skip
        if ( !first_exectuable && !second_exectuable ) {
            Utilities::CleanTrajectory( *BoundingTrajectories.first );
            Utilities::CleanTrajectory( *BoundingTrajectories.second );
            delete( BoundingTrajectories.first );
            delete( BoundingTrajectories.second );
            continue;
        }

        // collision check trajectories
        std::set<PVT_Obstacle*> inCollision_first;
        if ( first_exectuable && !Collisions::checkTrajectory(inCollision_first, *BoundingTrajectories.first, this->obstacle_set, this->constraints) ) {
            std::cerr << "SCIMP_Controller::filterUserControl: collision check failed (1). Reverting to user control for this time step." << std::endl;
            filtered_control = user_control;
            return false;
        }
        std::set<PVT_Obstacle*> inCollision_second;
        if ( second_exectuable && !Collisions::checkTrajectory(inCollision_second, *BoundingTrajectories.second, this->obstacle_set, this->constraints) ) {
            std::cerr << "SCIMP_Controller::filterUserControl: collision check failed (2). Reverting to user control for this time step." << std::endl;
            filtered_control = user_control;
            return false;
        }

        // if they're both in collision, skip this vertex; while it still
        // might be reachable, we're guaranteed at least one vertex is
        // reachable collision-free, so it's not worth the computation
        // expensive to try to connect to this one
        if ( !inCollision_first.empty() && !inCollision_second.empty() ) {
            Utilities::CleanTrajectory( *BoundingTrajectories.first );
            Utilities::CleanTrajectory( *BoundingTrajectories.second );
            delete( BoundingTrajectories.first );
            delete( BoundingTrajectories.second );
            continue;
        }

        // if the first trajectory is collision free, calculate control for this time step
        double acc_first = std::numeric_limits<double>::max();
        if ( first_exectuable && inCollision_first.empty() ) {
            double system_time_step = time_step / time_step_multiplier;
            double v = Utilities::trajectoryVelocityAtTime( *BoundingTrajectories.first, system_time_step );
            acc_first = (v - current_velocity) / system_time_step;
        }
        double acc_first_diff = fabs(acc_first - user_control);

        // likewise for second
        double acc_second = std::numeric_limits<double>::max();
        if ( second_exectuable && inCollision_second.empty() ) {
            double system_time_step = time_step / time_step_multiplier;
            double v = Utilities::trajectoryVelocityAtTime( *BoundingTrajectories.second, system_time_step );
            acc_second = (v - current_velocity) / system_time_step;
        }
        double acc_second_diff = fabs(acc_second - user_control);

        // update accordingly
        if ( acc_first_diff < acc_diff_min ) {

            acc_diff_min = acc_first_diff;
            scimp_control = acc_first;
            found_scimp_control = true;
        }

        if ( acc_second_diff < acc_diff_min ) {

            acc_diff_min = acc_second_diff;
            scimp_control = acc_second;
            found_scimp_control = true;
        }

        // clean up test trajectories
        Utilities::CleanTrajectory( *BoundingTrajectories.first );
        delete( BoundingTrajectories.first );
        Utilities::CleanTrajectory( *BoundingTrajectories.second );
        delete( BoundingTrajectories.second );

    }

    // release memory
    Utilities::CleanResults( G, Goal );
    Utilities::CleanG( G_intersect );

    if ( found_scimp_control ) {
        filtered_control = scimp_control;
        return true;
    }

    std::cout << "SCIMP_Controller::filterUserControl: Could not find a safe trajectory that was also executable. Reverting to user control for this time step." << std::endl;

    filtered_control = user_control;
    successFlag=false;
    return false;
}

double SCIMP_Controller::getAlpha() const {
    return this->alpha;
}

void SCIMP_Controller::setAlpha( double alpha ) {
    this->alpha = Utilities::truncateValue( alpha, 0., 1. );
}

std::ostream& operator<<( std::ostream& out, const SCIMP_Controller& scimp_controller ) {
    out << "Constraints: " << scimp_controller.constraints << std::endl;
    out << "Alpha: " << scimp_controller.alpha << std::endl;
    out << "Path translation: " << scimp_controller.current_path_translation << std::endl;
    out << "Time translation: " << scimp_controller.current_time_translation << std::endl;
    out << "Obstacle set: " << scimp_controller.obstacle_set << std::endl;
    out << "Vehicle: " << scimp_controller.vehicle << std::endl;
    return out;
}

} // end SCIMP_Scenario namespace


#ifndef PVTP_SCIMP_H
#define PVTP_SCIMP_H

#include <PVTP/PVT_ObstacleSet.hpp>
#include <PVTP/Scenario.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {

/**
     * This class describes a controller that operates on the Safety
     * Constrained Intereference Minimization Principle. It exists as a filter
     * between the user and the user's vehicle to attempt to prevent unsafe
     * controls from being executed. If the user attempts to execute an unsafe
     * control, the controller adjusts that control to make it safer.
     */
class SCIMP_Controller {

    /**
         * Overload the output operator
         */
    friend std::ostream& operator<<( std::ostream& out, const SCIMP_Controller& scimp_controller );

public:
    SCIMP_Controller ();
    /**
         * Constructor: constructs the path-time obstacles for a scenario
         */
    SCIMP_Controller ( Scenario& scenario, Vehicle& vehicle, double alpha, Constraints& c );

    /**
         * Destructor
         */
    ~SCIMP_Controller ();
    void initSet( Scenario& scenario, Vehicle& vehicle, double alpha, Constraints& c) ;
    void initSet(const SCIMP_Controller& sc);
    bool addPVTObstacleSet(double tu,double td,double pu,double pd);
    /**
         * Given a user's control (rate of acceleration/deceleration),
         * current velocity, a time step over which to consider the user's
         * control, position on the path, and current time, this analyzes the
         * control for safety, either passing it through, if it's safe, or
         * adjusting it minimally to make it safer.
         */
    bool filterUserControl( Interval& final_velocity_interval,
                            double& filtered_control,
                            double& planner_control,
                            double user_control,
                            double current_velocity,
                            double time_step,
                            double current_path_position,
                            double current_time,
                            bool& successFlag,
                            bool ignore_scimp = false );

    /**
         * Accessor for SCIMP alpha-level
         */
    double getAlpha() const;

    /**
         * Mutator for SCIMP alpha-level, alpha is truncated to be in [0, 1]
         */
    void setAlpha( double alpha );

    /**
         * Return a reference to the obstacle set
         */
    const PVT_ObstacleSet& getPVTObstacleSet() const;

    /**
         * The set of path-time obstacles for the given scenario
         */
    PVT_ObstacleSet  obstacle_set;

    /**
         * The current translation for the obstacle set; keep track of
         * this to relative offsets don't have to be manually computed every
         * time the filter is called.
         */
    double current_path_translation;
    double current_time_translation;

    /**
         * A pointer to the scenario user's vehicle; the controller does *not*
         * own this object, so don't delete it in the destructor here.
         */
    Vehicle vehicle;

private:

    /**
         * The constraint set used for planning.
         */
    Constraints constraints;

    /**
         * The safety threshold below which the SCIMP controller interferes.
         *
         * alpha \in [0, 1]
         */
    double alpha;

};

} // end SCIMP_Scenario namespace

#endif

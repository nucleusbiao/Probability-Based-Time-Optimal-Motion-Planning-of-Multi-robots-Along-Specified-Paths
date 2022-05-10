#ifdef BUILDGSL
#if BUILDGSL

#ifndef PVTP_SCENARIO_GENERATOR_H
#define PVTP_SCENARIO_GENERATOR_H

#include <PVTP/Scenario.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	namespace IO {
		
		/**
		 * This method generates sets of vehicle scenarios by sampling parameters
		 * uniformly at random, and passing them to GenerateScenarioSet.
		 */
		void GenerateScenarios( Constraints& c,
							   std::string output_path = "data",
							   std::string output_filename = "scenario",
							   size_t num_scenario_sets = 3,
							   size_t num_scenarios_per_set = 3,
							   size_t num_lanes_limit = 8,
							   size_t num_vehicles_per_lane_limit = 10,
							   double initial_y_position = -30.,
							   double lane_stagger_limit = 4.,
							   double lane_offset = 2.,
							   double lane_width = 4.,
							   double lane_length = 1000.,
							   double default_time_step = 0.1,
							   double user_vehicle_length = 5.5,
							   double user_vehicle_width = 3.5,
							   double user_vehicle_initial_velocity = 0.,
							   double mean_space_between_vehicles = 35.,
							   double space_between_vehicles_sigma = 10.,
							   double lane_velocity_sigma = 1.5,
							   double mean_vehicle_width = 3.5,
							   double vehicle_width_sigma = .15,
							   double mean_vehicle_length = 5.5,
							   double vehicle_length_sigma = .25,
							   double alpha = 0.999 );
		
		/**
		 * This method generates a set of vehicle scenarios according to 
		 * normal distributions on the parameters of the scenario
		 */
		void GenerateScenarioSet( Constraints& c,
								 std::string output_path = "data",
								 std::string output_filename = "scenario",
								 size_t num_scenarios = 10,
								 size_t num_lanes = 8,
								 size_t num_vehicles_per_lane = 10,
								 double initial_y_position = -30.,
								 double lane_stagger = 4.,
								 double lane_offset = 2.,
								 double lane_width = 4.,
								 double lane_length = 1000.,
								 double min_final_vel = 0.,
								 double max_final_vel = 10.,
								 double default_time_step = 0.1,
								 double user_vehicle_length = 5.5,
								 double user_vehicle_width = 3.5,
								 double user_vehicle_initial_velocity = 0.,
								 double mean_space_between_vehicles = 35.,
								 double space_between_vehicles_sigma = 10.,
								 double mean_lane_velocity = 7.,
								 double lane_velocity_sigma = 1.5,
								 double mean_vehicle_width = 3.5,
								 double vehicle_width_sigma = .15,
								 double mean_vehicle_length = 5.5,
								 double vehicle_length_sigma = .25,
								 double alpha = 0.999 );
		
	} // end IO namespace
	
} // end SCIMP_Scenario namespace

#endif

#endif
#endif

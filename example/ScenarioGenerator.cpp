#ifdef BUILDGSL
#if BUILDGSL

#include <sstream>
#include <fstream>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <PVTP/ScenarioGenerator.hpp>

namespace SCIMP_Scenario {
	
	namespace IO {
		
		void GenerateScenarios( Constraints& c,
							   std::string output_path,
							   std::string output_filename,
							   size_t num_scenario_sets,
							   size_t num_scenarios_per_set,
							   size_t num_lanes_limit,
							   size_t num_vehicles_per_lane,
							   double initial_y_position,
							   double lane_stagger_limit,
							   double lane_offset,
							   double lane_width,
							   double lane_length,
							   double default_time_step,
							   double user_vehicle_length,
							   double user_vehicle_width,
							   double user_vehicle_initial_velocity,
							   double mean_space_between_vehicles,
							   double space_between_vehicles_sigma,
							   double lane_velocity_sigma,
							   double mean_vehicle_width,
							   double vehicle_width_sigma,
							   double mean_vehicle_length,
							   double vehicle_length_sigma,
							   double alpha ) {
			
			// Random number generator
			const gsl_rng_type * T;
			gsl_rng * r;
			
			gsl_rng_env_setup();
			
			T = gsl_rng_default;
			r = gsl_rng_alloc (T);
			
			long seed = time (NULL) * getpid();
			gsl_rng_set(r, seed);
			
			// generate sets
			for ( size_t i=0; i<num_scenario_sets; i++ ) {

				// sample parameter sets to construct scenario sets
				size_t num_lanes = (size_t)( (double)num_lanes_limit * gsl_rng_uniform(r) );
				double lane_stagger = lane_stagger_limit * gsl_rng_uniform( r );
				double mean_lane_velocity = c.getVMin() + (c.getVMax() - c.getVMin()) * gsl_rng_uniform( r );
				double min_final_vel = c.getVMin() + (c.getVMax() - c.getVMin()) * gsl_rng_uniform( r );
				double max_final_vel = c.getVMin() + (c.getVMax() - c.getVMin()) * gsl_rng_uniform( r );
				
				// make sure min and max final velocities make sense
				if ( min_final_vel > max_final_vel ) {
					double swp = max_final_vel;
					max_final_vel = min_final_vel;
					min_final_vel = swp;
				}
				
				// basename for this set
				std::stringstream ss;
				ss << output_filename << "_" << i;
				
				// generate scenario sets
				GenerateScenarioSet( c,
									output_path,
									ss.str(),
									num_scenarios_per_set,
									num_lanes,
									num_vehicles_per_lane,
									initial_y_position,
									lane_stagger,
									lane_offset,
									lane_width,
									lane_length,
									min_final_vel,
									max_final_vel,
									default_time_step,
									user_vehicle_length,
									user_vehicle_width,
									user_vehicle_initial_velocity,
									mean_space_between_vehicles,
									space_between_vehicles_sigma,
									mean_lane_velocity,
									lane_velocity_sigma,
									mean_vehicle_width,
									vehicle_width_sigma,
									mean_vehicle_length,
									vehicle_length_sigma,
									alpha );

			}

			// free memory used by the random number generator
			gsl_rng_free (r);
			
		}
	
		void GenerateScenarioSet( Constraints& c,
								 std::string output_path,
								 std::string output_filename,
								 size_t num_scenarios,
								 size_t num_lanes,
								 size_t num_vehicles_per_lane,
								 double initial_y_position,
								 double lane_stagger,
								 double lane_offset,
								 double lane_width,
								 double lane_length,
								 double min_final_vel,
								 double max_final_vel,
								 double default_time_step,
								 double user_vehicle_length,
								 double user_vehicle_width,
								 double user_vehicle_initial_velocity,
								 double mean_space_between_vehicles,
								 double space_between_vehicles_sigma,
								 double mean_lane_velocity,
								 double lane_velocity_sigma,
								 double mean_vehicle_width,
								 double vehicle_width_sigma,
								 double mean_vehicle_length,
								 double vehicle_length_sigma,
								 double alpha ) {
			
			// Random number generator
			const gsl_rng_type * T;
			gsl_rng * r;
			
			gsl_rng_env_setup();
			
			T = gsl_rng_default;
			r = gsl_rng_alloc (T);
			
			long seed = time (NULL) * getpid();
			gsl_rng_set(r, seed);
			
			for ( size_t scenario_num=0; scenario_num<num_scenarios; scenario_num++ ) {
				// XML header
				std::stringstream ss;
				ss << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>" << std::endl << std::endl;
				
				// root tag
				ss << "<!-- The 'scenario' tag is the root tag, and it specifies certains constraints on the scenario -->" << std::endl;
				ss << "<" << XmlRootNode << " " << XmlScenarioDistanceUnitsAttr << "=\"meters\" " << XmlScenarioTimeUnitsAttr << "=\"seconds\" ";
				ss << XmlConstraintsTimeLimitAttr << "=\"" << c.getTLimit() << "\" ";
				ss << XmlConstraintsMinFinalVelAttr << "=\"" << min_final_vel << "\" " << XmlConstraintsMaxFinalVelAttr << "=\"" << max_final_vel << "\" ";
				ss << XmlConstraintsTimeStepAttr << "=\"" << default_time_step << "\" " << XmlConstraintsAlphaAttr << "=\"" << alpha << "\">" << std::endl << std::endl;
				
				// tag for vehicles
				ss << "\t<!-- All vehicles in a given scenario are grouped under the 'vehicles' tag -->" << std::endl;
				ss << "\t<" << XmlVehiclesNode << ">" << std::endl << std::endl;
				
				// driver vehicle
				ss << "\t\t<!-- The user-controlled vehicle lives in the 'user' tag; there should be only one vehicle here, others will be ignored -->" << std::endl;
				ss << "\t\t<" << XmlUserVehicleNode << ">" << std::endl << std::endl;
				ss << "\t\t\t<!-- The first 'vehicle' tag within the 'user' tag specifies the user-controlled vehicle -->" << std::endl;
				ss << "\t\t\t<" << XmlVehicleNode << " " << XmlVehicleLengthAttr << "=\"" << user_vehicle_length << "\" ";
				ss << XmlVehicleVelAttr << "=\"" << user_vehicle_initial_velocity << "\" ";
				ss << XmlVehicleWidthAttr << "=\"" << user_vehicle_width << "\" " << XmlConstraintsMinVelAttr << "=\"" << c.getVMin() << "\" ";
				ss << XmlConstraintsMaxVelAttr << "=\"" << c.getVMax() << "\" ";
				ss << XmlConstraintsMinAccAttr << "=\"" << c.getAMin() << "\" " << XmlConstraintsMaxAccAttr << "=\"" << c.getAMax() << "\">" << std::endl << std::endl;
				
				// driver vehicle path
				ss << "\t\t\t\t<!-- The first 'path' tag within the 'vehicle' tag specifies the path that this particular vehicle follows -->" << std::endl;
				ss << "\t\t\t\t<" << XmlPathNode << " " << XmlPathTypeAttr << "=\"linear\" " << XmlPathStartXAttr << "=\"0.0\" ";
				ss << XmlPathEndXAttr << "=\"" << c.getXLimit() << "\" " << XmlPathStartYAttr << "=\"0.0\" " << XmlPathEndYAttr << "=\"0.0\"/>" << std::endl << std::endl;
				
				// close driver vehicle tag
				ss << "\t\t\t</" << XmlVehicleNode << ">" << std::endl << std::endl;
				
				// close user tag
				ss << "\t\t</" << XmlUserVehicleNode << ">" << std::endl << std::endl;
				
				// tag for simulator vehicles
				ss << "\t\t<!-- All simulator controlled vehicles live in the 'simulator' tag; there can be arbitrarily many vehicles here -->" << std::endl;
				ss << "\t\t<" << XmlSimulatorVehiclesNode << ">" << std::endl << std::endl;
				
				// now is when we generate all the vehicles needed by the scenario
				ss << "\t\t\t<!-- Each simulator-controlled vehicle is specified with a 'vehicle' tag -->" << std::endl;
				double stagger = 0.;
				double last_x_position = lane_offset * lane_width;
				for ( size_t i=0; i<num_lanes; i++ ) {
					
					// generate lane velocity
					double lane_velocity = mean_lane_velocity + gsl_ran_gaussian_ziggurat( r, lane_velocity_sigma );
					
					// make sure lane velocity is non-negative
					if ( lane_velocity < 0. ) {
						lane_velocity = 0.;
					}
					
					// generate all vehicles in lane; last open position is the first
					// position in the lane at which cars will be added.
					double last_y_position = initial_y_position;
					for ( size_t j=0; j<num_vehicles_per_lane; j++ ) {
						
						// generate width
						double width = mean_vehicle_width + gsl_ran_gaussian_ziggurat( r, vehicle_width_sigma );
						
						// make sure width is within lane width and at least positive
						if ( Maths::approxGe(width, lane_width, c.getEpsilon()) ) {
							width = lane_width;
						} else if ( Maths::approxLe(width, 0., c.getEpsilon()) ) {
							width = 2. * c.getEpsilon();
						}
						
						// generate length
						double length = mean_vehicle_length + gsl_ran_gaussian_ziggurat( r, vehicle_length_sigma );
						
						// make sure length is at least positive
						if ( Maths::approxLe(length, 0., c.getEpsilon()) ) {
							length = 2. * c.getEpsilon();
						}
						
						// generate space between this vehicle and one in front of it
						double space_between_vehicles = mean_space_between_vehicles + gsl_ran_gaussian_ziggurat( r, space_between_vehicles_sigma );
						
						// make sure space between vehicles is non-negative
						if ( space_between_vehicles < 0. ) {
							space_between_vehicles = length;
						}
						
						// compute this car's position in the lane (reference point at rear license plate)
						double y_position = stagger + last_y_position + space_between_vehicles + length;
						
						// update next available open position
						last_y_position = y_position;
						
						// add vehicle tag to XML
						ss << "\t\t\t<" << XmlVehicleNode << " " << XmlVehicleAccAttr << "=\"0.0\" " << XmlScenarioDirectionAttr << "=\"forward\" ";
						ss << XmlVehicleLengthAttr << "=\"" << length << "\" ";
						ss << XmlVehicleVelAttr << "=\"" << lane_velocity << "\" ";
						ss << XmlVehicleWidthAttr << "=\"" << width << "\" " << XmlConstraintsMinVelAttr << "=\"" << c.getVMin() << "\" ";
						ss << XmlConstraintsMaxVelAttr << "=\"" << c.getVMax() << "\" ";
						ss << XmlConstraintsMinAccAttr << "=\"" << c.getAMin() << "\" " << XmlConstraintsMaxAccAttr << "=\"" << c.getAMax() << "\">" << std::endl;
						
						// add path tag to vehicle tag
						ss << "\t\t\t\t<" << XmlPathNode << " " << XmlPathTypeAttr << "=\"linear\" " << XmlPathStartXAttr << "=\"" << last_x_position << "\" ";
						ss << XmlPathEndXAttr << "=\"" << last_x_position << "\" ";
						ss << XmlPathStartYAttr << "=\"" << last_y_position << "\" " << XmlPathEndYAttr << "=\"" << -(lane_length / 2.) << "\"/>" << std::endl;
						
						// close vehicle tag
						ss << "\t\t\t</" << XmlVehicleNode << ">" << std::endl << std::endl;
						
						// if we hit the lane limit, stop adding vehicles
						if ( last_y_position >= lane_length ) {
							break;
						}
						
					}
					
					// update the x position for cars in this lane
					last_x_position += lane_width;
					stagger += lane_stagger;
					
				}
				
				// close simulator tag
				ss << "\t\t</" << XmlSimulatorVehiclesNode << ">" << std::endl << std::endl;
				
				// close vehicles tag
				ss << "\t</" << XmlVehiclesNode << ">" << std::endl << std::endl;
				
				// close scenario tag
				ss << "</" << XmlRootNode << ">" << std::endl;
				
				// write file to disk
				std::ofstream scenario;
				std::stringstream filename;
				filename << output_path << "/" << output_filename << "_" << scenario_num << ".xml";
				scenario.open( filename.str().c_str() );
				scenario << ss.str();
				scenario.close();
			}
			
			// free memory used by random number generator
			gsl_rng_free (r);
			
		}
	
	} // end IO namespace
	
} // end SCIMP_Scenario namespace

#endif
#endif

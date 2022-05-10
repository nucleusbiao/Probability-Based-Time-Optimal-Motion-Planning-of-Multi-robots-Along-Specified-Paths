#include <fstream>
#include <PVTP/Maths.hpp>
#include <PVTP/Constraints.hpp>
#include <PVTP/ScenarioReader.hpp>
#include <PVTP/Vehicle.hpp>

using namespace PVTP;

namespace SCIMP_Scenario {
	
	namespace IO {
		
		bool LoadFileToString( std::string& text, std::string& path ) {

			std::ifstream input_file;
			input_file.open( path.c_str() );
			
			if( !input_file.good() ) {
				std::cerr << "SCIMP_Scenario::IO::LoadFileToString: Error opening file." << std::endl;
				return false;
			}
			
			while( !input_file.eof() ) {
				std::string line;
				getline( input_file, line );
				text = text + line + "\n";
			}
			input_file.close();
			
			return true;
		}
		
		bool ReadScenarioFromXML( ScenarioUser *& scenario_user,
								 std::string path,
								 double sigma_pos_obs,
								 double sigma_vel_obs,
								 double sigma_pos_user,
								 double sigma_vel_user,
								 double user_path_offset,
								 double user_time_offset,
								 double width_padding,
								 double length_padding
								 ) {
			
			// load data file into text
			std::string text;
			if ( !IO::LoadFileToString(text, path) ) {
				std::cerr << "SCIMP_Scenario::IO::ReadScenarioFromXML: Failed to load file: " << path << std::endl;
				return false;
			}
			
			// read in scenario
			return ReadScenarioFromString( scenario_user,
										  text,
										  sigma_pos_obs,
										  sigma_vel_obs,
										  sigma_pos_user,
										  sigma_vel_user,
										  user_path_offset,
										  user_time_offset,
										  width_padding,
										  length_padding );
		}
		
		bool ReadScenarioFromString( ScenarioUser *& scenario_user,
								 std::string text,
								 double sigma_pos_obs,
								 double sigma_vel_obs,
								 double sigma_pos_user,
								 double sigma_vel_user,
								 double user_path_offset,
								 double user_time_offset,
								 double width_padding,
								 double length_padding
								 ) {
			
			scenario_user = NULL;
			
			// make a non-const copy of the string
			char * cstr;
			cstr = new char[text.size() + 1];
			strcpy( cstr, text.c_str() );
			
			// parse input file
			rapidxml::xml_document<> doc;
			try {
				doc.parse<0>( cstr );
			} catch ( rapidxml::parse_error& e ) {
				std::cerr << "SCIMP_Scenario::IO::ReadScenarioFromXML: " << e.what() << std::endl;
				delete( cstr );
				return false;
			}
			
			// root node must be 'scenario'
			if ( IO::XmlRootNode.compare(doc.first_node()->name()) ) {
				std::cerr << "SCIMP_Scenario::IO::ReadScenarioFromXML: Root node '" << IO::XmlRootNode << "' not found." << std::endl;
				delete( cstr );
				return false;
			}
			rapidxml::xml_node<> * root = doc.first_node();
			
			// storage for scenario constraints
			double t_limit = std::numeric_limits<double>::quiet_NaN();
			double v_final_min = std::numeric_limits<double>::quiet_NaN();
			double v_final_max = std::numeric_limits<double>::quiet_NaN();
			double alpha = std::numeric_limits<double>::quiet_NaN();
			double time_step = std::numeric_limits<double>::quiet_NaN();

			// cycle through attributes
			for ( rapidxml::xml_attribute<> *attr=root->first_attribute(); attr; attr=attr->next_attribute() )  {
				
				double value = atof( attr->value() );
				if ( value==std::numeric_limits<double>::infinity() ) {
					continue;
				}
				
				if ( !IO::XmlConstraintsMinFinalVelAttr.compare(attr->name()) ) {
					if ( value >= 0. ) {
						v_final_min = value;
					}
					continue;
				}
				if ( !IO::XmlConstraintsMaxFinalVelAttr.compare(attr->name()) ) {
					if ( value >= 0. ) {
						v_final_max = value;
					}
					continue;
				}
				if ( !IO::XmlConstraintsTimeLimitAttr.compare(attr->name()) ) {
					if ( value > 0. ) {
						t_limit = value;
					}
					continue;
				}
				if ( !IO::XmlConstraintsAlphaAttr.compare(attr->name()) ) {
					if ( (value >= 0.) && (value <= 1.) ) {
						alpha = value;
					}
					continue;
				}
				if ( !IO::XmlConstraintsTimeStepAttr.compare(attr->name()) ) {
					if ( value > 0. ) {
						time_step = value;
					}
					continue;
				}
				
			}
			if ( Maths::isNaN(t_limit) || Maths::isNaN(v_final_min)
				|| Maths::isNaN(v_final_max) || Maths::isNaN(alpha) || Maths::isNaN(time_step) ) {
				std::cerr << "SCIMP_Scenario::IO::ReadScenarioFromXML: Required scenario constraints either not specified or invalid in scenario tag." << std::endl;
				delete( cstr );
				return false;
			}
			
			// storage for scenario obstacles
            std::vector<ScenarioObstacle*> scenario_obstacles;

			// cycle through children of root, getting constraint and vehicle information
			for ( rapidxml::xml_node<> *node=root->first_node(); node; node=node->next_sibling() ) {
				
				// set the simulator vehicles
				if ( !IO::XmlVehiclesNode.compare(node->name()) ) {
					
					// child nodes: there are user and simulator vehicles
					for ( rapidxml::xml_node<> *child=node->first_node(); child; child=child->next_sibling() ) {
						
						// simulator vehicles
						if ( !IO::XmlSimulatorVehiclesNode.compare(child->name()) ) {
							
							// storage for simulator vehicle attributes
							double width = std::numeric_limits<double>::quiet_NaN();
							double length = std::numeric_limits<double>::quiet_NaN();
							double velocity = std::numeric_limits<double>::quiet_NaN();
							double acceleration = std::numeric_limits<double>::quiet_NaN();
							double v_min = std::numeric_limits<double>::quiet_NaN();
							double v_max = std::numeric_limits<double>::quiet_NaN();
							double a_min = std::numeric_limits<double>::quiet_NaN();
							double a_max = std::numeric_limits<double>::quiet_NaN();
							
							// storage for simulator vehicle path attributes
							double simulator_path_startX = std::numeric_limits<double>::quiet_NaN();
							double simulator_path_endX = std::numeric_limits<double>::quiet_NaN();
							double simulator_path_startY = std::numeric_limits<double>::quiet_NaN();
							double simulator_path_endY = std::numeric_limits<double>::quiet_NaN();
							std::string simulator_path_type;
							
							// cycle through all simulator vehicles
							for ( rapidxml::xml_node<> *vehicle=child->first_node(); vehicle; vehicle=vehicle->next_sibling() ) {
								
								if ( IO::XmlVehicleNode.compare(vehicle->name()) ) {
									continue;
								}
								
								// simulator vehicle attributes
								for ( rapidxml::xml_attribute<> *attr=vehicle->first_attribute(); attr; attr=attr->next_attribute() )  {

									double value = atof( attr->value() );
									if ( value==std::numeric_limits<double>::infinity() ) {
										continue;
									}
									
									// physical attributes
									if ( !IO::XmlVehicleVelAttr.compare(attr->name()) ) {
										velocity = value;
										continue;
									}
									if ( !IO::XmlVehicleAccAttr.compare(attr->name()) ) {
										acceleration = value;
										continue;
									}
									if ( !IO::XmlVehicleLengthAttr.compare(attr->name()) ) {
										length = value + length_padding;
										continue;
									}
									if ( !IO::XmlVehicleWidthAttr.compare(attr->name()) ) {
										width = value + width_padding;
										continue;
									}
									
									// constraint attributes
									if ( !IO::XmlConstraintsMinVelAttr.compare(attr->name()) ) {
										v_min = value;
										continue;
									}
									if ( !IO::XmlConstraintsMaxVelAttr.compare(attr->name()) ) {
										v_max = value;
										continue;
									}
									if ( !IO::XmlConstraintsMinAccAttr.compare(attr->name()) ) {
										a_min = value;
										continue;
									}
									if ( !IO::XmlConstraintsMaxAccAttr.compare(attr->name()) ) {
										a_max = value;
										continue;
									}
									
								}
								
								// simulator vehicle path
								for ( rapidxml::xml_node<> *path=vehicle->first_node(); path; path=path->next_sibling() ) {
									
									if ( IO::XmlPathNode.compare(path->name()) ) {
										continue;
									}
									
									// simulator path attributes
									for ( rapidxml::xml_attribute<> *attr=path->first_attribute(); attr; attr=attr->next_attribute() )  {
										
										double value = atof( attr->value() );
										if ( value==std::numeric_limits<double>::infinity() ) {
											continue;
										}
										if ( !IO::XmlPathStartXAttr.compare(attr->name()) ) {
											simulator_path_startX = value;
											continue;
										}
										if ( !IO::XmlPathEndXAttr.compare(attr->name()) ) {
											simulator_path_endX = value;
											continue;
										}
										if ( !IO::XmlPathStartYAttr.compare(attr->name()) ) {
											simulator_path_startY = value;
											continue;
										}
										if ( !IO::XmlPathEndYAttr.compare(attr->name()) ) {
											simulator_path_endY = value;
											continue;
										}
										if ( !IO::XmlPathTypeAttr.compare(attr->name()) ) {
											// todo
										}
									}
									
								}
								
								// build obstacle vehicle
								if ( Maths::isNaN(width) || Maths::isNaN(length)
									|| Maths::isNaN(v_min) || Maths::isNaN(v_max)
									|| Maths::isNaN(a_min) || Maths::isNaN(a_max) ) {
									continue;
								}
								Vehicle obstacle_vehicle( width, length, v_min, v_max, a_min, a_max );
								obstacle_vehicle.setVelocity( velocity );
								obstacle_vehicle.setAcceleration( acceleration );
								
								// build obstacle vehicle path
								if ( Maths::isNaN(simulator_path_startX) || Maths::isNaN(simulator_path_endX)
									|| Maths::isNaN(simulator_path_startY) || Maths::isNaN(simulator_path_endY) ) {
									std::cout << simulator_path_startX << ", " << simulator_path_endX << ", " << simulator_path_startY << ", " << simulator_path_endY << std::endl;
									continue;
								}
								XY_Point obs_path_begin( simulator_path_startX, simulator_path_startY );
								XY_Point obs_path_end( simulator_path_endX, simulator_path_endY );
								Path obstacle_path( obs_path_begin, obs_path_end );

								// build obstacle constroller
								Controller obstacle_controller;
								obstacle_controller.addControl( 0., 0. );

								// build scenario obstacle
                                scenario_obstacles.push_back( new ScenarioObstacle(obstacle_vehicle, obstacle_path, obstacle_controller) );
							}
							
						}
						
						// user vehicle section
						if ( !IO::XmlUserVehicleNode.compare(child->name()) ) {
							
							// storage for constraint values
							//double x_limit = std::numeric_limits<double>::quiet_NaN();
							double v_min = std::numeric_limits<double>::quiet_NaN();
							double v_max = std::numeric_limits<double>::quiet_NaN();
							double a_min = std::numeric_limits<double>::quiet_NaN();
							double a_max = std::numeric_limits<double>::quiet_NaN();
							double epsilon = std::numeric_limits<double>::quiet_NaN();
							
							// storage for user vehicle attributes
							double user_length = std::numeric_limits<double>::quiet_NaN();
							double user_velocity = std::numeric_limits<double>::quiet_NaN();
							double user_width = std::numeric_limits<double>::quiet_NaN();
							
							// storage for user vehicle path attributes
							double user_path_startX = std::numeric_limits<double>::quiet_NaN();
							double user_path_endX = std::numeric_limits<double>::quiet_NaN();
							double user_path_startY = std::numeric_limits<double>::quiet_NaN();
							double user_path_endY = std::numeric_limits<double>::quiet_NaN();
							std::string user_path_type;
							
							// user vehicle
							for ( rapidxml::xml_node<> *vehicle=child->first_node(); vehicle; vehicle=vehicle->next_sibling() ) {
								
								if ( IO::XmlVehicleNode.compare(vehicle->name()) ) {
									continue;
								}

								// user vehicle attributes
								for ( rapidxml::xml_attribute<> *attr=vehicle->first_attribute(); attr; attr=attr->next_attribute() )  {
									
									double value = atof( attr->value() );
									if ( value==std::numeric_limits<double>::infinity() ) {
										continue;
									}
									
									// physical attributes
									if ( !IO::XmlVehicleLengthAttr.compare(attr->name()) ) {
										user_length = value;
										continue;
									}
									if ( !IO::XmlVehicleVelAttr.compare(attr->name()) ) {
										user_velocity = value;
										continue;
									}
									if ( !IO::XmlVehicleWidthAttr.compare(attr->name()) ) {
										user_width = value;
										continue;
									}
									
									// constraint attributes
									if ( !IO::XmlConstraintsTimeLimitAttr.compare(attr->name()) ) {
										t_limit = value;
										continue;
									}
									if ( !IO::XmlConstraintsMinVelAttr.compare(attr->name()) ) {
										v_min = value;
										continue;
									}
									if ( !IO::XmlConstraintsMaxVelAttr.compare(attr->name()) ) {
										v_max = value;
										continue;
									}
									if ( !IO::XmlConstraintsMinAccAttr.compare(attr->name()) ) {
										a_min = value;
										continue;
									}
									if ( !IO::XmlConstraintsMaxAccAttr.compare(attr->name()) ) {
										a_max = value;
										continue;
									}
									if ( !IO::XmlConstraintsEpsilonAttr.compare(attr->name()) ) {
										epsilon = value;
										continue;
									}
									
								}

								// user vehicle path
								for ( rapidxml::xml_node<> *path=vehicle->first_node(); path; path=path->next_sibling() ) {
									
									if ( IO::XmlPathNode.compare(path->name()) ) {
										continue;
									}
									
									// user path attributes
									for ( rapidxml::xml_attribute<> *attr=path->first_attribute(); attr; attr=attr->next_attribute() )  {
										
										double value = atof( attr->value() );
										if ( value==std::numeric_limits<double>::infinity() ) {
											continue;
										}
										
										if ( !IO::XmlPathStartXAttr.compare(attr->name()) ) {
											user_path_startX = value;
											continue;
										}
										if ( !IO::XmlPathEndXAttr.compare(attr->name()) ) {
											user_path_endX = value;
											continue;
										}
										if ( !IO::XmlPathStartYAttr.compare(attr->name()) ) {
											user_path_startY = value;
											continue;
										}
										if ( !IO::XmlPathEndYAttr.compare(attr->name()) ) {
											user_path_endY = value;
											continue;
										}
										if ( !IO::XmlPathTypeAttr.compare(attr->name()) ) {
											// todo
										}
									}
									
								}
								
								// there should be only one user vehicle
								break;
								
							}

							// build user vehicle path
							if ( Maths::isNaN(user_path_startX) || Maths::isNaN(user_path_endX)
								|| Maths::isNaN(user_path_startY) || Maths::isNaN(user_path_endY) ) {
								continue;
							}
							XY_Point user_path_begin( user_path_startX, user_path_startY );
							XY_Point user_path_end( user_path_endX, user_path_endY );
							Path user_path( user_path_begin, user_path_end );

							// build planner constraints
							Constraints * constraints = NULL;
							try {
								if ( Maths::isNaN(epsilon) ) {
									constraints = new Constraints( user_path_endX - user_path_startX,
																  t_limit,
																  v_min,
																  v_max,
																  a_min,
																  a_max );
								} else {
									constraints = new Constraints( user_path_endX - user_path_startX,
																  t_limit,
																  v_min,
																  v_max,
																  a_min,
																  a_max,
																  epsilon );
								}
							} catch ( int e ) {
								Constraints::exceptionMessage( e );
								std::cout << "Invalid constraints in XML." << std::endl;
								delete( cstr );
								return false;
							}

							// build user vehicle
							if ( Maths::isNaN(user_width) || Maths::isNaN(user_length) ) {
								delete( constraints );
								continue;
							}
							Vehicle user_vehicle( user_width,
												 user_length,
												 constraints->getVMin(),
												 constraints->getVMax(),
												 constraints->getAMin(),
												 constraints->getAMax() );
							user_vehicle.setVelocity( user_velocity );
							user_vehicle.setPosition( user_path_offset );

							// build scenario user
							scenario_user = new ScenarioUser( user_vehicle,
															 user_path,
															 v_final_min,
															 v_final_max,
															 time_step,
															 alpha,
															 *constraints );
							scenario_user->setCurrentTime( user_time_offset );
							
							// free allocated memory
							delete( constraints );

						}
						
					}
					
				}
				
			}

			if ( scenario_user != NULL ) {
				for ( size_t i=0; i<scenario_obstacles.size(); i++ ) {
                    scenario_user->scenario.addObstacle( *scenario_obstacles.at(i) );
                    delete( scenario_obstacles.at(i) );
				}
				scenario_user->initializeScenario();
				delete( cstr );
				return true;
			} else {
				for ( size_t i=0; i<scenario_obstacles.size(); i++ ) {
                    delete( scenario_obstacles.at(i) );
				}
				delete( cstr );
				std::cerr << "ScenarioReader: scenario_user is NULL; aborting" << std::endl;
				return false;
			}
			
		}
		
	} // end IO namespace
	
} // end SCIMP_Scenario namespace

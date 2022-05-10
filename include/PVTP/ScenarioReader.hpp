#ifndef XML_READER
#define XML_READER

#include <string.h>
#include <PVTP/ScenarioUser.hpp>
#include <PVTP/rapidxml/rapidxml.hpp>

namespace SCIMP_Scenario {
	
	/**
	 * This namespace contains I/O functions for the SCIMP controller.
	 */
	namespace IO {
		
		/**
		 * Load a given file into a string. Adapted from:
		 *
		 * http://www.cplusplus.com/forum/general/31874/#msg172576
		 */
		bool LoadFileToString( std::string& text, std::string& path );
	
		/**
		 * Read scenario information specified in the XML file and generate the
		 * appropriate objects for used by the SCIMP system, store at the location
		 * specified by 'user'. Should parsing or scenario generation fail, false is
		 * returned, and user is set to NULL. Otherwise true is returned.
		 *
		 * The optional final parameters allow you to insert noise into the reading.
		 */
		bool ReadScenarioFromString( ScenarioUser *& scenario_user,
								 std::string text,
								 double sigma_pos_obs = 0.,
								 double sigma_vel_obs = 0.,
								 double sigma_pos_user = 0.,
								 double sigma_vel_user = 0.,
								 double user_path_offset = 0.,
								 double user_time_offset = 0.,
								 double width_padding = 0.0,
								 double length_padding = 0.0
								 );
		
		/**
		 * Wrapper for ReadScenarioFromString that takes in a path to an XML file
		 */
		bool ReadScenarioFromXML( ScenarioUser *& scenario_user,
									std::string path,
									double sigma_pos_obs = 0.,
									double sigma_vel_obs = 0.,
									double sigma_pos_user = 0.,
									double sigma_vel_user = 0.,
									double user_path_offset = 0.,
									double user_time_offset = 0.,
									double width_padding = 0.0,
									double length_padding = 0.0
								 );
	} // end IO namespace
	
} // end SCIMP_Scenario namespace

#endif

#ifndef PVTP_SOCKETS
#define PVTP_SOCKETS

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <PVTP/Planner.hpp>
#include <PVTP/ScenarioUser.hpp>
#include <PVTP/Utilities.hpp>
#include <PVTP/ScenarioReader.hpp>

namespace SCIMP_Scenario {
	
	/**
	 * This namespace contains a socket interface for the PVT planner.
	 * http://net.pku.edu.cn/~course/cs501/2011/code/BSD_Socket.t/sockets.pdf
	 */
	namespace SocketInterface {
		
		/**
		 * Create a simple iterative socket server.
		 */
		bool createServer( int portno );
		
		/**
		 * Delimiter for command strings
		 */
		static std::string COMMAND_DELIM = ",";
		
	} // end SocketInterface namespace
	
} // end SCIMP_Scenario namespace

#endif

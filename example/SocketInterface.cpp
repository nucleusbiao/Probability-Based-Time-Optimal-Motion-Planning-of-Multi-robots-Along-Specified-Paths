#include <errno.h>
#include <string.h>
#include <PVTP/SocketInterface.hpp>
#include <PVTP/Utilities.hpp>

namespace SCIMP_Scenario {
	
	namespace SocketInterface {
		
		bool createServer( int portno ) {
			
			int sockfd, newsockfd, n;
			socklen_t clilen;

			struct sockaddr_in serv_addr, cli_addr;
			
			sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
			if ( sockfd < 0 ) { 
				std::cerr << "ERROR opening socket" << std::endl;
				close( sockfd );
				return false;
			} else {
				std::cout << "Socket opened on port " << portno << " with file descriptor " << sockfd << std::endl;
			}
			int yes = 1;
			if ( setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) == -1 ) {
				std::cerr << "ERROR in setsockopt" << std::endl;
				close( sockfd );
				return false;
			}
			
			bzero((char *) &serv_addr, sizeof(serv_addr));
			
			serv_addr.sin_family = AF_INET;
			serv_addr.sin_addr.s_addr = INADDR_ANY;
			serv_addr.sin_port = htons(portno);
			
			int b = bind( sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr) );
			if ( b < 0 ) {
				std::cerr << "ERROR binding to port " << portno << ", bind returned " << b << ", errno reports \"" << strerror(errno) << "\"" << std::endl;
				close( sockfd );
				return false;
			} else {
				std::cout << "Socket bound, bind returned " << b << std::endl;
			}
			
			int l = listen( sockfd, 5 );
			if ( l < 0 ) {
				std::cerr << "ERROR listening to port " << portno << ", listen returned " << l << ", errno reports \"" << strerror(errno) << "\"" << std::endl;
				close( sockfd );
				return false;
			}
			
			std::cout << "Socket server started with pid " << getpid() << ", listening..." << std::endl;
			
			// storage for the scenario
			SCIMP_Scenario::ScenarioUser * user = NULL;
			
			char buffer[4096];
			std::stringstream data;
			while ( 1 ) {

				int connectFd = accept( sockfd, (struct sockaddr *) &cli_addr, &clilen );
				if ( connectFd < 0 ) {
					std::cerr << "Error accepting, accept returned \"" << strerror(errno) << "\"" << std::endl;
					close( sockfd );
					return false;
				}
				
				// try to read file input
				size_t buffer_size = sizeof( buffer );
				int bytes_read;
				do {

					// attempt read--don't block, just loop until you get something
					bytes_read = recv( connectFd, buffer, buffer_size, MSG_DONTWAIT );
					if ( bytes_read >= 0 ) {

						// concatenate buffer to data string
						data << buffer;
						
					} else {
						if ( errno == EAGAIN ) {
							bytes_read = (int)buffer_size;
							continue;
						}
						std::cerr << "Error reading from socket, recv returned \"" << strerror(errno) << "\"" << std::endl;
						std::cerr << "bytes_read: " << bytes_read << ", buffer_size: " << buffer_size << std::endl;
						return false;
					}
					
				} while ( bytes_read >= (int)buffer_size );
				std::string success = "SUCCESS";
				std::cout << success << std::endl;
				int n = send( connectFd, success.c_str(), success.length()+1, MSG_WAITALL );
				if ( n < 0 ) {
					std::cerr << "Write failed (1), send returned \"" << strerror(errno) << "\"" << std::endl;
					shutdown( connectFd, 2 );
					close( connectFd );
					close( sockfd );
					return false;
				}
				
				
				/**
				 * 'data' now contains the XML for the scenario. Run the
				 * scenario and return the set of controls.
				 */
				
				// attempt to load the scenario from 'path' into 'user'
				if ( !SCIMP_Scenario::IO::ReadScenarioFromString(user, data.str()) ) {
					
					std::string err_string = "Scenario creation from input file failed.";
					std::cerr << err_string << std::endl;
					int n = send( connectFd, err_string.c_str(), err_string.length()+1, MSG_WAITALL );
					if ( n < 0 ) {
						std::cerr << "Write failed (2), send returned \"" << strerror(errno) << "\"" << std::endl;
						shutdown( connectFd, 2 );
						close( connectFd );
						close( sockfd );
						return false;
					}
					
					// close this connection
					shutdown( connectFd, 2 );
					close( connectFd );
					close( sockfd );
					
					return false;
				}
				std::cout << "Scenario created successfully, awaiting user input..." << std::endl;
				
				/**
				 * Scenario is now constructed, listen for time step/control pairs
				 */
				double current_time = 0.;
				std::string response;
				while ( 1 ) {
					data.str( "" );
					
					// try to read user input
					char buffer[64];
					size_t buffer_size = sizeof( buffer );
					int bytes_read;
					do {
						
						// attempt read--don't block, just loop until you get something
						bytes_read = recv( connectFd, buffer, buffer_size, MSG_DONTWAIT );
						if ( bytes_read > 0 ) {
							
							// concatenate buffer to data string
							data << buffer;
							
						}  else {
							if ( errno == EAGAIN ) {
								bytes_read = (int)buffer_size;
								continue;
							}
							std::cerr << "Error reading from socket, recv returned \"" << strerror(errno) << "\"" << std::endl;
							std::cerr << "bytes_read: " << bytes_read << ", buffer_size: " << buffer_size << std::endl;
							return false;
						}
						
					} while ( bytes_read >= (int)buffer_size );
					
					// parse out time step and control
					// [time step]:[current position]:[current velocity]:[target position]:[target velocity]
					std::vector<std::string> pieces;
					Utilities::StringExplode( data.str(), COMMAND_DELIM, pieces );

					if ( pieces.size() != 5 ) {
						std::cerr << "Command string improperly formatted: " << data.str() << " skipping." << std::endl;
						continue;
					}
					
					double time_step = atof( pieces[0].c_str() );
					double cur_position = atof( pieces[1].c_str() );
					double cur_velocity = atof( pieces[2].c_str() );
					double des_position = atof( pieces[3].c_str() );
					double des_velocity = atof( pieces[4].c_str() );
					
					// adjust path position and velocity to those recieved from sim
					user->setVehicleState( cur_position, cur_velocity );
					
					// compute control that tries to achieve desired position/velocity
					double user_control = Utilities::getOptimalExecutableControl( cur_velocity, des_position - cur_position, des_velocity, time_step );
					
					// filter the control					
					double scimp_control;
					if ( !user->pathTraversed() && !user->timeLimitReached() && (current_time <= user->getConstraints().getTLimit()) ) {
						
						// Attempt to apply user_control over this time step
						scimp_control = user->applyFilteredControl( user_control, time_step );
						
						// Step forward in time.
						current_time += time_step;
						
					} else {
						//std::cout << "stopping time: " << current_time << "s" << std::endl;
						response = "DONE";
						break;
					}
					
					// write executed control to socket
					// [time step]:[filtered control]:[path position]:[user velocity]
					std::stringstream output;
					output << time_step << COMMAND_DELIM << scimp_control << COMMAND_DELIM << user->getVehicle().getPosition() << COMMAND_DELIM << user->getVehicle().getVelocity();
					n = send( connectFd, output.str().c_str(), output.str().length()+1, MSG_WAITALL );
					if ( n < 0 ) {
						std::cerr << "Failed writing output string to socket, send returned \"" << strerror(errno) << "\"" << std::endl;
						shutdown( connectFd, 2 );
						close( connectFd );
						close( sockfd );
						return false;
					}
					
				}
				std::cout << response << std::endl;
				n = send( connectFd, response.c_str(), response.length()+1, MSG_WAITALL );
				if ( n < 0 ) {
					std::cerr << "Failed writing response string to socket, send returned \"" << strerror(errno) << "\"" << std::endl;
					shutdown( connectFd, 2 );
					close( connectFd );
					close( sockfd );
					return false;
				}
				
				// close this connection
				shutdown( connectFd, 2 );
				close( connectFd );
				close( sockfd );
				break;
			}
			
			std::cout << "Socket closed." << std::endl;
			return true;
		}
	
	} // end SocketInterface namespace
	
} // end SCIMP_Scenario namespace

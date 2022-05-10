#include <PVTP/Utilities.hpp>

namespace PVTP {
	
	namespace Utilities {
		
		bool IntersectionOfReachableSets( std::vector<PVT_G*>& G_intersect,
										 std::vector<PVT_G*>& G1,
										 std::vector<PVT_G*>& G2,
										 Constraints& c ) {
			
			// these should match exactly
			if ( G_intersect.size() != G1.size() ) {
				std::cerr << "ERROR IN: Utilities::IntersectionOfReachableSets: G_intersect and G1 differ in size: " << G_intersect.size() << " " << G1.size() << std::endl;
				return false;
			}

			// iterate over sets, storing union in G1
			size_t G2_index = 0;
			for ( size_t i=0; i<G1.size(); i++ ) {
				
				// these should match exactly
				if ( !G_intersect.at(i)->p->equals(*G1.at(i)->p, c) ) {
					std::cerr << "ERROR IN: Utilities::IntersectionOfReachableSets: G_union and G1, G2 differ at index " << i << "." << std::endl;
					return false;
				}
				
				// verify that we're at the same point
				if ( !G1.at(i)->p->equals(*G2.at(G2_index)->p, c) ) {
					continue;
				}
				
				// store the intersection in a G_intersect
				Interval::intersect( *G_intersect.at(i)->V, *G1.at(i)->V, *G2.at(G2_index)->V, c );
				
				if ( ++G2_index >= G2.size() ) {
					break;
				}
			}
			
			// successful completion
			return true;
		}
		
		void SpecialUnionOfReachableSets( std::vector<PVT_G*>& G1,
										 std::vector<PVT_G*>& G2,
										 Constraints& c ) {
			
			// iterate over sets, storing union in G1
			size_t G2_index = 0;
			for ( size_t i=0; i<G1.size(); i++ ) {
				
				// verify that we're at the same point
				if ( !G1.at(i)->p->equals(*G2.at(G2_index)->p, c) ) {
					//std::cout << "PPP: " << *G1.at(i)->p << " " << *G2.at(G2_index)->p << std::endl;
					continue;
				}

				// create a new vector of intervals that contains those from G1 and G2
				size_t G1_v_size = G1.at(i)->V->size();
				size_t G2_v_size = G2.at(G2_index)->V->size();
				std::vector<Interval*> interval_union;
				for ( size_t j=0; j<G1_v_size; j++ ) {
					interval_union.push_back( G1.at(i)->V->at(j) );
				}
				for ( size_t j=0; j<G2_v_size; j++ ) {
					interval_union.push_back( G2.at(G2_index)->V->at(j) );
				}
				
				// preserve pointer to G1's interval vector
				std::vector<Interval*> * ptr = G1.at(i)->V;
				
				// put an empty interval vector into G1
				G1.at(i)->V = new std::vector<Interval*>();
				
				// perform special union with combined G1 and G2 intervals
				Interval::specialUnionOfIntervals( *G1.at(i)->V, interval_union );
				
				// clear G1's original interval vector
				for ( size_t j=0; j<ptr->size(); j++ ) {
					delete( ptr->at(j) );
				}
				ptr->clear();
				delete( ptr );
				
				if ( ++G2_index >= G2.size() ) {
					break;
				}
			}
			
			// successful completion
			return;
		}
		
		void TranslateAndMirrorObstacleSet( PVT_ObstacleSet& O_backward,
										   PVT_ObstacleSet& O_forward,
										   PVT_Point& p,
										   Constraints& c ) {
			
			// number of obstacles
            size_t obstacle_count = O_forward.obstacles.size();
			
			for ( size_t i=0; i<obstacle_count; i++ ) {
				
                PVT_Obstacle * obs_backwards = new PVT_Obstacle( O_forward.obstacles.at(i), c );
				
                for ( size_t j=0; j<obs_backwards->vertices.size(); j++ ) {
					
                    PVT_ObstaclePoint obs_p = obs_backwards->vertices.at( j );
					
					// mirror about x and t axes
                    obs_p.setCoords( -obs_p.getPathCoord(), -obs_p.getTimeCoord() );
					
					// translate s.t. p is origin
                    obs_p.translate( p.getPathCoord(), p.getTimeCoord() );
				}
				
				// since vertices swap places, their type also changes
                char p0_type = obs_backwards->vertices.at(0).getType();
                char p1_type = obs_backwards->vertices.at(1).getType();
                char p2_type = obs_backwards->vertices.at(2).getType();
                char p3_type = obs_backwards->vertices.at(3).getType();
				
				// make sure the vertices are ordered correctly
                PVT_ObstaclePoint p_swap = obs_backwards->vertices.at( 0 );
                obs_backwards->vertices.at(0) = obs_backwards->vertices.at( 2 );
                obs_backwards->vertices.at(0).setType( p0_type );
                obs_backwards->vertices.at(2) = p_swap;
                obs_backwards->vertices.at(2).setType( p2_type );
				
                p_swap = obs_backwards->vertices.at( 1 );
                obs_backwards->vertices.at(1) = obs_backwards->vertices.at( 3 );
                obs_backwards->vertices.at(1).setType( p1_type );
                obs_backwards->vertices.at(3) = p_swap;
                obs_backwards->vertices.at(3).setType( p3_type );
				
				// add to set
                O_backward.obstacles.push_back( *obs_backwards );
			}
			
			// add pseudo obstacle with only one reachable point that will
			// correspond to the origin of the untransformed system
			double obs_coords[4];
			obs_coords[0] = p.getPathCoord();
			obs_coords[1] = c.getXLimit() + 1.;
			obs_coords[2] = -1.;
			obs_coords[3] = p.getTimeCoord();
			
			PVT_Obstacle * obs_origin = new PVT_Obstacle( obs_coords, c );
			
            O_backward.obstacles.push_back( *obs_origin );
		}
		
		void MirrorAndTranslateReachableSets( std::vector<PVT_G*>& G,
											 PVT_Point& p ) {
			
			// the reachable intervals exist in G, but the points have to
			// be transformed back to the original space
			for ( size_t j=0; j<G.size(); j++ ) {
				
				// point with reachable interval
				PVT_ObstaclePoint * p_reachable = G.at(j)->p;
				
				// first undo mirror about axes
				p_reachable->setCoords( -p_reachable->getPathCoord(), -p_reachable->getTimeCoord() );
				
				// now undo translation
				p_reachable->translate( p.getPathCoord(), p.getTimeCoord() );
			}
			
		}
		
		void CleanResults( std::vector<PVT_G*>& G, std::vector<PVT_S*>& Goal ) {
			CleanG( G );
			CleanGoal( Goal );
		}
		
		void CleanG( std::vector<PVT_G*>& G ) {
			for ( size_t i=0; i<G.size(); i++ ) {
				if ( G.at(i) != NULL ) {
					delete( G.at(i) );
				}
			}
			G.clear();
		}
		
		void CleanGoal( std::vector<PVT_S*>& Goal ) {
			for ( size_t i=0; i<Goal.size(); i++ ) {
				delete( Goal.at(i) );
			}
			Goal.clear();
		}
		
		void CleanTrajectory( std::vector<TrajectorySegment*>& T ) {
			for ( size_t i=0; i<T.size(); i++ ) {
				delete( T.at(i) );
			}
			T.clear();
		}
		
		void SanitizeTrajectory( std::vector<TrajectorySegment*>& T, Constraints& c ) {

			size_t i = 0;
			while ( 1 ) {
				if ( i >= T.size() ) {
					break;
				}
				
				if ( T.at(i)->isNullTransition(c) ) {
					delete( T.at(i) );
					T.erase(T.begin()+i);
				} else {
					i++;
				}
			}
		}
		
		void DescribeResults( std::vector<PVT_G*>& G, std::vector<PVT_S*>& Goal, bool verbose ) {
			DescribeG( G, verbose );
			DescribeGoal( Goal, verbose );
		}
		
		void DescribeG( std::vector<PVT_G*>& G, bool verbose ) {
			int cnt = 0;
			if ( !G.empty() ) {
				for ( size_t i=0; i<G.size(); i++ ) {
					if ( !G.at(i)->V->empty() ) {
						cnt++;
						if ( verbose ) {
							std::cout << "Intervals: ";
							for ( size_t j=0; j<G.at(i)->V->size(); j++ ) {
								std::cout << *G.at(i)->V->at(j) << " ";
							}
							std::cout << std::endl << "Point: " << *G.at(i)->p;
							std::cout << std::endl << std::endl;
						}
					}
				}
			}
			std::cout << "Points with reachable intervals: " << cnt << std::endl;
		}
		
		void DescribeGoal( std::vector<PVT_S*>& Goal, bool verbose ) {
			if ( verbose && !Goal.empty() ) {
				
				// Construct a comparator for sorting
				struct PVT_S_Comparator comp2;
				
				// Sort by value, then by type (min or max) descendingly
				std::sort( Goal.begin(), Goal.end(), comp2 );
				for ( size_t i=0; i<Goal.size(); i++ ) {
					std::cout << "UB: " << std::endl;
					for ( size_t j=0; j<Goal.at(i)->UB->size(); j++ ) {
						std::cout << *Goal.at(i)->UB->at(j) << std::endl;
					}
					std::cout << std::endl;
					std::cout << "LB: " << std::endl;
					for ( size_t j=0; j<Goal.at(i)->LB->size(); j++ ) {
						std::cout << *Goal.at(i)->LB->at(j) << std::endl;
					}
					std::cout << std::endl;
				}
			}
			std::cout << "Goal-reachable intervals: " << Goal.size() << std::endl;
		}
		
		void PrintTrajectory( std::vector<TrajectorySegment*>& T ) {
			for ( size_t i=0; i<T.size(); i++ ) {
				std::cout << *T.at(i) << std::endl;
			}
		}
		
		void PrintObstacles( PVT_ObstacleSet& obs ) {
            PrintObstacles( obs.obstacles );
		}
		
        void PrintObstacles( std::vector<PVT_Obstacle>& obstacles ) {
			for ( size_t i=0; i<obstacles.size(); i++ ) {
                std::cout << "[" << obstacles.at(i).vertices.at(0).getPathCoord();
                std::cout << " " << obstacles.at(i).vertices.at(2).getPathCoord();
                std::cout << " " << obstacles.at(i).vertices.at(2).getTimeCoord();
                std::cout << " " << obstacles.at(i).vertices.at(0).getTimeCoord();
				std::cout << "]" << std::endl;
			}
			std::cout << std::endl;
		}
		
		void PrintSignature( std::vector<char>& hClass ) {
			for ( size_t i=0; i<hClass.size(); i++ ) {
				std::cout << (int)hClass.at(i) << " ";
			}
			std::cout << std::endl;
		}
		
		void ExtractVelocityInterval( Interval& I,
									 std::vector<TrajectorySegment*>& UB,
									 std::vector<TrajectorySegment*>& LB ) {
			double min = LB.back()->getFinalState().getVelocityCoord();
			double max = UB.back()->getFinalState().getVelocityCoord();
			I.setBounds( min, max );
		}
		
		double truncateValue( double value, double min, double max ) {
			if ( value < min ) {
				return min;
			}
			if ( value > max ) {
				return max;
			}
			return value;
		}
		
		double trajectoryVelocityAtTime( std::vector<TrajectorySegment*>& T, double time, bool truncate ) {
			if ( T.empty() || (time < 0.) ) {
				return std::numeric_limits<double>::quiet_NaN();
			}
			
			double initial_time = T.at(0)->getInitialState().getTimeCoord();
			double final_time = T.at(T.size()-1)->getFinalState().getTimeCoord();
			double total_time = final_time - initial_time;
			
			if ( time > total_time ) {
				if ( truncate ) {
					time = total_time;
				} else {
					return std::numeric_limits<double>::quiet_NaN();
				}
			}
			
			double current_velocity = T.at(0)->getInitialState().getVelocityCoord();
			for ( size_t i=0; i<T.size(); i++ ) {
				
				double t1 = T.at(i)->getInitialState().getTimeCoord();
				double t2 = T.at(i)->getFinalState().getTimeCoord();
				double acc = T.at(i)->getAcceleration();
				
				// if time is within this segment, we're done
				if ( time <= t2 ) {
					current_velocity = Maths::V2_FromV1_T_A( current_velocity, time - t1, acc, 0. );
					break;
				}
				
				// otherwise move to the end of the segment, update current velocity, repeat
				current_velocity = T.at(i)->getFinalState().getVelocityCoord();
			}
			
			return current_velocity;
		}
		
		double trajectoryDisplacementAtTime( std::vector<TrajectorySegment*>& T, double time, bool truncate ) {

			if ( T.empty() || (time < 0.) ) {
				return std::numeric_limits<double>::quiet_NaN();
			}
			
			double initial_time = T.at(0)->getInitialState().getTimeCoord();
			double final_time = T.at(T.size()-1)->getFinalState().getTimeCoord();
			double total_time = final_time - initial_time;
			
			if ( time > total_time ) {
				if ( truncate ) {
					time = total_time;
				} else {
					return std::numeric_limits<double>::quiet_NaN();
				}
			}
			
			double current_displacement = 0.;
			for ( size_t i=0; i<T.size(); i++ ) {
				
				double t1 = T.at(i)->getInitialState().getTimeCoord();
				double t2 = T.at(i)->getFinalState().getTimeCoord();
				double acc = T.at(i)->getAcceleration();
				double vel = T.at(i)->getInitialState().getVelocityCoord();
				
				// if time is within this segment, we're done
				if ( (t1+time) <= t2 ) {
					current_displacement += Maths::motionX_FromV1_T1_T2_A( vel, 0., time, acc, 0. );
					break;
				}
				
				// otherwise move to the end of the segment, update displacement, repeat
				current_displacement = T.at(i)->getFinalState().getPathCoord() - T.at(0)->getInitialState().getPathCoord();
			}
			
			return current_displacement;
		}
		
		bool trajectoryIsExecutable( std::vector<TrajectorySegment*>& T, double time_step, double range ) {
			
			// for testing
			return true;
			
			if ( T.empty() || (time_step < 0.) || (range < 0.) ) {
				return false;
			}
			if ( range > time_step ) {
				return true;
			}
			
			// get the duration of each segment
			// compute the remainder when divided by the time step
			// compare (time_step - remainder) with range
			for ( size_t i=0; i<T.size(); i++ ) {
				double duration = T.at(i)->getDuration();
				double from_last_time_step = fmod( duration, time_step );
				double to_next_time_step = time_step - from_last_time_step;
				if ( (from_last_time_step > range) && (to_next_time_step > range) ) {
					return false;
				}
			}
			
			return true;
		}
		
		double getOptimalExecutableControl( std::vector<TrajectorySegment*>& T, double time_step ) {
			
			if ( T.empty() || (time_step==0.) ) {
				return 0.;
			}
			
			// get what displacement and velocity would be under perfect control
			double initial_velocity = T.at(0)->getInitialState().getVelocityCoord();
			double optimal_velocity = trajectoryVelocityAtTime( T, time_step );
			double optimal_displacement = trajectoryDisplacementAtTime( T, time_step );
			
			return getOptimalExecutableControl( initial_velocity, optimal_displacement, optimal_velocity, time_step );
			
		}
		
		double getOptimalExecutableControl( double initial_velocity, double optimal_displacement, double optimal_velocity, double time_step, double C1, double C2 ) {
			
			// compute executable control that minimizes deviation from vel + displacement
			// Err = optimal_velocity - (v + a * t) + optimal_displacement - (v * t + 1/2 * a * t^2)

			// solve assuming Err = 0
			return (C1 * (optimal_velocity - initial_velocity) + C2 * (optimal_displacement - initial_velocity * time_step)) / (C1 * time_step + 0.5 * C2 * time_step * time_step);
		}
		
		void StringExplode( std::string str, std::string separator, std::vector<std::string>& results ) {
			int found = str.find_first_of(separator);
			while ( found != (int)std::string::npos ) {
				if ( found > 0 ) {
					results.push_back(str.substr(0,found));
				}
				str = str.substr( found + 1 );
				found = str.find_first_of( separator );
			}
			if( str.length() > 0 ) {
				results.push_back(str);
			}
		}
		
	} // end Utilities namespace

} // end PVTP namespace

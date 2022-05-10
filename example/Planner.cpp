#include <stdio.h>
#include <errno.h>
#include <PVTP/Maths.hpp>
#include <PVTP/Constants.hpp>
#include <PVTP/Utilities.hpp>
#include <PVTP/Planner.hpp>

namespace PVTP {
	
	namespace Planner {
		
		bool Backward( std::vector<PVT_G*>& G_intersect,
					  std::vector<PVT_G*>& G_forward,
					  std::vector<PVT_S*>& Goal_forward,
					  Interval& V_f,
					  PVT_ObstacleSet& O_forward,
					  Constraints& c_forward ) {
			
			// constraint set for backwards problem: accelerations are inverted
			Constraints c_backward( c_forward, true );
			
			// ensure that intervals are sorted
			struct PVT_G_Comparator comp;
			comp.epsilon = c_forward.getEpsilon();
			std::sort( G_forward.begin(), G_forward.end(), comp );
			
			struct PVT_S_Comparator goal_comp;
			std::sort( Goal_forward.begin(), Goal_forward.end(), goal_comp );
			
			// this set will contain the special union of the backwards 
			// reachable intervals
			std::vector<PVT_G*> G_union;

			// initialize union and intersection sets with empty intervals
			for ( size_t i=0; i<G_forward.size(); i++ ) {
				G_union.push_back( new PVT_G( *G_forward.at(i)->p) );
				G_intersect.push_back( new PVT_G( *G_forward.at(i)->p) );
			}
			
			// for each point with goal-reachable velocities, mirror the
			// problem and run the forward algorithm
			for ( size_t i=0; i<Goal_forward.size(); i++ ) {
				
				// point about which problem is to be mirrored
				PVT_ObstaclePoint p = *Goal_forward.at(i)->p;
				
				// translate and mirror the problem for this point
				PVT_ObstacleSet O_backward;
				Utilities::TranslateAndMirrorObstacleSet( O_backward,
														 O_forward,
														 p,
														 c_backward );
				
				// run forward propagation on this obstacle set
				// don't do goal propagation; it isn't necessary for this
				std::vector<PVT_G*> G;
				std::vector<PVT_S*> Goal_dummy;
				Interval V_i_backward;
				Goal_forward.at(i)->getReachableInterval( V_i_backward );
				if ( !Forward(G, Goal_dummy, V_i_backward, V_f, O_backward, c_backward, false, false) ) {
					std::cerr << "ERROR IN Planner::Backward: Backward propagation failed." << std::endl;
					return false;
				}
				
				
				// the reachable intervals exist in G, but the points have to
				// be transformed back to the original space
				Utilities::MirrorAndTranslateReachableSets( G, p );
				
				// sort the intervals so that the order matches G_forward
				struct PVT_G_Comparator comp;
				comp.epsilon = c_forward.getEpsilon();
				std::sort( G.begin(), G.end(), comp );
				
				/**
				 * G now contains the backwards reachable intervals for the
				 * i-th point with goal-reachable intervals, ordered as in
				 * G_forward
				 */
				
				// union G with G_union to accumulate a union of backwards
				// reachable intervals
				Utilities::SpecialUnionOfReachableSets(G_union, G, c_forward);
				
				// clean memory for this iteration
				Utilities::CleanResults( G, Goal_dummy );
			}

			/**
			 * G_union now contains the special union of the backwards reachable
			 * intervals. The intersection of this set with that of the forward
			 * set will yield exacty the set of intervals that yield
			 * goal-reachable trajectories
			 */
			if ( !Utilities::IntersectionOfReachableSets(G_intersect, G_union, G_forward, c_forward) ) {
				std::cerr << "ERROR IN Planner::Backward: Intersection of forwards and backwards reachable sets failed." << std::endl;
				return false;
			}
			
			// clean union memory
			Utilities::CleanG( G_union );
			
			// success
			return true;
		}
		
		bool Forward( std::vector<PVT_G*>& G,
					 std::vector<PVT_S*>& Goal,
					 Interval& _V_i,
					 Interval& _V_f,
					 PVT_ObstacleSet& O,
					 Constraints& c,
					 bool do_goal_propagation,
					 bool add_origin ) {
			
			// get our own copy of the intervals
			Interval V_i( _V_i );
			Interval V_f( _V_f );
			
			// make sure input is within bounds
			Interval v_feasible( c.getVMin(), c.getVMax() );
			V_i.intersect( v_feasible, c );
			V_f.intersect( v_feasible, c );
			if ( V_i.isEmpty() || V_f.isEmpty() ) {
				std::cerr << "ERROR IN Planner::Forward: Input velocities invalid, V_i: " << V_i << ", V_f: " << V_f << ", v_feasible: " << v_feasible << std::endl;
				return false;
			}
			
			// get points
			std::vector<PVT_ObstaclePoint*> P_t;
            O.getAllVertices( P_t, c );
			
			// add origin, if necessary
			PVT_ObstaclePoint p1( 0., 0., Constants::H_CLASS_ORIGIN, c );
			if ( add_origin ) {
				P_t.push_back( &p1 );
			}
			
			// sort points by t-coord
			struct PVT_ObstaclePointComparator comp;
			comp.epsilon = c.getEpsilon();
			std::sort( P_t.begin(), P_t.end(), comp );
			
			// allocate storage
			G.resize( P_t.size(), NULL );
			
			// initialize with first point
			G.at(0) = new PVT_G( V_i, *P_t.at(0) );
			
			// try to connect points
			for ( size_t j=0; j<P_t.size(); j++ ) {
				
				// destination point
				PVT_ObstaclePoint * p2 = P_t.at(j);
				
				// entry for this point in G
				if ( G.at(j) == NULL ) {
					G.at(j) = new PVT_G( *p2 );
				}

				// storage for reachable velocity intervals
				std::vector<PVT_S*> S;
				for ( size_t l=0; l<G.at(j)->V->size(); l++ ) {
					S.push_back( new PVT_S(*G.at(j)->V->at(l)) );
				}
				
				// only need to try connecting points before this in time
				for ( size_t i=0; i<j; i++ ) {
					
					// source point
					PVT_ObstaclePoint * p1 = P_t.at(i);
					
					// initial velocity intervals at point i
					std::vector<Interval*> * V_i = G.at(i)->V;

					// for each disjoing interval in V_i
					for ( size_t k=0; k<V_i->size(); k++ ) {
						
						// continuous velocity interval
						Interval * V_int = V_i->at(k);
						
						// build set S for this connection
						if ( !Propagate(S, *p1, *p2, *V_int, O, P_t, c) ) {
							std::cerr << "ERROR IN Planner::Forward: propagation failed" << std::endl;
							return false;
						}
					}
				}
				
				// store information for this point
				Merge( *G.at(j)->V, S, c );
				
				// Clean S
				for ( size_t k=0; k<S.size(); k++ ) {
					delete( S.at(k) );
				}
				
				if ( do_goal_propagation ) {
					std::vector<PVT_S*> S_goal;
					for ( size_t k=0; k<G.at(j)->V->size(); k++ ) {
						Interval * V_int = G.at(j)->V->at(k);
						if ( !PropagateGoal(S_goal, *G.at(j)->p, *V_int, V_f, O, P_t, c) ) {
							std::cerr << "ERROR IN Planner::Forward: PropagateGoal failed" << std::endl;
							return false;
						}
					}
					
					// merge goal velocities, add to goal intervals
					MergeGoal( Goal, S_goal, c );

					// Clean S_goal
					for ( size_t k=0; k<S_goal.size(); k++ ) {
						delete( S_goal.at(k) );
					}
				}
			}
			return true;
		}
		
		bool Propagate( std::vector<PVT_S*>& S,
					   PVT_Point& p1,
					   PVT_Point& p2,
					   Interval& V_int,
					   PVT_ObstacleSet& O,
					   std::vector<PVT_ObstaclePoint*>& P_t,
					   Constraints& c ) {
			
			// check reachability
			std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > BoundingTrajectories;
			BoundingTrajectories.first = NULL;
			BoundingTrajectories.second = NULL;
			if ( !NextReachableSet(BoundingTrajectories, p1, p2, V_int, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: NextReachableSet failed" << std::endl;
				return false;
			}
			if ( (BoundingTrajectories.first == NULL) || (BoundingTrajectories.second == NULL) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::Propagate: No bounding trajectories found" << std::endl;
#endif
				return true;
			}
			
			// get trajectory signatures
			std::vector<char> BL;
			if ( !Channel(BL, *BoundingTrajectories.second, O, P_t, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: Channel failed (1)" << std::endl;
				return false;
			}
			
			std::vector<char> BU;
			if ( !Channel(BU, *BoundingTrajectories.first, O, P_t, c) ) {
				std::cerr << "ERROR IN Planner::Propagate: Channel failed (2)" << std::endl;
				return false;
			}
			
			if ( BL.empty() && BU.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::Propagate: Empty channel classes" << std::endl;
#endif
				// free memory used by trajectories
				Utilities::CleanTrajectory( *BoundingTrajectories.first );
				delete( BoundingTrajectories.first );
				Utilities::CleanTrajectory( *BoundingTrajectories.second );
				delete( BoundingTrajectories.second );
				
				return true;
			}
						
			//
			// Add velocity intervals
			//
			
			// if signatures are equal
			bool sigs_equal = BL.size() == BU.size();
			if ( sigs_equal ) {
				for ( size_t i=0; i<BL.size(); i++ ) {
					if ( BU.at(i) != BL.at(i) ) {
						sigs_equal = false;
						break;
					}
				}
			}
			
			// both bounds reachable
			if ( sigs_equal ) {
				double v_min = BoundingTrajectories.second->back()->getFinalState().getVelocityCoord();
				double v_max = BoundingTrajectories.first->back()->getFinalState().getVelocityCoord();
				Interval i( v_min, v_max );
				S.push_back( new PVT_S( i, BL ) );
			
			// only singletons reachable
			} else {
				
				if ( !BL.empty() ) {
					double v_min = BoundingTrajectories.second->back()->getFinalState().getVelocityCoord();
					Interval i( v_min, v_min );
					S.push_back( new PVT_S( i, BL ) );
				}
				
				if ( !BU.empty() ) {
					double v_max = BoundingTrajectories.first->back()->getFinalState().getVelocityCoord();
					Interval i( v_max, v_max );
					S.push_back( new PVT_S( i, BU ) );
				}				
			}
			
			// free memory used by trajectories
			Utilities::CleanTrajectory( *BoundingTrajectories.first );
			delete( BoundingTrajectories.first );
			Utilities::CleanTrajectory( *BoundingTrajectories.second );
			delete( BoundingTrajectories.second );
			
#ifdef SHOW_COMMENTS
			std::cout "Propagated: " << V_int << ": " << p1 << " -> " << p2 << std::endl;
#endif
			return true;
		}
		
		bool PropagateGoal( std::vector<PVT_S*>& S_goal,
						   PVT_ObstaclePoint& p1,
						   Interval& V_i,
						   Interval& V_f,
						   PVT_ObstacleSet& O,
						   std::vector<PVT_ObstaclePoint*>& P_t,
						   Constraints& c ) {
			
			// check reachability
			std::vector<TrajectorySegment*> UB;
			std::vector<TrajectorySegment*> LB;
			if ( !GoalConnect(UB, LB, p1, V_i, V_f, O, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: GoalConnect failed" << std::endl;
				return false;
			}
			if ( UB.empty() || LB.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::PropagateGoal: Goal unreachable from " << p1 << std::endl;
#endif
				return true;
			}
			
			// get trajectory signatures
			std::vector<char> BL;
			if ( !Channel(BL, LB, O, P_t, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: Channel failed (1)" << std::endl;
				return false;
			}
			std::vector<char> BU;
			if ( !Channel(BU, UB, O, P_t, c) ) {
				std::cerr << "ERROR IN Planner::PropagateGoal: Channel failed (1)" << std::endl;
				return false;
			}
			if ( BL.empty() && BU.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::PropagateGoal: Empty channel classes, collision propagating " << p1 << std::endl;
#endif
				Utilities::CleanTrajectory( UB );
				Utilities::CleanTrajectory( LB );
				
				return true;
			}
			
			//
			// Add velocity intervals
			//
			
			// if signatures are equal
			bool sigs_equal = BL.size() == BU.size();
			if ( sigs_equal ) {
				for ( size_t i=0; i<BL.size(); i++ ) {
					if ( BU.at(i) != BL.at(i) ) {
						sigs_equal = false;
						break;
					}
				}
			}
			
			// both bounds reachable
			if ( sigs_equal ) {
				S_goal.push_back( new PVT_S( UB, LB, BL, p1 ) );
				
			// only singletons reachable
			} else {
				
				if ( !BL.empty() ) {
					S_goal.push_back( new PVT_S( LB, LB, BL, p1 ) );
				} else {
					Utilities::CleanTrajectory( LB );
				}
				
				if ( !BU.empty() ) {
					S_goal.push_back( new PVT_S( UB, UB, BU, p1 ) );
				} else {
					Utilities::CleanTrajectory( UB );
				}
			}
#ifdef SHOW_COMMENTS
			std::cout << "Propagated Goal: " << V_i << ": " << p1;
			for ( size_t i=0; i<BL->size(); i++ ) {
				std::cout << ((BL->at(i)==0)?"0":"1") << " ";
			}
			std::cout << std::endl;
			for ( size_t i=0; i<BU->size(); i++ ) {
				std::cout << ((BU->at(i)==0)?"0":"1") << " ";
			}
			std::cout << std::endl;
#endif			
			return true;
		}
		
		bool Channel( std::vector<char>& hClass,
					   std::vector<TrajectorySegment*>& T,
					   PVT_ObstacleSet& O,
					   std::vector<PVT_ObstaclePoint*>& P,
					   Constraints& c ) {
			if ( T.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::Channel: empty trajectory" << std::endl;
#endif
				hClass.clear();
				return true;
			}
			
			double epsilon = c.getEpsilon();
			
			std::set<PVT_Obstacle*> inCollision;
			if ( !Collisions::checkTrajectory(inCollision, T, O, c) ) {
				std::cerr << "ERROR IN Planner::Channel: collision check failed" << std::endl;
				hClass.clear();
				return false;
			}
			if ( !inCollision.empty() ) {
				hClass.clear();
				return true;
			}
			
			for ( size_t i=0; i<P.size(); i++ ) {
				PVT_ObstaclePoint * p = P.at(i);
				
				double p_x = p->getPathCoord();
				double p_t = p->getTimeCoord();
				
				//
				// Cycle through trajectory segments, finding the highest time
				// for this obstacle point
				//

				PVT_State * s1 = NULL;
				PVT_State * s2 = NULL;
				for ( size_t tmp=0; tmp<T.size(); tmp++ ) {

					if ( s1 == NULL ) {

						s1 = &T.at(tmp)->getInitialState();
						s2 = &T.at(tmp)->getFinalState();

						// add first point
						PVT_Point p_tmp( s1->getPathCoord(), s1->getTimeCoord() );
						if ( p_tmp.equals(*((PVT_Point*)p), c) ) {
							hClass.push_back( p->getType() );
							break;
						}
						
					} else {
						s2 = &T.at(tmp)->getFinalState();
					}

					// skip duplicates
					if ( s1->equals(*s2, c) ) {
						s1 = s2;
						continue;
					}
					
					// parameters for this segment
					double x1 = s1->getPathCoord();
					double t1 = s1->getTimeCoord();
					double v1 = s1->getVelocityCoord();
					double t2 = s2->getTimeCoord();
					double v2 = s2->getVelocityCoord();

					// if the point is wholly above or below the time range of the trajectory, we're done
					if ( Maths::approxLe(p_t, t1, epsilon) || Maths::approxGt(p_t, t2, epsilon) ) {
						s1 = s2;
						continue;
					}
					
					//
					// At this point, p exists somewhere before or after the segment on the path
					//

					// path coordinate of trajectory at p's time coordinate
					double acc = Maths::A_FromV1_V2_T1_T2( v1, v2, t1, t2, epsilon );
					if ( Maths::isNaN(acc) ) {
						std::cerr << "ERROR IN Planner::Channel: invalid acceleration" << std::endl;
						return false;
					}
					double path_x = x1 + Maths::motionX_FromV1_T1_T2_A( v1, 0., p_t - t1, acc, epsilon );
					
					// compare this to p's path coordinate
					if ( Maths::approxEq(path_x, p_x, epsilon) ) {
						hClass.push_back( p->getType() );
						break;
					} else if ( Maths::approxGt(path_x, p_x, epsilon) ) {
						hClass.push_back( Constants::H_CLASS_BELOW );
						break;
					} else {
						hClass.push_back( Constants::H_CLASS_ABOVE );
						break;
					}

				}
			
			}
			
			// in the absence of any viable points, the trajectory belongs to the origin class
			if ( hClass.empty() ) {
				hClass.push_back( Constants::H_CLASS_ORIGIN );
			}
			
			return true;
		}
		
		bool isSuffix( std::vector<char>& B, std::vector<char>& B_prime ) {
			if ( B.size() > B_prime.size() ) {
				return false;
			}
			if ( B.empty() ) {
				return true;
			}

			ssize_t j = B_prime.size() - 1;

			for ( ssize_t i=B.size()-1; i>=0; i-- ) {
				
				// compare last bit
				if ( B.at(i) != B_prime.at(j) ) {
					return false;
				}
				
				// move up
				j--;
			}
			
			// if we make it here, B is a suffix of B_prime
			return true;
		}
		
		void Merge( std::vector<Interval*>& V, std::vector<PVT_S*>& S, Constraints& c ) {

			bool modified = true;
			
			while ( modified ) {
			
				modified = false;
				
				for ( size_t i=0; i<S.size(); i++ ) {
				
					for ( size_t j=0; j<S.size(); j++ ) {
						
						// don't merge with self
						if ( i == j ) {
							continue;
						}
						
						// test for suffix
						if ( isSuffix(*S.at(i)->B, *S.at(j)->B) ) {
							
							// get interval information
							double min_i = S.at(i)->V->getMin();
							double max_i = S.at(i)->V->getMax();
							double min_j = S.at(j)->V->getMin();
							double max_j = S.at(j)->V->getMax();
							
							// modify j
							S.at(j)->V->setBounds( std::min(min_i, min_j), std::max(max_i, max_j) );
							
							// mark as modified
							modified = true;
							break;							
						}
					}
					
					// element i was merged, break the loop and start over
					if ( modified ) {
						delete( S.at(i) );
						S.erase( S.begin() + i );
						break;
					}
				}
			}
			
			// gather final velocity intervals together
			std::vector<Interval*> V_tmp;
			for ( size_t i=0; i<S.size(); i++ ) {
				
				if ( !S.at(i)->V->isEmpty() ) {
					V_tmp.push_back( S.at(i)->V );
				}
			}
			
			// clear previous velocity intervals
			for ( size_t i=0; i<V.size(); i++ ) {
				delete( V.at(i) );
			}
			V.clear();
			
			// perform union of intervals
			Interval::specialUnionOfIntervals( V, V_tmp );
		}
		
		void MergeGoal( std::vector<PVT_S*>& Goal, std::vector<PVT_S*>& S, Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			// merge information for each channel class
			bool modified = true;
			while ( modified ) {
				modified = false;
				
				for ( size_t i=0; i<S.size(); i++ ) {
					for ( size_t j=0; j<S.size(); j++ ) {
						
						// don't self merge
						if ( j == i ) {
							continue;
						}
						
						// is element i a suffix of j?
						if ( isSuffix(*S.at(i)->B, *S.at(j)->B) ) {
							
							double min_i = S.at(i)->UB->back()->getFinalState().getTimeCoord();
							double max_i = S.at(i)->LB->back()->getFinalState().getTimeCoord();
							double min_j = S.at(j)->UB->back()->getFinalState().getTimeCoord();
							double max_j = S.at(j)->LB->back()->getFinalState().getTimeCoord();
							
							// merge lower time bounding trajectory
							if ( Maths::approxLt(min_i, min_j, epsilon) ) {
								S.at(j)->setUB( *S.at(i)->UB );
							}
							
							// merge upper time bounding trajectory
							if ( Maths::approxGt(max_i, max_j, epsilon) ) {
								S.at(j)->setLB( *S.at(i)->LB );
							}
							
							// mark as modified
							modified = true;
							break;
						}
					}
					
					// element i was merged, break the loop and start over
					if ( modified ) {
						delete( S.at(i) );
						S.erase( S.begin() + i );
						break;
					}
				}
			}
			
			// add to Goal
			for ( size_t i=0; i<S.size(); i++ ) {
				Goal.push_back( new PVT_S( *S.at(i) ) );
			}
		}
		
		bool NextReachableSet( std::pair< std::vector<TrajectorySegment*> *, std::vector<TrajectorySegment*> * >& Reachable,
							  PVT_Point& p1,
							  PVT_Point& p2,
							  Interval& V_i,
							  Constraints& c ) {
			
			// enforce monotonicity
			if ( !p1.canSee(p2, c) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Not reachable: Monotonicity violation." << std::endl;
#endif
				return true;
			}
			
			Interval i_V;
			if ( !Planner::FindInitialVelocityRange( i_V, p1, p2, c ) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::NextReachableSet failed to find initial velocities." << std::endl;
#endif
				return true;
			}
			Interval V_tmp( i_V );
			
			i_V.intersect( V_i, c );
			if ( i_V.isEmpty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::NextReachableSet feasible set of initial velocities is empty." << std::endl;
				std::cout << V_i << ": " << p1 << " -> " << p2 << std::endl;
#endif
				return true;
			}
			
			PVT_State s_i_min( p1.getPathCoord(), p1.getTimeCoord(), i_V.getMin() );
			PVT_State s_i_max( p1.getPathCoord(), p1.getTimeCoord(), i_V.getMax() );
			
			std::vector<TrajectorySegment*> * UB_min = new std::vector<TrajectorySegment*>();
			std::vector<TrajectorySegment*> * UB_max = new std::vector<TrajectorySegment*>();
			std::vector<TrajectorySegment*> * LB_min = new std::vector<TrajectorySegment*>();
			std::vector<TrajectorySegment*> * LB_max = new std::vector<TrajectorySegment*>();
			if ( !UpperBoundingStates(*UB_min, s_i_min, p2, c) ) {
				std::cerr << "ERROR IN Planner::NextReachableSet: Failure in Planner::UpperBoundingStates (1)" << std::endl;
				return false;
			}
			if ( !UpperBoundingStates(*UB_max, s_i_max, p2, c) ) {
				std::cerr << "ERROR IN Planner::NextReachableSet: Failure in Planner::UpperBoundingStates (2)" << std::endl;
				return false;
			}
			if ( !LowerBoundingStates(*LB_min, s_i_min, p2, c) ) {
				std::cerr << "ERROR IN Planner::NextReachableSet: Failure in Planner::LowerBoundingStates (1)" << std::endl;
				return false;
			}
			if ( !LowerBoundingStates(*LB_max, s_i_max, p2, c) ) {
				std::cerr << "ERROR IN Planner::NextReachableSet: Failure in Planner::LowerBoundingStates (2)" << std::endl;
				return false;
			}
			
			// this test should never fail
			if ( UB_min->empty() || UB_max->empty() || LB_min->empty() || LB_max->empty() ) {
				std::cerr << "ERROR IN Planner::NextReachableSet: Invalid range." << std::endl;
				std::cerr << s_i_min << " " << s_i_max << " " << p2 << std::endl;
				std::cerr << "UB_min size: " << UB_min->size() << " UB_max size: " << UB_max->size() << std::endl;
				std::cerr << "LB_min size: " << LB_min->size() << " LB_max size: " << LB_max->size() << std::endl;
				return false;
			}
			
			// figure correct bounds
			double UB_min_v2 = UB_min->back()->getFinalState().getVelocityCoord();
			double UB_max_v2 = UB_max->back()->getFinalState().getVelocityCoord();
			if ( UB_min_v2 > UB_max_v2 ) {
				Reachable.first = UB_min;
				Utilities::CleanTrajectory( *UB_max );
				delete( UB_max );
			} else {
				Reachable.first = UB_max;
				Utilities::CleanTrajectory( *UB_min );
				delete( UB_min );
			}
			
			double LB_min_v2 = LB_min->back()->getFinalState().getVelocityCoord();
			double LB_max_v2 = LB_max->back()->getFinalState().getVelocityCoord();
			if ( LB_max_v2 < LB_min_v2 ) {
				Reachable.second = LB_max;
				Utilities::CleanTrajectory( *LB_min );
				delete( LB_min );
			} else {
				Reachable.second = LB_min;
				Utilities::CleanTrajectory( *LB_max );
				delete( LB_max );
			}
			
			return true;
		}
		
		bool FindInitialVelocityRange( Interval& V_i, PVT_Point& p1, PVT_Point& p2, Constraints& c ) {
			
			if ( p1.equals( p2, c ) ) {
				return true;
			}
			
			if ( !p1.canSee( p2, c ) ) {
				return true;
			}
			
			if ( !c.validT(p2.getTimeCoord()) || !c.validX(p2.getPathCoord()) ) {
				return true;
			}
			
			
			double int_max, int_min;
			
			double x1 = p1.getPathCoord();
			double t1 = p1.getTimeCoord();
			double x2 = p2.getPathCoord();
			double t2 = p2.getTimeCoord();
			
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			
			if ( delta_x < 0 ) {
				delta_x = Maths::clipToZero( delta_x, Constants::CLIP_TO_ZERO );
			}
			if ( delta_t < 0 ) {
				delta_t = Maths::clipToZero( delta_t, Constants::CLIP_TO_ZERO );
			}
			
			// Set up constraints for this problem
			Constraints c_point( c );
			c_point.setXLimit( delta_x );
			c_point.setTLimit( delta_t );
			
			// The bounds
			double v2_min = (c_point.getVMin() > p2.i->getMin()) ? c_point.getVMin() : p2.i->getMin();
			double v2_max = (c_point.getVMax() < p2.i->getMax()) ? c_point.getVMax() : p2.i->getMax();
			double v1_min = (c_point.getVMin() > p1.i->getMin()) ? c_point.getVMin() : p1.i->getMin();
			double v1_max = (c_point.getVMax() < p1.i->getMax()) ? c_point.getVMax() : p1.i->getMax();
			double a_min = c_point.getAMin();
			double a_max = c_point.getAMax();
			double epsilon = c_point.getEpsilon();
			
			double v_avg = Maths::avgVelFromX1_X2_T1_T2( x1, x2, t1, t2, epsilon );
			if ( Maths::isNaN(v_avg) ) {
				std::cerr << "Return 0." << std::endl;
				return false;
			}
			
			//
			// Quick check: If average velocity is outside bounds, p2 is unreachable
			//
			if ( !c_point.validV(v_avg) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::FindInitialVelocityRange: p2 unreachable, average velocity " << v_avg << " invalid." << std::endl;
#endif
				return true;
			}
			
			//
			// First determine the maximum possible incoming velocity
			//
			
			// Find path coordinate where parabola mins assuming v1 = v_max
			double v1 = v1_max;
			double v2 = v2_min;
			double a1 = a_min;
			
			// interested in the absolute value of x, so use -a1
			double x = Maths::parabolaDelX_FromV1_V2_A( v1, v2, -a1, epsilon );
			if ( Maths::isNaN(x) ) {
				std::cerr << "Return 1." << std::endl;
				return false;
			}
			double t_test = Maths::motionT_FromV1_X1_X2_A( v1, x1, x2, a1, epsilon );
			
			// If the min point is beyond the end point, adjust the parabola
			// backward by decreasing the initial velocity
			if ( Maths::approxGt(x, delta_x, epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Max, case 1." << std::endl;
#endif
				
				// Try to find an initial that passes through end point
				v1 = Maths::motionV1_FromX1_X2_T1_T2_A( 0., delta_x, 0., delta_t, a1, epsilon );
				double v_f = Maths::V2_FromV1_T_A( v1, delta_t, a1, epsilon );
				if ( !c_point.validV(v1) || !c_point.validV(v_f) ) {

					// The initial velocity that mins at x2
					v1 = Maths::parabolaV1_FromV2_A( v2, delta_x, a1, epsilon );
					if ( Maths::isNaN(v1) ) {
						std::cerr << "Planner::FindInitialVelocityRange: Invalid initial velocity calculated: v2 " << v2 << ", delta_x " << delta_x << ", a1 " << a1 << ", p1 " << p1 << ", p2 " << p2 << std::endl;
						return false;
					}

					// Make sure this satisfies the time constraint; it will always
					// satisfy the velocity constraint because it goes from v_max to v_min
					t_test = Maths::motionT_FromV1_X1_X2_A( v1, x1, x2, a1, epsilon );
					
					if ( !Maths::isNaN(t_test) && !c_point.validT(t_test) ) {
						
						
						
						// If we get here, there is no dynamically feasible initial velocity
						// that can reach the endpoint via constant deceleration; try introducing
						// an initial linear segment
						double a = a1;
						double b = -2 * a1 * delta_t;
						double c = a1 * delta_t * delta_t + 2 * v1_max * delta_t - 2 * delta_x;
						std::pair<double, double> roots;
						Maths::quadratic( roots, a, b, c, epsilon );
						double t1 = roots.first;
						if ( Maths::isNaN(t1) || !c_point.validT(t1) ) {
#ifdef SHOW_COMMENTS
							std::cout << "Return 3." << std::endl;
#endif
							return true;
						}
						v1 = v1_max;
					}
				}
				
				// At this point, v1 is the maximum initial velocity
				int_max = v1;
				
			// The min point was at or before the end point
			} else {
#ifdef SHOW_COMMENTS
				std::cout << "Max, case 2." << std::endl;
#endif
				// use the acceleration that will yeild a shorter line segment
				if ( Maths::approxGt(v1, v_avg, epsilon) ) {
					a1 = a_min;
				} else {
					a1 = a_max;
				}
				
				// If it mins before the endpoint, ensure that velocity constraints
				// are respected by finding the tangent line connecting the
				// parabola with the endpoint and verifying its slope
				double t_tangent = Maths::parabolaT_TangentFromV1_X2_T2_A( v1, delta_x, delta_t, a1, epsilon );
				
				// sanity check: time coordinate of tangent point valid
				if ( !c_point.validT(t_tangent) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Return 4." << std::endl;
#endif
					return true;
				}
				
				// The tangent point
				double v_tangent = Maths::V2_FromV1_T_A( v1, t_tangent, a1, epsilon );
				
				// velocity check
				if ( !c_point.validV(v_tangent) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Return 5." << std::endl;
#endif
					return true;
				}
				
				double x_tangent = v_tangent * t_tangent + delta_x - v_tangent * delta_t;
				
				// path check
				if ( !c_point.validX(x_tangent) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Return 6: x_tangent: " << x_tangent << std::endl;
#endif
					return true;
				}
				
				// At this point, v1 is the maximum initial velocity
				int_max = v1;
			}
			
			//
			// Now determine the minimum possible incoming velocity
			//
			
			a1 = a_max;
			v1 = v1_min;
			t_test = Maths::motionT_FromV1_X1_X2_A( v1, x1, x2, a1, epsilon );
			if ( !c_point.validT(t_test) ) {			
#ifdef SHOW_COMMENTS
				std::cout << "Min, case 1." << std::endl;
#endif
				// If the arrival time is greater than the given time,
				// adjust v1 such that t1 = t2
				v1 = Maths::motionV1_FromX1_X2_T1_T2_A( x1, x2, t1, t2, a1, epsilon );
				
				if ( !c_point.validV(v1) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Return 7." << std::endl;
#endif
					return true;
				}
				
				// check arrival velocity
				v2 = Maths::V2_FromV1_T_A( v1, t_test, a1, epsilon );
				if ( !(Maths::approxGe(v2, v2_min, epsilon) && Maths::approxLe(v2, v2_max, epsilon)) ) {
					
					// if it's not valid, drop a linear segment at v_max from
					// the destination, find the parabola tangent to it
					double x_int = delta_x - v2_max * delta_t;
					std::pair<double, double> roots;
					Maths::quadratic( roots, a1, -2 * a1 * x_int, a1 * x_int * x_int + (2 * x_int * v2_max * v2_max), epsilon );
					double t_tangent = (roots.first - x_int) / v2_max;
					v1 = v2_max - a1 * t_tangent;
					
					if ( !c_point.validV(v1) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Return 8." << std::endl;
#endif
						return true;
					}
				}
					
				// If we make it here, everything checks out
				int_min = v1;
					
			// The arrival time is valid
			} else {
#ifdef SHOW_COMMENTS
				std::cout << "Min, case 2." << std::endl;
#endif
				// verify the tangent line connecting the parabola to the
				// endpoint
				double t_tangent = Maths::parabolaT_TangentFromV1_X2_T2_A( v1, delta_x, delta_t, a1, epsilon );
				
				if ( !c_point.validT(t_tangent) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Return 9: t_tangent = " << t_tangent << std::endl;
#endif
					return true;
				}
				
				double v_tangent = Maths::V2_FromV1_T_A( v1, t_tangent, a1, epsilon );
				
				if ( !c_point.validV(v_tangent) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Return 10: v_tangent = " << v_tangent << " v1 = " << v1 << " t_tangent = " << t_tangent << " a1 = " << a1 << std::endl;
#endif
					return true;
				}
				
				double x_tangent = v_tangent * t_tangent + delta_x - v_tangent * delta_t;

				if ( !c_point.validX(x_tangent) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Return 11." << std::endl;
#endif
					return true;
				}
				
				// everything checks out
				int_min = v1;
			}

			V_i.init( int_min, int_max );
#ifdef SHOW_COMMENTS
			std::cout << "Return 12." << std::endl;
#endif

			return true;
		}
		
		bool UpperBoundingStates( std::vector<TrajectorySegment*>& T,
								 PVT_State& s_i,
								 PVT_Point& p,
								 Constraints& c ) {
			
			double x1 = s_i.getPathCoord();
			double t1 = s_i.getTimeCoord();
			double v1 = s_i.getVelocityCoord();
			
			double x2 = p.getPathCoord();
			double t2 = p.getTimeCoord();
			
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			
			// Set up constraints for this problem
			Constraints c_point( c );
			c_point.setXLimit( delta_x );
			c_point.setTLimit( delta_t );
			
			// The bounds
			double v2_min = (c_point.getVMin() > p.i->getMin()) ? c_point.getVMin() : p.i->getMin();
			double v2_max = (c_point.getVMax() < p.i->getMax()) ? c_point.getVMax() : p.i->getMax();
			double v_min = c_point.getVMin();
			double a_min = c_point.getAMin();
			double a_max = c_point.getAMax();
			double epsilon = c_point.getEpsilon();
			
			// Is it even possible to reach p?
			if ( !s_i.canReach(p, c_point) ) {
#ifdef SHOW_COMMENTS
				std::cout << "UB: Unreachable." << std::endl;
#endif
				return true;
			}
			
			//
			// CASE 1: Initial and final curves are separate or tangent, final
			// velocity of v_max. Find tangency.
			//
			PVT_State s2( p.getPathCoord(), p.getTimeCoord(), v2_max );
			if ( !BuildPLP(T, s_i, s2, c) ) {
				std::cerr << "ERROR IN Planner::LowerBoundingStates: BuildPLP failed." << std::endl;
				return false;
			}
			if ( !T.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "UB: v_max reachable." << std::endl;
#endif
				return true;
			}

			//
			// CASE 2: Final velocity less than maximum velocity with v1 = v_min
			//
			
			double a1 = a_min;
			double a2 = a_max;
			double v_tangent = v_min;
			
			std::pair< std::pair<double, double>, std::pair<double, double> > coords;
			Maths::parabolasTangentLine( coords, v1, v_tangent, a1, a2, delta_x, delta_t, epsilon );
			double x1_tangent = coords.first.first;
			double t1_tangent = coords.first.second;
			double x2_tangent = coords.second.first;
			double t2_tangent = coords.second.second;
			if ( !(Maths::isNaN(x1_tangent)
				   || Maths::isNaN(t1_tangent)
				   || Maths::isNaN(x2_tangent)
				   || Maths::isNaN(t2_tangent)
				   || !c.validV(Maths::slope(coords.first, coords.second, epsilon))) ) {
				
				if ( c_point.validX(x1_tangent)
					&& c_point.validX(x2_tangent)
					&& c_point.validT(t1_tangent)
					&& c_point.validT(t2_tangent)
					&& Maths::approxGe( t2_tangent, t1_tangent, epsilon) ) {
					
					double v2 = Maths::V2_FromV1_T_A( v_tangent, (delta_t - t2_tangent), a2, epsilon );
					if ( c_point.validV(v2) ) {
#ifdef SHOW_COMMENTS
						std::cout << "UB: 2a." << std::endl;
#endif
						
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x1_tangent, t1 + t1_tangent, v_tangent );
						PVT_State s3( x1 + x2_tangent, t1 + t2_tangent, v_tangent );
						PVT_State s4( x2, t2, v2 );
						
						T.push_back( new TrajectorySegment(s1, s2) );
						T.push_back( new TrajectorySegment(s2, s3) );
						T.push_back( new TrajectorySegment(s3, s4) );
						return true;
						
					}
					
				}
				
			}
			
			// otherwise adjust tangent velocity
			double qa = a1 - a2;
			double qb = 2 * delta_t * (a2 - a1);
			double qc = 2 * delta_x - a2 * delta_t * delta_t - 2 * v1 * delta_t;
			std::pair<double, double> roots;
			Maths::quadraticOrdered( roots, qa, qb, qc, epsilon );
			if ( Maths::isNaN(roots.first) && Maths::isNaN(roots.second) ) {
				std::cerr << "ERROR IN Planner::UpperBoundingStates: no solution to quadratic (2)" << std::endl;
				return false;
			}
			double t_tangent = roots.first;
			
			if ( c_point.validT(t_tangent) ) {

				double x_tangent = Maths::motionX_FromV1_T1_T2_A( v1, 0., t_tangent, a1, epsilon );
				double v_tangent = Maths::V2_FromV1_T_A( v1, t_tangent, a1, epsilon );
				double t_star = ((a2 - a1) * t_tangent - v1 ) / a2;
				double v2 = Maths::V2_FromV1_T_A( 0., delta_t - t_star, a2, epsilon );
				if ( Maths::approxGe(v2, v2_min, epsilon) && Maths::approxLe(v2, v2_max, epsilon)
					&& Maths::approxGe(v_tangent, v2_min, epsilon) && Maths::approxLe(v_tangent, v2_max, epsilon) ) {
					PVT_State s1( s_i );
					PVT_State s2( x1 + x_tangent, t1 + t_tangent, v_tangent );
					PVT_State s3( x2, t2, v2 );
					if ( !s1.equals(s2, c) ) {
						T.push_back( new TrajectorySegment(s1, s2) );
					}
					if ( !s2.equals(s3, c) ) {
						T.push_back( new TrajectorySegment(s2, s3) );
					}
#ifdef SHOW_COMMENTS
					std::cout << "UB: Case 2b." << std::endl;
#endif
					return true;
				}
				
			}
			
			std::cerr << "ERROR IN Planner::UpperBoundingStates: Unexpected upper-bound case. Re-examine algorithm; this is probably caused by numeric instability. Reconstruct the error condition with the following information:" << std::endl;
			std::cerr << "Start state: " << s_i << ", goal point: " << p << std::endl;
			std::cerr << "Constraints: " << c << std::endl;
			return true;
		}
		
		
		bool LowerBoundingStates( std::vector<TrajectorySegment*>& T,
								 PVT_State& s_i,
								 PVT_Point& p,
								 Constraints& c ) {
			double x1 = s_i.getPathCoord();
			double t1 = s_i.getTimeCoord();
			double v1 = s_i.getVelocityCoord();
			
			double x2 = p.getPathCoord();
			double t2 = p.getTimeCoord();
			
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			
			// Set up constraints for this problem
			Constraints c_point( c );
			c_point.setXLimit( delta_x );
			c_point.setTLimit( delta_t );
			
			// The bounds
			double v2_min = (c_point.getVMin() > p.i->getMin()) ? c_point.getVMin() : p.i->getMin();
			double v2_max = (c_point.getVMax() < p.i->getMax()) ? c_point.getVMax() : p.i->getMax();
			double v_min = c_point.getVMin();
			double v_max = c_point.getVMax();
			double a_min = c_point.getAMin();
			double a_max = c_point.getAMax();
			double epsilon = c_point.getEpsilon();
			
			// Is it even possible to reach p?
			if ( !s_i.canReach(p, c_point) ) {
#ifdef SHOW_COMMENTS
				std::cout << "LB: Unreachable." << std::endl;
#endif
				return true;
			}
			
			//
			// CASE 1: Acc. and dec. curves are separate or tangent, final velocity
			// of v_min. Find tangency.
			//
			
			PVT_State s2( p.getPathCoord(), p.getTimeCoord(), v2_min );
			if ( !BuildPLP(T, s_i, s2, c) ) {
				std::cerr << "ERROR IN Planner::LowerBoundingStates: BuildPLP failed." << std::endl;
				return false;
			}
			if ( !T.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "LB: v_min reachable." << std::endl;
#endif
				return true;
			}
			
			double a1, a2; //, v2;
			
			//
			// CASE 2: Final velocity greater than minimum velocity
			//
			
			a1 = a_max;
			a2 = a_min;
			double v_tangent = v_max;
			
			std::pair< std::pair<double, double>, std::pair<double, double> > coords;
			Maths::parabolasTangentLine( coords, v1, v_tangent, a1, a2, delta_x, delta_t, epsilon );
			double x1_tangent = coords.first.first;
			double t1_tangent = coords.first.second;
			double x2_tangent = coords.second.first;
			double t2_tangent = coords.second.second;
			if ( !(Maths::isNaN(x1_tangent) || Maths::isNaN(t1_tangent) || Maths::isNaN(x2_tangent) || Maths::isNaN(t2_tangent) || 
				   !c.validV(Maths::slope(coords.first, coords.second, epsilon))) ) {
				
				if ( c_point.validX(x1_tangent)
					&& c_point.validX(x2_tangent)
					&& c_point.validT(t1_tangent)
					&& c_point.validT(t2_tangent)
					&& Maths::approxGe(t2_tangent, t1_tangent, epsilon) ) {
					
					double v2 = Maths::V2_FromV1_T_A( v_tangent, (delta_t - t2_tangent), a2, epsilon );
					
					if ( c_point.validV(v2) ) {
#ifdef SHOW_COMMENTS
						std::cout << "LB: 2a." << std::endl;
#endif
						
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x1_tangent, t1 + t1_tangent, v_tangent );
						PVT_State s3( x1 + x2_tangent, t1 + t2_tangent, v_tangent );
						PVT_State s4( x2, t2, v2 );
						
						T.push_back( new TrajectorySegment(s1, s2) );
						T.push_back( new TrajectorySegment(s2, s3) );
						T.push_back( new TrajectorySegment(s3, s4) );
						return true;
						
					}
					
				}
				
			}
			
			// otherwise adjust tangent velocity
			double q_a = a1 - a2;
			double q_b = 2 * delta_t * (a2 - a1);
			double q_c = 2 * delta_x - a2 * delta_t * delta_t - 2 * v1 * delta_t;
			std::pair<double, double> roots;
			Maths::quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			if ( Maths::isNaN(roots.first) && Maths::isNaN(roots.second) ) {
				std::cerr << "ERROR IN Planner::LowerBoundingStates: no solution to quadratic (2)" << std::endl;
				return false;
			}
			double t_tangent = roots.second;

			if ( c_point.validT(t_tangent) ) {
				double x_tangent = Maths::motionX_FromV1_T1_T2_A( v1, 0., t_tangent, a1, epsilon );
				double v_tangent = Maths::V2_FromV1_T_A( v1, t_tangent, a1, epsilon );
				double t_star = ((a2 - a1) * t_tangent - v1 ) / a2;
				double v2 = Maths::V2_FromV1_T_A( 0., delta_t - t_star, a2, epsilon );
				if ( Maths::approxGe(v2, v2_min, epsilon) && Maths::approxLe(v2, v2_max, epsilon)
					&& Maths::approxGe(v_tangent, v2_min, epsilon) && Maths::approxLe(v_tangent, v2_max, epsilon) ) {
					PVT_State s1( s_i );
					PVT_State s2( x1 + x_tangent, t1 + t_tangent, v_tangent );
					PVT_State s3( x2, t2, v2 );
					if ( !s1.equals(s2, c) ) {
						T.push_back( new TrajectorySegment(s1, s2) );
					}
					if ( !s2.equals(s3, c) ) {
						T.push_back( new TrajectorySegment(s2, s3) );
					}
#ifdef SHOW_COMMENTS
					std::cout << "LB: Case 2b." << std::endl;
#endif
					return true;
				}
				
			}
			
			std::cerr << "ERROR IN Planner::LowerBoundingStates: Unexpected lower-bound case. Re-examine algorithm; this is probably caused by numeric instability. Reconstruct the error condition with the following information:" << std::endl;
			std::cerr << "Start state: " << s_i << ", goal point: " << p << std::endl;
			std::cerr << "Constraints: " << c << std::endl;			
			return true;
		}
		
		bool GoalConnect( std::vector<TrajectorySegment*>& UB,
						 std::vector<TrajectorySegment*>& LB,
						 PVT_Point& p1,
						 Interval& V_i,
						 Interval& V_f,
						 PVT_ObstacleSet& O,
						 Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			double x_limit = c.getXLimit();
			double t_limit = c.getTLimit();
			
			double v_g_min = V_f.getMin();
			double v_g_max = V_f.getMax();
			double v_max = c.getVMax();
			double a_min = c.getAMin();
			double a_max = c.getAMax();
			
			double v_f_min = v_g_min;
			double v_f_max = v_g_max;
			
			//
			// From the bound of V_i, calculate the fastest and slowest we can
			// arrive at x_limit while respecting dynamic constraints
			//
			
			double v1_min = V_i.getMin();
			double v1_max = V_i.getMax();
			
			double p_x = p1.getPathCoord();
			double p_t = p1.getTimeCoord();
			
			double delta_x = x_limit - p_x;
			
			// Set up constraints for this problem
			Constraints c_point( c );
			c_point.setXLimit( delta_x );
			c_point.setTLimit( t_limit - p_t );
			
			//
			// PART 1: First find absolute velocity limits at goal, then
			// intersect with V_f, then compute bounding trajectories from there
			//
			
			// max arrival velocity
			double delta_t = Maths::motionT_FromV1_X1_X2_A( v1_max, p_x, x_limit, a_max, epsilon );
			if ( c_point.validT(delta_t) ) {
				
				// max arrival velocity
				v_f_max = Maths::V2_FromV1_T_A( v1_max, delta_t, a_max, epsilon );
				if ( !c_point.validV(v_f_max) ) {

					// point at which velocity maxes
					double t_i = Maths::T_FromV1_V2_A( v1_max, v_max, a_max, epsilon );
					double x_i = Maths::motionX_FromV1_T1_T2_A( v1_max, 0., t_i, a_max, epsilon );
					
					// linear segment to goal
					double t_f = Maths::T_FromV1_V2_X1_X2( v_max, v_max, 0., delta_x - x_i, epsilon );
					
					// total time
					double delta_t = t_i + t_f;
					
					// if this exceeds bounds, the goal is unreachable
					if ( !c_point.validT(delta_t) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Planner::GoalConnect: Goal unreachable, case 1." << std::endl;
#endif
						return true;
					}
					
					// Goal is reachabe at max velocity
					v_f_max = v_max;
				}
			}
			
			// min arrival velocity
			std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > Reachable;
			Reachable.first = NULL;
			Reachable.second = NULL;
			PVT_Point p2( x_limit, t_limit );
			if ( !NextReachableSet(Reachable, p1, p2, V_i, c) ) {
				std::cerr << "ERROR IN API:GoalConnect: NextReachableSet failed" << std::endl;
				return false;
			}
			
			// If the time limit cannot be reached with v_min, then the trajectory
			// must hit the x_limit below the t_limit with some v > v_min; find
			// that time
			if ( (Reachable.first == NULL) || (Reachable.second == NULL) ) {
				
				// try decelerating from v1_min first
				double v1 = v1_min;
				double t_test = Maths::motionT_FromV1_X1_X2_A( v1, 0., delta_x, a_min, epsilon );
				if ( !c_point.validT(t_test) ) {
					v1 = v1_max;
					t_test = Maths::motionT_FromV1_X1_X2_A( v1, 0., delta_x, a_min, epsilon );
					if ( !c_point.validT(t_test) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Planner::GoalConnect: Goal unreachable, case 2." << std::endl;
#endif
						return true;
					}
				}
				
				// calculate arrival velocity
				v_f_min = Maths::V2_FromV1_T_A( v1, t_test, a_min, epsilon );
				
			} else {
				
				// make sure minium velocity is feasible
				v1_min = std::min( Reachable.first->front()->getInitialState().getVelocityCoord(), Reachable.second->front()->getInitialState().getVelocityCoord() );
				
				// min arrival velocity
				v_f_min = Reachable.second->back()->getFinalState().getVelocityCoord();
			}
			
			// velocity range at goal that is dynamically feasible
			Interval V_g( v_f_min, v_f_max );
			
			V_g.intersect( V_f, c );
			if ( V_g.isEmpty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::GoalConnect: Goal unreachable, case 3." << std::endl;
#endif
				return true;
			}
			
			// now with valid bounds on arrival velocity, compute corresponding trajectories
			v_f_min = V_g.getMin();
			v_f_max = V_g.getMax();
			
			//
			// PART 2: Build representative trajectories that arrive at the goal
			// velocity bounds
			//
			
			// find crossover point with v1 = v1_min, a1 = a_max, a2 = a_min
			double v1 = v1_min;
			double vf = v_f_max;
			
			if ( !BuildGoalPLP(UB, p1, V_i, vf, c) ) {
				std::cerr << "ERROR IN Planner::GoalConnect: BuildGoalPLP failed (1)" << std::endl;
				return false;
			}
			if ( UB.empty() ) {
				
				// if there is no feasible trajectory, try with other initial velocity
				v1 = v1_max;
				if ( !BuildGoalPLP(UB, p1, V_i, vf, c) ) {
					std::cerr << "ERROR IN Planner::GoalConnect: BuildGoalPLP failed (2)" << std::endl;
					return false;
				}
				
				// at this point, the bounding initial velocities don't admit feasible
				// trajectories, so just connect to the limit point with NRS2
				if ( UB.empty() ) {
					
					if ( false && Reachable.first != NULL ) {
						UB.swap( *Reachable.first );
					} else {
						std::cerr << "ERROR IN Planner::GoalConnect: Failed to find UB" << std::endl;
						std::cerr << "V_i: " << V_i << ", V_f: " << V_f << std::endl;
						std::cerr << "p1: " << p1 << ", v1: " << v1 << ", vf: " << vf << " previous v1: " << v1_max << std::endl;
						std::cerr << "x_limit: " << c.getXLimit() << ", t_limit: " << c.getTLimit() << std::endl;
						//std::cerr << O << std::endl;
						return false;
					}
				}
				
			}
			
			// build trajectory with min arrival velocity
			v1 = v1_max;
			vf = v_f_min;
			double x2 = delta_x;
			
			if ( !BuildGoalPLP(LB, p1, V_i, vf, c) ) {
				std::cerr << "ERROR IN Planner::GoalConnect: BuildGoalPLP failed (3)" << std::endl;
				return false;
			}
			if ( LB.empty() ) {
				
				// if there is no feasible trajectory, try with other initial velocity
				v1 = v1_min;
				if ( !BuildGoalPLP(LB, p1, V_i, vf, c) ) {
					std::cerr << "ERROR IN Planner::GoalConnect: BuildGoalPLP failed (4)" << std::endl;
					return false;
				}
				
				// at this point, the bounding initial velocities don't admit feasible
				// trajectories, so just connect to the limit point with NRS2
				if ( LB.empty() ) {
					
					if ( false && Reachable.second != NULL ) {
						LB.swap( *Reachable.second );
					} else {
						std::cerr << "ERROR IN Planner::GoalConnect: Failed to find LB" << std::endl;
						std::cerr << "V_i: " << V_i << ", V_f: " << V_f << std::endl;
						std::cerr << "p1: " << p1 << ", v1: " << v1 << ", vf: " << vf << " previous v1: " << v1_max << std::endl;
						std::cerr << "x_limit: " << c.getXLimit() << ", t_limit: " << c.getTLimit() << std::endl;
						//std::cerr << O << std::endl;
						return false;
					}
					
				}
			}
			
			// free memory used by trajectories
			if ( Reachable.first != NULL ) {
				Utilities::CleanTrajectory( *Reachable.first );
				delete( Reachable.first );
			}
			if ( Reachable.second != NULL ) {
				Utilities::CleanTrajectory( *Reachable.second );
				delete( Reachable.second );
			}
			
#ifdef SHOW_COMMENTS
			std::cout << "Propagate Goal: succeeded." << std::endl;
#endif
			return true;
		}
		
		bool BuildGoalPLP( std::vector<TrajectorySegment*>& T,
						  PVT_Point& p1,
						  Interval& V_i,
						  double v2,
						  Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			double a_max = c.getAMax();
			double a_min = c.getAMin();
			double v_max = c.getVMax();
			
			double x_limit = c.getXLimit();
			double p_x = p1.getPathCoord();
			double p_t = p1.getTimeCoord();
			
			double delta_x = x_limit - p_x;
			
			double v0_max = V_i.getMax();
			
			//
			// Compute bounding trajectories:
			// Assume initial acceleration is a_max, final is a_min. First compute
			// switching time by ignoring velocity constraints along trajectory. If
			// there exists a valid switching time, then check velocity constraint at
			// that time. If it fails, insert a linear segment.
			//
			
			double a1 = a_max;
			double a2 = a_min;
			double v0 = v0_max;
			double qa = a1 * a2 - a1 * a1;
			double qb = 2 * v0 * (a2 - a1);
			double qc = v2 * v2 - v0 * v0 - 2 * a2 * delta_x;
			std::pair<double, double> roots;
			Maths::quadraticOrdered( roots, qa, qb, qc, epsilon );
			double t1 = roots.second;
			if ( Maths::isNaN(t1) ) {
				std::cerr << "ERROR IN Planner::BuildGoalPLP: Failed computing switching time." << std::endl;
				return false;
			}
			
			// ensure the switch occurs at valid time; if not adjust initial
			// velocity such that it does
			if ( t1 < 0. ) {
				t1 = Maths::clipToZero( t1, Constants::CLIP_TO_ZERO );
				if ( t1 < 0. ) {
					t1 = 0.;
					double discriminant = v2 * v2 - 2 * a2 * delta_x;
					if ( discriminant < 0. ) {
						discriminant = Maths::clipToZero( discriminant, Constants::CLIP_TO_ZERO );
						if ( discriminant < 0. ) {
							std::cerr << "ERROR IN Planner::BuildGoalPLP: Unable to compute valid initial velocity." << std::endl;
							return false;
						}
					}
					v0 = sqrt( discriminant );
				}
			}
			
			// make sure this initial velocity is valid
			if ( !V_i.contains(v0, c) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildGoalPLP: Calculated initial velocity " << v0 << " invalid, goal unreachable." << std::endl;
#endif
				return true;
			}
			
			// calculate velocity switching time
			double v1 = v0 + a1 * t1;
			
			// if switching velocity is outside bounds, inser linear segment
			if ( !c.validV(v1) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildGoalPLP: Linear segment case." << std::endl;
#endif
				
				// build linear segment
				t1 = Maths::T_FromV1_V2_A( v0, v_max, a1, epsilon );
				double t2 = Maths::T_FromV1_V2_A( v_max, v2, a2, epsilon );
				double x2 = Maths::motionX_FromV1_T1_T2_A( v_max, 0., t2, a2, epsilon );
				double x1 = Maths::motionX_FromV1_T1_T2_A( v0, 0., t1, a1, epsilon );
				double delta_x_linear = delta_x - x1 - x2;
				double delta_t_linear = Maths::T_FromV1_V2_X1_X2( v_max, v_max, 0., delta_x_linear, epsilon );
				
				// make sure time is valid
				double total_time = p_t + t1 + delta_t_linear + t2;
				if ( !c.validT(total_time) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Planner::BuildGoalPLP: Linear segment case, time constraint violated. (1)" << std::endl;
#endif
					return true;
				}
				
				// if we make it here, we're done
				PVT_State s1( p_x, p_t, v0 );
				PVT_State s2( p_x + x1, p_t + t1, v_max );
				PVT_State s3( p_x + x1 + delta_x_linear, p_t + t1 + delta_t_linear, v_max );
				PVT_State s4( p_x + x1 + delta_t_linear + x2, total_time, v2 );
				T.push_back( new TrajectorySegment(s1, s2) );
				T.push_back( new TrajectorySegment(s2, s3) );
				T.push_back( new TrajectorySegment(s3, s4) );
				return true;
			}
			
#ifdef SHOW_COMMENTS
			std::cout << "Planner::BuildGoalPLP: No linear segment case." << std::endl;
#endif
			double t2 = Maths::T_FromV1_V2_A( v1, v2, a2, epsilon );
			double total_time = p_t + t1 + t2;
			if ( !c.validT(total_time) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildGoalPLP: Linear segment case, time constraint violated. (2)" << std::endl;
#endif
				return true;
			}
			
			double x1 = Maths::motionX_FromV1_T1_T2_A( v0, 0., t1, a1, epsilon );
			double x2 = Maths::motionX_FromV1_T1_T2_A( v1, 0., t2, a2, epsilon );
			
			PVT_State s1( p_x, p_t, v0 );
			PVT_State s2( p_x + x1, p_t + t1, v1 );
			PVT_State s3( p_x + x1 + x2, total_time, v2 );
			T.push_back( new TrajectorySegment(s1, s2) );
			T.push_back( new TrajectorySegment(s2, s3) );
			return true;
		}
		
		bool BuildPLP( std::vector<TrajectorySegment*>& T,
					  PVT_State& s1,
					  PVT_State& s2,
					  Constraints& c ) {
			double epsilon = c.getEpsilon();
			
			double a_min = c.getAMin();
			double a_max = c.getAMax();
			double v_min = c.getVMin();
			double v_max = c.getVMax();
			Interval v_feasible( v_min, v_max );
			
			double v1 = s1.getVelocityCoord();
			double v2 = s2.getVelocityCoord();
			
			// average velocity
			double x1 = s1.getPathCoord();
			double t1 = s1.getTimeCoord();
			double x2 = s2.getPathCoord();
			double t2 = s2.getTimeCoord();
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			double v_avg = Maths::avgVelFromX1_X2_T1_T2( x1, x2, t1, t2, epsilon );
			
			// Set up constraints for this problem
			Constraints c_point( c );
			c_point.setXLimit( delta_x );
			c_point.setTLimit( delta_t );
			
			// quick check of feasibility
			if ( !c_point.validV(v_avg) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildPLP: average velocity makes trajectory infeasible" << std::endl;
#endif
				return true;
			}
			
			// if initial/final/average velocity happen to be the same, we're done
			if ( Maths::approxEq(v1, v2, epsilon) && Maths::approxEq(v2, v_avg, epsilon) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildPLP: Case 1." << std::endl;
#endif
				PVT_State s1( x1, t1, v1 );
				PVT_State s2( x2, t2, v2 );
				T.push_back( new TrajectorySegment(s1, s2) );
				return true;
			}
			
			// if final velocity < average velocity, try final P curve - (minus)
			double a2, x_star, t_star;
			if ( Maths::approxLt(v2, v_avg, epsilon) ) {
			
				a2 = a_min;
			
			// otherwise, try + (plus)
			} else {
				
				a2 = a_max;
			}
			
			// origin of final P curve
			std::pair<double, double> origin;
			if ( !Maths::parabolaOriginOffset(origin, v2, a2, epsilon) ) {
				std::cerr << "ERROR IN Planner::BuildPLP: parabolaOriginOffset failed (1)" << std::endl;
				return false;
			}
			
			x_star = delta_x + origin.first;
			t_star = delta_t + origin.second;

			// Decide whether initial acceleration should be + or -
			// If the initial velocity line intersects the acceleration curve,
			// a1 should decelerate; otherwise accelerate
			double a1;
			std::pair<double, double> roots;
			Maths::quadraticOrdered( roots, a2, -2 * (a2 * t_star + v1), 2 * x_star + a2 * t_star * t_star, epsilon );
			if ( Maths::isNaN(roots.first) || Maths::isNaN(roots.second) ) {
				if ( Maths::approxLt(a2, 0., epsilon) ) {
					a1 = a_min;
				} else {
					a1 = a_max;
				}
			} else {
				if ( Maths::approxLt(a2, 0., epsilon) ) {
					a1 = a_max;
				} else {
					a1 = a_min;
				}
			}
			
			// try to find the tangent line between curves
			std::pair< std::pair<double, double>, std::pair<double, double> > coords;
			double v_tangent = Maths::parabolasTangentLine( coords, v1, a1, a2, x_star, t_star, epsilon );
			if ( !Maths::isNaN(coords.first.first) && c.validV(v_tangent) ) {
				double x1_tangent = coords.first.first;
				double t1_tangent = coords.first.second;
				double x2_tangent = coords.second.first;
				double t2_tangent = coords.second.second;
				if ( c_point.validX(x1_tangent) && c_point.validT(t1_tangent)
					&& c_point.validX(x2_tangent) && c_point.validT(t2_tangent) ) {

					if ( v_feasible.contains(v_tangent, c_point) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Planner::BuildPLP: Case 2a." << std::endl;
#endif
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x1_tangent, t1 + t1_tangent, v_tangent );
						PVT_State s3( x1 + x2_tangent, t1 + t2_tangent, v_tangent );
						PVT_State s4( x2, t2, v2 );
						T.push_back( new TrajectorySegment(s1, s2) );
						T.push_back( new TrajectorySegment(s2, s3) );
						T.push_back( new TrajectorySegment(s3, s4) );
						return true;
					}
				}
			}
				
			// if that fails, try without linear segment
			double q_a = a1 - a2;
			double q_b = 2 * (a2 * delta_t - a1 * delta_t);
			double q_c = 2 * delta_x - a2 * delta_t * delta_t - 2 * v1 * delta_t;
			Maths::quadratic( roots, q_a, q_b, q_c, epsilon );
			if ( !Maths::isNaN(roots.first) ) {
				double t_tangent = roots.first;
				if ( c_point.validT(t_tangent) ) {
					
					double x_tangent = Maths::motionX_FromV1_T1_T2_A( v1, 0., t_tangent, a1, epsilon );
					double v_tangent = Maths::V2_FromV1_T_A( v1, t_tangent, a1, epsilon );
					double v2_final = Maths::V2_FromV1_T_A( v1 + a1*t_tangent, delta_t - t_tangent, a2, epsilon );
					if ( Maths::approxEq(v2, v2_final, epsilon) ) {
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x_tangent, t1 + t_tangent, v_tangent );
						PVT_State s3( x2, t2, v2 );
						if ( !s1.equals(s2, c) ) {
							T.push_back( new TrajectorySegment(s1, s2) );
						}
						if ( !s2.equals(s3, c) ) {
							T.push_back( new TrajectorySegment(s2, s3) );
						}
#ifdef SHOW_COMMENTS
						std::cout << "BuildPLP: Case 2b." << std::endl;
#endif
						return true;
					}
				}
			}
			
			// if nothing was found, try swapping the final P curve sign
			if ( Maths::approxLt(a2, 0., epsilon) ) {
				a2 = a_max;
				
			} else {
				a2 = a_min;
			}
			
			// origin of final P curve
			if ( !Maths::parabolaOriginOffset(origin, v2, a2, epsilon) ) {
				std::cerr << "ERROR IN Planner::BuildPLP: parabolaOriginOffset failed (2)" << std::endl;
				return false;
			}
			
			x_star = delta_x + origin.first;
			t_star = delta_t + origin.second;

			// Decide whether initial acceleration should be + or -
			// If the initial velocity line intersects the acceleration curve,
			// a1 should decelerate; otherwise accelerate
			Maths::quadratic( roots, a2, -2 * (a2 * t_star + v1), 2 * x_star + a2 * t_star * t_star, epsilon );
			if ( Maths::isNaN(roots.first) || Maths::isNaN(roots.second) ) {
				if ( Maths::approxLt(a2, 0., epsilon) ) {
					a1 = a_min;
				} else {
					a1 = a_max;
				}
			} else {
				if ( Maths::approxLt(a2, 0., epsilon) ) {
					a1 = a_max;
				} else {
					a1 = a_min;
				}
			}
			
			// try to find the tangent line between curves
			v_tangent = Maths::parabolasTangentLine( coords, v1, a1, a2, x_star, t_star, epsilon );
			if ( !Maths::isNaN(coords.first.first) && c.validV(v_tangent) ) {
				double x1_tangent = coords.first.first;
				double t1_tangent = coords.first.second;
				double x2_tangent = coords.second.first;
				double t2_tangent = coords.second.second;
				if ( c_point.validX(x1_tangent) && c_point.validT(t1_tangent)
					&& c_point.validX(x2_tangent) && c_point.validT(t2_tangent) ) {
					if ( v_feasible.contains(v_tangent, c_point) ) {
#ifdef SHOW_COMMENTS
						std::cout << "Planner::BuildPLP: Case 3a." << std::endl;
#endif
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x1_tangent, t1 + t1_tangent, v_tangent );
						PVT_State s3( x1 + x2_tangent, t1 + t2_tangent, v_tangent );
						PVT_State s4( x2, t2, v2 );
						T.push_back( new TrajectorySegment(s1, s2) );
						T.push_back( new TrajectorySegment(s2, s3) );
						T.push_back( new TrajectorySegment(s3, s4) );
						return true;
					}
				}
			}
			// if that fails, try without linear segment
			q_a = a1 - a2;
			q_b = 2 * (a2 * delta_t - a1 * delta_t);
			q_c = 2 * delta_x - a2 * delta_t * delta_t - 2 * v1 * delta_t;
			Maths::quadratic( roots, q_a, q_b, q_c, epsilon );
			if ( !Maths::isNaN(roots.first) ) {
				double t_tangent = roots.first;
				if ( c_point.validT(t_tangent) ) {
					
					double x_tangent = Maths::motionX_FromV1_T1_T2_A( v1, 0., t_tangent, a1, epsilon );
					double v_tangent = Maths::V2_FromV1_T_A( v1, t_tangent, a1, epsilon );
					double v2_final = Maths::V2_FromV1_T_A( v1 + a1*t_tangent, delta_t - t_tangent, a2, epsilon );
					if ( Maths::approxEq(v2, v2_final, epsilon) ) {
						PVT_State s1( x1, t1, v1 );
						PVT_State s2( x1 + x_tangent, t1 + t_tangent, v_tangent );
						PVT_State s3( x2, t2, v2 );
						if ( !s1.equals(s2, c) ) {
							T.push_back( new TrajectorySegment(s1, s2) );
						}
						if ( !s2.equals(s3, c) ) {
							T.push_back( new TrajectorySegment(s2, s3) );
						}
#ifdef SHOW_COMMENTS
						std::cout << "BuildPLP: Case 3b." << std::endl;
#endif
						return true;
					}
				}
			}

			
#ifdef SHOW_COMMENTS
			std::cout << "Planner::BuildPLP: Unreachable" << std::endl;
#endif
			T.clear();
			return true;
		}
		
		bool BuildOptimalTrajectory( std::vector<TrajectorySegment*>& T,
									std::vector<PVT_G*>& G,
									std::vector<PVT_S*>& Goal,
									PVT_ObstacleSet& O,
									Constraints& c ) {
			T.clear();
			double epsilon = c.getEpsilon();
			
			if ( Goal.empty() ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildOptimalTrajectory: No goal-reachable intervals." << std::endl;
#endif
				return true;
			}
			
			// fastest goal connection, add to T
			size_t fastest_index = 0;
			double fastest_time = std::numeric_limits<double>::max();
			for ( size_t i=0; i<Goal.size(); i++ ) {
				double cur_time =  Goal.at(i)->UB->back()->getFinalState().getTimeCoord();
				if ( cur_time < fastest_time ) {
					fastest_time = cur_time;
					fastest_index = i;
				}
			}
			for ( ssize_t i=Goal.at(fastest_index)->UB->size()-1; i>=0; i-- ) {
				T.push_back( new TrajectorySegment(*Goal.at(fastest_index)->UB->at(i)) );
			}

			// if we're at the origin, we're done
			if ( T.back()->getInitialState().atPoint(0., 0., c) ) {
#ifdef SHOW_COMMENTS
				std::cout << "Planner::BuildOptimalTrajectory: At origin (1)." << std::endl;
#endif
				std::reverse( T.begin(), T.end() );
				return true;
			}

			// run backwards through velocity intervals
			for ( ssize_t i=G.size()-1; i>=0; i-- ) {
				
				// if we're at the origin, we're done
				if ( T.back()->getInitialState().atPoint(0., 0., c) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Planner::BuildOptimalTrajectory: At origin (2)." << std::endl;
#endif
					break;
				}
				
				if ( G.at(i)->V->empty() ) {
					continue;
				}

				PVT_State cur_state = T.back()->getInitialState();

				double cur_x = cur_state.getPathCoord();
				double cur_t = cur_state.getTimeCoord();
				double cur_v = cur_state.getVelocityCoord();
				
				PVT_ObstaclePoint * cur_p = G.at(i)->p;

				if ( Maths::approxGt(cur_p->getTimeCoord(), cur_t, epsilon)
					|| Maths::approxGt(cur_p->getPathCoord(), cur_x, epsilon)
					|| cur_state.atPoint(*cur_p, c) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Planner::BuildOptimalTrajectory: Invalid or null transition: " << *cur_p << " -> " << cur_state << std::endl;
#endif
					continue;
				}

				double cand_x = cur_p->getPathCoord();
				double cand_t = cur_p->getTimeCoord();
				
				// reflect the problem, find backward route with NextReachableSet
				double delta_x = cur_x - cand_x;
				double delta_t = cur_t - cand_t;
				double test_x = cur_x + delta_x;
				double test_t = cur_t + delta_t;
				PVT_Point p1( cur_x, cur_t );
				PVT_Point p2( test_x, test_t );
				Interval V_f( cur_v, cur_v );
				
				// adjust velocities for mirrored problem
				Constraints c_test( c, true );
				c_test.setXLimit( 2 * c.getXLimit() );
				c_test.setTLimit( 2 * c.getTLimit() );
				
				std::pair< std::vector<TrajectorySegment*>*, std::vector<TrajectorySegment*>* > Reachable;
				Reachable.first = NULL;
				Reachable.second = NULL;
				if ( !NextReachableSet(Reachable, p1, p2, V_f, c_test) ) {
					std::cerr << "ERROR IN Planner::BuildOptimalTrajectory: Planner::NextReachableSet failed." << std::endl;
					return false;
				}
				if ( (Reachable.first == NULL) || (Reachable.second == NULL) ) {
#ifdef SHOW_COMMENTS
					std::cout << "Planner::BuildOptimalTrajectory: NRS unreachable." << std::endl;
					std::cout << p1 << " -> " << p2 << ": " << V_f << std::endl;
#endif
					continue;
				}

				// get feasible initial velocities
				std::vector<TrajectorySegment*> * traj1 = Reachable.first;
				std::vector<TrajectorySegment*> * traj2 = Reachable.second;
				
				double traj1_v = traj1->back()->getFinalState().getVelocityCoord();
				double traj2_v = traj2->back()->getFinalState().getVelocityCoord();
								
				// feasible velocities at candidate point
				Interval V_i( traj2_v, traj1_v );
				std::vector<Interval*> result;
				Interval::intersect( result, V_i, *G.at(i)->V, c );
				if ( result.empty() ) {
#ifdef SHOW_COMMENTS
					std::cout.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
					std::cout << "p: ";
					for ( size_t l=0; l<G.at(i)->V->size(); l++ ) {
						std::cout << *G.at(i)->V->at(l) << std::endl;
					}
					std::cout << "V_i: " << V_i << std::endl;
					std::cout << "Planner::BuildOptimalTrajectory: No feasible velocities to " << *cur_p << std::endl;
#endif
					
					// free memory used by trajectories
					Utilities::CleanTrajectory( *Reachable.first );
					delete( Reachable.first );
					Utilities::CleanTrajectory( *Reachable.second );
					delete( Reachable.second );
					continue;
				}

				// collect bounds of feasible initial velocities
				std::vector<double> initial_velocities;
				for ( size_t j=0; j<result.size(); j++ ) {
					initial_velocities.push_back( result.at(j)->getMin() );
					initial_velocities.push_back( result.at(j)->getMax() );
				}
				
				// each of these bounds should produce a feasible trajectory
				std::vector< std::vector<TrajectorySegment*>* > trajes;
				for ( size_t j=0; j<initial_velocities.size(); j++) {
					PVT_State s1( cand_x, cand_t, initial_velocities.at(j));
					std::vector<TrajectorySegment*> * traj = new std::vector<TrajectorySegment*>();
					if ( !BuildPLP(*traj, s1, cur_state, c) ) {
						std::cerr << "ERROR IN Planner::BuildOptimalTrajectory: Planner::BuildPLP failed." << std::endl;
						return false;
					}
					if ( traj->empty() ) {
						std::cerr << "ERROR IN Planner::BuildOptimalTrajectory: Failed to find connecting PLP trajectory." << std::endl;
						std::cerr << V_i << " " << s1 << " " << cur_state << std::endl;
						delete( traj );
					} else {
						trajes.push_back( traj );
					}
				}
				
				// not feasibly connectable
				if ( trajes.empty() ) {
#ifdef SHOW_COMMENTS
					std::cout.precision(Constants::DEBUGGING_OUTPUT_PRECISION);
					std::cout << "Planner::BuildOptimalTrajectory: No feasible connection found" << std::endl;
					std::cout << *cur_p << " -> " << cur_state << std::endl;
#endif
					// free memory used by trajectories
					Utilities::CleanTrajectory( *Reachable.first );
					delete( Reachable.first );
					Utilities::CleanTrajectory( *Reachable.second );
					delete( Reachable.second );
					
					// free memory used by interval intersection
					for ( size_t j=0; j<result.size(); j++ ) {
						delete( result.at(j) );
					}
					
					continue;
				}

				// reverse trajectories - also need to swap states, so do it manually
				size_t j = 0;
				while ( 1 ) {
					if ( j >= trajes.size() ) {
						break;
					}
					std::vector<TrajectorySegment*> * traj = trajes.at(j);
					
					// collision check trajectory
					std::set<PVT_Obstacle*> inCollision;
					if ( !Collisions::checkTrajectory(inCollision, *traj, O, c) ) {
						std::cerr << "ERROR IN Planner::BuildOptimalTrajectory: Collision check failed." << std::endl;
						return false;
					}
					if ( !inCollision.empty() ) {
						Utilities::CleanTrajectory( *trajes.at(j) );
						delete( trajes.at(j) );
						trajes.erase( trajes.begin() + j );
						continue;
					}
					j++;
				}
				
				if ( trajes.empty() ) {
#ifdef SHOW_COMMENTS
					std::cout << "Planner::BuildOptimalTrajectory: No collision-free connection." << std::endl;
#endif
					
					// free memory used by trajectories
					Utilities::CleanTrajectory( *Reachable.first );
					delete( Reachable.first );
					Utilities::CleanTrajectory( *Reachable.second );
					delete( Reachable.second );
					
					// free memory used by interval intersection
					for ( size_t j=0; j<result.size(); j++ ) {
						delete( result.at(j) );
					}
					
					continue;
				}
				
				// at this point there is a feasible segment, add it
				for ( size_t j=0; j<trajes.size(); j++ ) {
					std::vector<TrajectorySegment*> * traj = trajes.at(j);
					for ( ssize_t k=traj->size()-1; k>=0; k-- ) {
						T.push_back( new TrajectorySegment(*traj->at(k)) );
					}
					break;
				}
				
				// free memory used by trajectories
				Utilities::CleanTrajectory( *Reachable.first );
				delete( Reachable.first );
				Utilities::CleanTrajectory( *Reachable.second );
				delete( Reachable.second );
				
				// free memory used by interval intersection
				for ( size_t j=0; j<result.size(); j++ ) {
					delete( result.at(j) );
				}
				
				// free memory used by trajes
				for ( size_t j=0; j<trajes.size(); j++ ) {
					Utilities::CleanTrajectory( *trajes.at(j) );
					delete( trajes.at(j) );
				}
			}
			
			// we'd better be at the origin at this point
			if ( T.empty() || !T.back()->getInitialState().atPoint(0., 0., c) ) {
				
				// clear trajectory
				Utilities::CleanTrajectory( T );
				
				// if there are no more goal trajectories to try
				if ( Goal.size() == 1 ) {
					std::cerr << "ERROR IN Planner::BuildOptimalTrajectory: Failed to find trajectory to origin." << std::endl;
					Utilities::PrintTrajectory(T);
					return false;
				}
				
				// remove this goal trajectory
				delete( Goal.at(0) );
				Goal.erase( Goal.begin() );
				
				// try again
				return BuildOptimalTrajectory(  T, G, Goal, O, c );
			}
			
			// trajectory is built in reverse order, so put it right
			std::reverse( T.begin(), T.end() );

			return true;
		}
		
	} // end Planner namespace

} // end PVTP namespace

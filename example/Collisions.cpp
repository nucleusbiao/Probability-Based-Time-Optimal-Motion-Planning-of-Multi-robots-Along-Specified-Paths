#include <stdio.h>
#include <errno.h>
#include <PVTP/Maths.hpp>
#include <PVTP/Collisions.hpp>

namespace PVTP {
	
	namespace Collisions {

		bool checkTrajectory( std::set<PVT_Obstacle*>& inCollision,
							 std::vector<TrajectorySegment*>& T,
							 PVT_ObstacleSet& O,
							 Constraints& c,
							 bool return_obstacles ) {
			
            if ( (T.size() == 0) || (O.obstacles.size() == 0) ) {
				return true;
			}
			for ( size_t i=0; i<T.size(); i++ ) {
				if ( !checkTrajectorySegment(inCollision, *T.at(i), O, c, return_obstacles) ) {
					std::cerr << "ERROR IN Collisions::checkTrajectory: problem checking segment" << std::endl;
					return false;
				}
			}
			
			return true;
		}
		
		bool checkTrajectorySegment( std::set<PVT_Obstacle*>& inCollision,
									TrajectorySegment& T,
									PVT_ObstacleSet& O,
									Constraints& c,
									bool return_obstacles ) {
			double epsilon = c.getEpsilon();
			
			double x1 = T.getInitialState().getPathCoord();
			double t1 = T.getInitialState().getTimeCoord();
			double v1 = T.getInitialState().getVelocityCoord();
			double x2 = T.getFinalState().getPathCoord();
			double t2 = T.getFinalState().getTimeCoord();
			double v2 = T.getFinalState().getVelocityCoord();
			
			double delta_v = v2 - v1;
			double delta_t = t2 - t1;

			
			if ( Maths::approxEq(delta_t, 0., epsilon) ) {
				return true;
			}
			
			double acc = Maths::A_FromV1_V2_T1_T2( 0., delta_v, 0., delta_t, epsilon );

			if ( Maths::isNaN(acc) ) {
				std::cerr << "ERROR IN Collisions::checkTrajectorySegment: delta_t = " << delta_t << ", delta_v = " << delta_v << std::endl;
				std::cerr << T.getInitialState() << " -> " << T.getFinalState() << std::endl;
				return false;
			}
			
            for ( size_t i=0; i<O.obstacles.size(); i++ ) {
                PVT_Obstacle o = O.obstacles.at(i);
                double x_o_min = o.vertices.at(0).getPathCoord();
                double t_o_max = o.vertices.at(0).getTimeCoord();
                double x_o_max = o.vertices.at(2).getPathCoord();
                double t_o_min = o.vertices.at(2).getTimeCoord();

				double delta_x1 = x_o_min - x1;
				double delta_x2 = x_o_max - x1;
				
				// obstacle before (in path dimension) trajectory
				if ( Maths::approxLe(x_o_max, x1, epsilon) ) {
					continue;
				}

				// obstacle after (in path dimension) trajectory
				if ( Maths::approxGe(x_o_min, x2, epsilon) ) {
					continue;
				}

				// obstacle before (in time dimesnion) this trajectory
				if ( Maths::approxLe(t_o_max, t1, epsilon) ) {
					continue;
				}

				// obstacle after (in time dimension) this trajectory
				if ( Maths::approxGe(t_o_min, t2, epsilon) ) {
					continue;
				}

				// if trajectory starts within obstacle bounds, set delta_x1 = 0
				if ( delta_x1 < 0. ) {
					delta_x1 = 0.;
				}
				
				//
				// Check trajectory endpoints
				//
				
				if ( Maths::approxGt(x1, x_o_min, epsilon) && Maths::approxLt(x1, x_o_max, epsilon)
					&& Maths::approxGt(t1, t_o_min, epsilon) && Maths::approxLt(t1, t_o_max, epsilon) ) {
                    inCollision.insert( &o );
					if ( !return_obstacles ) {
						return true;
					}
					continue;
				}

				if ( Maths::approxGt(x2, x_o_min, epsilon) && Maths::approxLt(x2, x_o_max, epsilon)
					&& Maths::approxGt(t2, t_o_min, epsilon) && Maths::approxLt(t2, t_o_max, epsilon) ) {
                    inCollision.insert(&o );
					if ( !return_obstacles ) {
						return true;
					}
					continue;
				}
				
				if ( !(Maths::approxEq(acc, 0., epsilon) && Maths::approxEq(v1, 0., epsilon)) ) {
					double t1_prime = t1 + Maths::motionT_FromV1_X1_X2_A( v1, 0., delta_x1, acc, epsilon );
					double t2_prime = t1 + Maths::motionT_FromV1_X1_X2_A( v1, 0., delta_x2, acc, epsilon );
					
					if ( Maths::isNaN(t2_prime) ) {
						if ( t1_prime > t_o_max ) {
							continue;
						}
					} else if ( (Maths::approxGe(t1_prime, t_o_max, epsilon) && Maths::approxGe(t2_prime, t_o_max, epsilon))
						|| (Maths::approxLe(t1_prime, t_o_min, epsilon) && Maths::approxLe(t2_prime, t_o_min, epsilon)) ) {
						continue;
					}
				}

				// if this point is reached, there is collision
                inCollision.insert( &o );
				
				if ( !return_obstacles ) {
					return true;
				}
			}
			
			return true;
		}

	} // end Collisions namespace

} // end PVTP namespace

#include <PVTP/PVT_ObstacleSet.hpp>

namespace PVTP {
	
	PVT_ObstacleSet::PVT_ObstacleSet () {
        this->obstacles.clear();
	}
	
	PVT_ObstacleSet::PVT_ObstacleSet ( PVT_ObstacleSet& O, Constraints& c ) {
        this->obstacles.clear();
        for ( size_t i=0; i<O.obstacles.size(); i++ ) {
            this->obstacles.push_back(PVT_Obstacle(O.obstacles[i], c ) );
		}
	}

	PVT_ObstacleSet::PVT_ObstacleSet ( double boxes[][4], size_t box_count, Constraints& c ) {
        this->obstacles.clear();
		this->init( boxes, box_count, c );
	}
	
	PVT_ObstacleSet::PVT_ObstacleSet ( std::vector< std::vector<double> > boxes, Constraints& c ) {
        this->obstacles.clear();
		this->init( boxes, c );
	}
	
	PVT_ObstacleSet::~PVT_ObstacleSet () {
        this->freeObstacles();
	}
    void PVT_ObstacleSet::clear() {
        this->obstacles.clear();
    }
    void PVT_ObstacleSet::initSet(const PVT_ObstacleSet& O, const Constraints& c ) {
        this->obstacles.clear();
        for ( size_t i=0; i<O.obstacles.size(); i++ ) {
            this->obstacles.push_back(PVT_Obstacle(O.obstacles[i], c ) );
        }
    }

	void PVT_ObstacleSet::init( double boxes[][4], size_t box_count, Constraints& c ) {
		double epsilon = c.getEpsilon();
		this->freeObstacles();
        this->obstacles.clear();
		for ( size_t i=0; i<box_count; i++ ) {
			if ( Maths::approxGe(boxes[i][0], boxes[i][1], epsilon) ||
				Maths::approxGe(boxes[i][2], boxes[i][3], epsilon) ) {
				continue;
			}
            this->obstacles.push_back(PVT_Obstacle(boxes[i], c) );
		}
	}

	void PVT_ObstacleSet::init( std::vector< std::vector<double> >& boxes, Constraints& c ) {
		double epsilon = c.getEpsilon();
		this->freeObstacles();
        this->obstacles.clear();
		for ( size_t i=0; i<boxes.size(); i++ ) {
			if ( Maths::approxGe(boxes.at(i).at(0), boxes.at(i).at(1), epsilon) ||
				Maths::approxGe(boxes.at(i).at(2), boxes.at(i).at(3), epsilon) ) {
				continue;
			}
            this->obstacles.push_back(PVT_Obstacle(boxes.at(i), c) );
		}
	}
	
    void PVT_ObstacleSet::add(std::vector< std::vector<double> >& boxes, Constraints& c ){
        double epsilon = c.getEpsilon();
        for ( size_t i=0; i<boxes.size(); i++ ) {
            if ( Maths::approxGe(boxes.at(i).at(0), boxes.at(i).at(1), epsilon) ||
                Maths::approxGe(boxes.at(i).at(2), boxes.at(i).at(3), epsilon) ) {
                continue;
            }
            this->obstacles.push_back(PVT_Obstacle(boxes.at(i), c) );
        }
    }

	void PVT_ObstacleSet::init( PVT_ObstacleSet& O, Constraints& c ) {
		this->freeObstacles();
        this->obstacles.clear();
        for ( size_t i=0; i<O.obstacles.size(); i++ ) {
            this->obstacles.push_back(PVT_Obstacle(O.obstacles.at(i), c ) );
		}
	}
	
	void PVT_ObstacleSet::obstacleUnion( PVT_ObstacleSet& O, Constraints& c ) {

        for ( size_t i=0; i<O.obstacles.size(); i++ ) {
            PVT_Obstacle o_other = O.obstacles.at( i );
			
			// add other obstacle if we're at the end of our set
            if ( i >= this->obstacles.size() ) {
                this->obstacles.push_back(PVT_Obstacle( o_other, c ) );
				continue;
			}
            PVT_Obstacle o_this = this->obstacles.at( i );
			
			// get respective coords
            double o_other_min_x = o_other.vertices.at(0).getPathCoord();
            double o_other_max_x = o_other.vertices.at(2).getPathCoord();
            double o_other_max_t = o_other.vertices.at(0).getTimeCoord();
            double o_other_min_t = o_other.vertices.at(2).getTimeCoord();
			
            double o_this_min_x = o_this.vertices.at(0).getPathCoord();
            double o_this_max_x = o_this.vertices.at(2).getPathCoord();
            double o_this_max_t = o_this.vertices.at(0).getTimeCoord();
            double o_this_min_t = o_this.vertices.at(2).getTimeCoord();
			
			// determine unioned coords
			double min_x = std::min( o_other_min_x, o_this_min_x );
			double max_x = std::max( o_other_max_x, o_this_max_x );
			double min_t = std::min( o_other_min_t, o_this_min_t );
			double max_t = std::max( o_other_max_t, o_this_max_t );
			
			// construct new unioned obstacle
            o_this.vertices.at(0).setCoords( min_x, max_t );
            o_this.vertices.at(1).setCoords( max_x, max_t );
            o_this.vertices.at(2).setCoords( max_x, min_t );
            o_this.vertices.at(3).setCoords( min_x, min_t );
		}
		
	}
	
	void PVT_ObstacleSet::translateObstacles( double x_offset, double t_offset ) {
		if ( (x_offset == 0.) && (t_offset == 0.) ) {
			return;
		}
        for ( size_t i=0; i<this->obstacles.size(); i++ ) {
            this->obstacles.at(i).translateObstacle( x_offset, t_offset );
		}
	}
	
	void PVT_ObstacleSet::growObstacles( double x_padding, double t_padding ) {
		if ( (x_padding == 0.) && (t_padding == 0.) ) {
			return;
		}
		this->growObstacles( x_padding, x_padding, t_padding, t_padding );
	}
	
	void PVT_ObstacleSet::growObstacles( double x_fore_padding, double x_aft_padding, double t_fore_padding, double t_aft_padding ) {
		if ( (x_fore_padding == 0.) && (x_aft_padding == 0.) && (t_fore_padding == 0.) && (t_aft_padding == 0.) ) {
			return;
		}
        for ( size_t i=0; i<this->obstacles.size(); i++ ) {
            this->obstacles.at(i).growObstacle( x_fore_padding, x_aft_padding, t_fore_padding, t_aft_padding );
		}
	}
	
	void PVT_ObstacleSet::freeObstacles() {
        this->obstacles.clear();
	}
	
    void PVT_ObstacleSet::getAllVertices(std::vector<PVT_ObstaclePoint*>& vertices, Constraints& c ){
        for ( size_t i=0; i<this->obstacles.size(); i++ ) {
            for ( size_t j=0; j<this->obstacles[i].vertices.size(); j++ ) {
                if ( this->obstacles[i].vertices.at(j).isReachable(c) ) {
                    vertices.push_back(&(this->obstacles[i].vertices[j]));
				}
			}
		}
	}
	
	bool PVT_ObstacleSet::inCollision( PVT_Point& p, Constraints& c ) const {
		return this->inCollision( p.getPathCoord(), p.getTimeCoord(), c );
	}
	
	bool PVT_ObstacleSet::inCollision( double path_position, double time, Constraints& c ) const {
		double epsilon = c.getEpsilon();
		
        for ( size_t i=0; i<this->obstacles.size(); i++ ) {
			
			// current obstacle
            PVT_Obstacle  o = this->obstacles.at( i );
			
			// get obstacle coordinates
            double o_min_x = o.vertices.at(0).getPathCoord();
            double o_max_x = o.vertices.at(2).getPathCoord();
            double o_max_t = o.vertices.at(0).getTimeCoord();
            double o_min_t = o.vertices.at(2).getTimeCoord();
			
			// check for containment; note that the planner assumes PT
			// obstacles to be open sets, hence the strict inequalities
			if ( Maths::approxLt(time, o_max_t, epsilon) && Maths::approxGt(time, o_min_t, epsilon)
				&& Maths::approxLt(path_position, o_max_x, epsilon) && Maths::approxGt(path_position, o_min_x, epsilon) ) {
				return true;
			}
		}
		return false;			
	}
	
	std::ostream& operator<<(std::ostream& out, const PVT_ObstacleSet& of) {
        for ( size_t i=0; i<of.obstacles.size(); i++ ) {
			std::cout << "Obstacle:" << std::endl;
            out << (of.obstacles.at(i));
		}
		return out;
	}

} // end PVTP namespace

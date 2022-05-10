#include <PVTP/Constants.hpp>
#include <PVTP/PVT_Obstacle.hpp>

namespace PVTP {
	
	PVT_Obstacle::PVT_Obstacle ( double box[], Constraints& c ) {
        this->vertices.clear();
		this->init( box[0], box[1], box[2], box[3], c );
	}
	
	PVT_Obstacle::PVT_Obstacle ( std::vector<double>& box, Constraints& c ) {
        this->vertices.clear();
		this->init( box.at(0), box.at(1), box.at(2), box.at(3), c );
	}
	
    PVT_Obstacle::PVT_Obstacle ( const PVT_Obstacle& obs, const Constraints& c ) {
        this->vertices.clear();
        for ( size_t i=0; i<obs.vertices.size(); i++ ) {
            this->vertices.push_back(PVT_ObstaclePoint(obs.vertices.at(i), c ));
		}
	}
	
	void PVT_Obstacle::init( double min_x, double max_x, double min_t, double max_t, Constraints& c ) {
		/**/
        this->vertices.clear();
		/**/
        this->vertices.push_back(PVT_ObstaclePoint( min_x,
													  max_t,
													  -std::numeric_limits<double>::max(),
													  std::numeric_limits<double>::max(),
													  Constants::H_CLASS_ABOVE,
                                                      c ));
		
		// upper-right
        this->vertices.push_back(PVT_ObstaclePoint( max_x,
													  max_t,
													  std::numeric_limits<double>::max(),
													  -std::numeric_limits<double>::max(),
													  Constants::H_CLASS_ABOVE,
                                                      c ));
		
		// lower-right
        this->vertices.push_back(PVT_ObstaclePoint( max_x,
													  min_t,
													  -std::numeric_limits<double>::max(),
													  std::numeric_limits<double>::max(),
													  Constants::H_CLASS_BELOW,
                                                      c ));
		
		// lower-left
        this->vertices.push_back(PVT_ObstaclePoint( min_x,
													  min_t,
													  std::numeric_limits<double>::max(),
													  -std::numeric_limits<double>::max(),
													  Constants::H_CLASS_BELOW,
                                                      c ));
	}
	
	PVT_Obstacle::~PVT_Obstacle () {}
	
	void PVT_Obstacle::growObstacle( double x_padding, double t_padding ) {
		this->growObstacle( x_padding, x_padding, t_padding, t_padding );
	}
	
	void PVT_Obstacle::growObstacle( double x_fore_padding, double x_aft_padding, double t_fore_padding, double t_aft_padding ) {
		// upper-left
        this->vertices.at(0).setCoords( this->vertices.at(0).getPathCoord() - x_fore_padding, this->vertices.at(0).getTimeCoord() + t_aft_padding );
		
		// upper-right
        this->vertices.at(1).setCoords( this->vertices.at(1).getPathCoord() + x_aft_padding, this->vertices.at(1).getTimeCoord() + t_aft_padding );
		
		// lower-right
        this->vertices.at(2).setCoords( this->vertices.at(2).getPathCoord() + x_aft_padding, this->vertices.at(2).getTimeCoord() - t_fore_padding );
		
		// lower-left
        this->vertices.at(3).setCoords( this->vertices.at(3).getPathCoord() - x_fore_padding, this->vertices.at(3).getTimeCoord() - t_fore_padding );
	}
	
	double PVT_Obstacle::getMinPathCoord() const {
        return this->vertices.at(0).getPathCoord();
	}
	
	double PVT_Obstacle::getMaxPathCoord() const {
        return this->vertices.at(2).getPathCoord();
	}

	double PVT_Obstacle::getMinTimeCoord() const {
        return this->vertices.at(2).getTimeCoord();
	}

	double PVT_Obstacle::getMaxTimeCoord() const {
        return this->vertices.at(0).getTimeCoord();
	}
	
	void PVT_Obstacle::translateObstacle( double x_offset, double t_offset ) {
        for ( size_t i=0; i<this->vertices.size(); i++ ) {
            this->vertices.at(i).translate( x_offset, t_offset );
		}
	}
	
	bool PVT_Obstacle::equals( const PVT_Obstacle& other_obstacle, Constraints& c ) const {
        if ( this->vertices.size() != other_obstacle.vertices.size() ) {
			return false;
		}
        for ( size_t i=0; i<this->vertices.size(); i++ ) {
            if ( !this->vertices.at(i).equals(other_obstacle.vertices.at(i), c) ) {
				return false;
			}
		}
		return true;
	}

	std::ostream& operator<<(std::ostream& out, const PVT_Obstacle& o) {
        for ( size_t i=0; i<o.vertices.size(); i++ ) {
            out << (o.vertices.at(i)) << std::endl;
		}
		return out;
	}

} // end PVTP namespace

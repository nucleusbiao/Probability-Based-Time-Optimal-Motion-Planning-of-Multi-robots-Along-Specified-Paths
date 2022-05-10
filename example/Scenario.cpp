#include <PVTP/Utilities.hpp>
#include <PVTP/Scenario.hpp>

namespace SCIMP_Scenario {
	
	Scenario::Scenario ( Vehicle& vehicle, Path& path ) {
		this->user_half_width = vehicle.getWidth() / 2.;
		this->user_length = vehicle.getLength();
		this->path_min_x = path.getInitialPoint().getX();
		this->path_max_x = path.getFinalPoint().getX();
		this->path_y = path.getInitialPoint().getY();
	}
	
    Scenario::Scenario (const Scenario& scenario ) {
        for ( size_t i=0; i<scenario.obstacles.size(); i++ ) {
            this->addObstacle( scenario.obstacles[i] );
		}
		this->user_half_width = scenario.user_half_width;
		this->user_length = scenario.user_length;
		this->path_min_x = scenario.path_min_x;
		this->path_max_x = scenario.path_max_x;
		this->path_y = scenario.path_y;
	}
	
    Scenario::Scenario() {}
    Scenario::~Scenario() {}
    void Scenario::initSet( Vehicle& vehicle, Path& path ) {
        this->user_half_width = vehicle.getWidth() / 2.;
        this->user_length = vehicle.getLength();
        this->path_min_x = path.getInitialPoint().getX();
        this->path_max_x = path.getFinalPoint().getX();
        this->path_y = path.getInitialPoint().getY();
    }

    void Scenario::initSet(const Scenario& scenario) {
        for ( size_t i=0; i<scenario.obstacles.size(); i++ ) {
            this->addObstacle( scenario.obstacles[i] );
        }
        this->user_half_width = scenario.user_half_width;
        this->user_length = scenario.user_length;
        this->path_min_x = scenario.path_min_x;
        this->path_max_x = scenario.path_max_x;
        this->path_y = scenario.path_y;
    }
    void Scenario::addObstacle(const ScenarioObstacle& obstacle ) {
        this->obstacles.push_back(ScenarioObstacle(obstacle));
	}
    void Scenario::addObstacle(ScenarioObstacle& obstacle ){
        this->obstacles.push_back(ScenarioObstacle(obstacle));
    }
	bool Scenario::computePathTimeObstacles( PVT_ObstacleSet& obstacle_set, Constraints& c ) const {

		// storage for constructed obstacles
		std::vector< std::vector<double> > boxes;
		
		// get dimensions of user vehicle
		double user_half_width = this->user_half_width;
		double user_length = this->user_length;
		
		// get extents of user path
		double path_min_x = this->path_min_x;
		double path_max_x = this->path_max_x;
		
		// get horizontal position of path
		double path_y = this->path_y;
		
		// extend user path by length of the vehicle
		path_max_x += user_length;
		
		// build obstacles
        for ( size_t i=0; i<this->obstacles.size(); i++ ) {
			
			/**
			 * Convert obstacles to configuration space such that user vehicle is a point:
			 *
			 *	1. Extend obstacle's near side by user vehicle length
			 *	2. Extend obstacle's front by 1/2 user vehicle width
			 *	3. Extend obstacle's rear by 1/2 user vehicle width
			 */
			
            ScenarioObstacle obstacle = this->obstacles[i];
			
			// get vertical extents of this obstacle's path
            double obs_path_min_y = obstacle.getPath().getFinalPoint().getY() - path_y;
            double obs_path_max_y = obstacle.getPath().getInitialPoint().getY() - path_y;
			
			// the minimum y position this obstacle achieves in configuration space
            double obs_min_y_pos = obs_path_min_y - obstacle.getVehicle().getLength() - user_half_width;
			
			// the maximum y position this obstacle achieves in configuration space
			double obs_max_y_pos = obs_path_max_y + user_half_width;
			
			// if the path never intersects the user path, ignore it
			if ( (obs_min_y_pos >= 0.) || (obs_max_y_pos <= 0.) ) {
				std::cout << "Ignoring vertically non-intersecting obstacle path: [" << obs_min_y_pos << ", " << obs_max_y_pos << "]" << std::endl;
				continue;
			}
			
			// get extents of obstacle vehicle
            double obs_min_x = obstacle.getMinX( 0. );
            double obs_max_x = obstacle.getMaxX( 0. );
            double obs_max_y = obs_path_max_y - obstacle.getVehicle().getPosition();
            double obs_min_y = obs_max_y - obstacle.getVehicle().getLength();
			
			// if this obstacle is outside that path, ignore it
			if ( (obs_min_x >= path_max_x) || (obs_max_x <= path_min_x) ) {
				std::cout << "Ignoring horizontally non-intersecting obstacle." << std::endl;
				std::cout << "path_x: [" << path_min_x << ", " << path_max_x << "], obs_y: [" << obs_min_x << ", " << obs_max_x << "]" << std::endl;
				continue;
			}
			
			// extend obstacle's near side by user vehicle length
			obs_min_x -= user_length;
			
			// extend front and rear by half user vehicle width
			obs_min_y -= user_half_width;
			obs_max_y += user_half_width;
			
			// truncate obstacle path extents; we're only interested in what's over the path
			obs_min_x = Utilities::truncateValue( obs_min_x, path_min_x, path_max_x );
			obs_max_x = Utilities::truncateValue( obs_max_x, path_min_x, path_max_x );
			
			// compute near and far time coordinates
            double q_a = 0.5 * obstacle.getVehicle().getAcceleration();
            double q_b = obstacle.getVehicle().getVelocity();
			double q_c = -obs_min_y;
			std::pair<double, double> roots( std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN() );
			Maths::quadraticOrdered( roots, q_a, q_b, q_c, 0. );

			// if the first root is NaN, that means the obstacle is currently sitting over the path
			double obs_min_t;
			if ( Maths::isNaN(roots.first) ) {
				obs_min_t = 0.;
			} else {
				obs_min_t = roots.first;
			}
			
			// sanity check
			if ( Maths::isNaN(obs_min_t) ) {
				std::cerr << "ERROR IN Scenario::computePathTimeObstacles: Invalid extents in time axis for obstacle found." << std::endl;
				std::cerr << "min: " << obs_min_t << std::endl;
                std::cerr << "velocity: " << obstacle.getVehicle().getVelocity() << ", acceleration: " << obstacle.getVehicle().getAcceleration() << std::endl;
				return false;
			}
			
			q_c = -obs_max_y;
			roots.first = std::numeric_limits<double>::quiet_NaN();
			roots.second = std::numeric_limits<double>::quiet_NaN();
			Maths::quadraticOrdered( roots, q_a, q_b, q_c, 0. );
			double obs_max_t = roots.first;
			
			// if obs_max_t is NaN, it means the vehicle begins crossing the path, but, because of deceleration, never finishes
			if ( Maths::isNaN(obs_max_t) ) {
				obs_max_t = c.getTLimit();
			}
			
			// project obstacles onto user path, record intersections as obstacle extents
			// [min_x max_x min_t max_t]
			std::vector<double> box;
			box.push_back( obs_min_x );
			box.push_back( obs_max_x );
			box.push_back( obs_min_t );
			box.push_back( obs_max_t );
			
			// add box to obstacle set
			boxes.push_back( box );
		}
		
		// create obstacle set
		obstacle_set.init( boxes, c );
		
		return true;
	}
	
	std::ostream& operator<<( std::ostream& out, const Scenario& scenario ) {
        for ( size_t i=0; i<scenario.obstacles.size(); i++ ) {
            out << scenario.obstacles[i] << std::endl;
		}
		return out;
	}
	
} // end SCIMP_Scenario namespace

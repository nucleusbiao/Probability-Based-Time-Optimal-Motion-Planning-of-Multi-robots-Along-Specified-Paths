#ifndef PVTP_OBSTACLE_SET_H
#define PVTP_OBSTACLE_SET_H

#include <PVTP/PVT_Obstacle.hpp>

namespace PVTP {
	
	/**
	 * Path-Velocity-Time Obstacle Field class. This class acts as a container for all
	 * obstacles in a given scenario, and has convenience methods for extracting
	 * information from the field.
	 */
	class PVT_ObstacleSet {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const PVT_ObstacleSet& of);
		
	public:
		
		/**
		 * Constructor that builds an empty set
		 */
		PVT_ObstacleSet ();

		/**
		 * Obstacle field constructor. Rectangular obstacles.
		 */
		PVT_ObstacleSet ( double boxes[][4], size_t box_count, Constraints& c );
		
		/**
		 * Obstacle field constructor. Rectangular obstacles.
		 */
		PVT_ObstacleSet ( std::vector< std::vector<double> > boxes, Constraints& c );
		
		/**
		 * Obstacle field constructor. Copy constructor
		 */
		PVT_ObstacleSet ( PVT_ObstacleSet& O, Constraints& c );
		
		/**
		 * Destructor
		 */
		~PVT_ObstacleSet ();
        void initSet(const PVT_ObstacleSet& O, const Constraints& c );
		/**
		 * Initialization routine: double array
		 */
		void init( double boxes[][4], size_t box_count, Constraints& c );
        void clear();
		/**
		 * Initialization routine: vector of doubles
		 */
		void init( std::vector< std::vector<double> >& boxes, Constraints& c );

        void add(std::vector< std::vector<double> >& boxes, Constraints& c);
		/**
		 * Initialization routine: obstacle set
		 */
		void init( PVT_ObstacleSet& O, Constraints& c );
		
		/**
		 * Union this set of obstacles with another. The union is performed
		 * sequentially: The obstacles at index i in each set are unioned
		 * together, then i+1, and so on. If one set contains more obstacles
		 * than the other, the extra obstacles are copied over, therefore, this
		 * operation always produces two obstacles sets of the same size.
		 *
		 * The resulting set of unioned obstacles replaces the set of the
		 * calling object. The set belonging to the passed object is unchanged.
		 */
		void obstacleUnion( PVT_ObstacleSet& O, Constraints& c );
		
		/**
		 * Retrieve the vertices of all obstacles in this field, store in the
		 * vector pointed to by the passed reference.
		 */
        void getAllVertices(std::vector<PVT_ObstaclePoint*>& vertices, Constraints& c );
		
		/**
		 * Translate a set of obstacles by some offset in path and some offset in time
		 */
		void translateObstacles( double x_offset, double t_offset );
		
		/**
		 * Put a margin around all obstacles; padding applied equally to both sides.
		 *
		 * NOTE: This assumes rectangular obstacles.
		 */
		void growObstacles( double x_padding, double t_padding );
		
		/**
		 * Put a margin around all obstacles. "fore" refers to the side nearest
		 * the origin, "aft" the side farthest from the origin.
		 *
		 * NOTE: This assumes rectangular obstacles.
		 */
		void growObstacles( double x_fore_padding, double x_aft_padding, double t_fore_padding, double t_aft_padding );
		
		/**
		 * Given a current path position and a set of PT obstacles, check for collision
		 *
		 * NOTE: This method currently assumes polygonal obstacles!
		 */
		bool inCollision( double path_position, double time, Constraints& c ) const;
		
		/**
		 * Convenience wrapper for inCollision
		 *
		 * NOTE: This method currently assumes polygonal obstacles!
		 */
		bool inCollision( PVT_Point& p, Constraints& c ) const;
		
		/**
		 * Vector of obstacles.
		 */
        std::vector<PVT_Obstacle> obstacles;
		
	private:
		
		/**
		 * Free from memory the obstacle set
		 */
		void freeObstacles();
		
	};
	
} // end PVTP namespace

#endif

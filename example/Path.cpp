#include <PVTP/Path.hpp>

namespace SCIMP_Scenario {
	
	Path::Path ( XY_Point& p1, XY_Point& p2 ) {
		this->segments = new std::vector< std::pair<LinearSegment*, EllipticalSegment*>* >();
		this->addLinearSegment( p1, p2 );
	}
    Path::Path () {}
	Path::Path ( Path& path ) {
		this->segments = new std::vector< std::pair<LinearSegment*, EllipticalSegment*>* >();
		/**/
		for ( size_t i=0; i<path.segments->size(); i++ ) {
			std::pair<LinearSegment*, EllipticalSegment*> * pair = new std::pair<LinearSegment*, EllipticalSegment*>( NULL, NULL );
			if ( path.segments->at(i)->first != NULL ) {
				pair->first = new LinearSegment( *path.segments->at(i)->first );
			}
			this->segments->push_back( pair );
		}
	}
	
	Path::~Path () {
		for ( size_t i=0; i<this->segments->size(); i++ ) {
			std::pair<LinearSegment*, EllipticalSegment*> * segment = this->segments->at(i);
			if ( segment->first != NULL ) {
				delete( segment->first );
			}
			if ( segment->second != NULL ) {
				delete( segment->second );
			}
			delete( segment );
		}
		this->segments->clear();
		delete( this->segments );
	}
	

    void Path::initSet( const Path& path ) {
        this->segments = new std::vector< std::pair<LinearSegment*, EllipticalSegment*>* >();
        /**/
        for ( size_t i=0; i<path.segments->size(); i++ ) {
            std::pair<LinearSegment*, EllipticalSegment*> * pair = new std::pair<LinearSegment*, EllipticalSegment*>( NULL, NULL );
            if ( path.segments->at(i)->first != NULL ) {
                pair->first = new LinearSegment( *path.segments->at(i)->first );
            }
            this->segments->push_back( pair );
        }
    }
	XY_Point& Path::getInitialPoint() const {
		size_t index = 0;
		return this->segments->at(index)->first->getInitialPoint();
	}
	
	XY_Point& Path::getFinalPoint() const {
		size_t index = this->segments->size() - 1;
		return this->segments->at(index)->first->getFinalPoint();
	}
	
	double Path::getLength() const {
		double length = 0.;
		for ( size_t i=0; i<this->segments->size(); i++ ) {
			if ( this->segments->at(i)->first != NULL ) {
				length += this->segments->at(i)->first->getLength();
			}
		}
		return length;
	}
	
	void Path::addLinearSegment( XY_Point& p1, XY_Point& p2 ) {
		
		// create new pair, which will be the new segment
		std::pair<LinearSegment*, EllipticalSegment*> * segment = new std::pair<LinearSegment*, EllipticalSegment*>();
		
		// set elliptical member to null
		segment->second = NULL;
		
		// create linear member
		segment->first = new LinearSegment( p1, p2 );
		
		// add new segment
		this->segments->push_back( segment );
		
	}
	
	void Path::addEllipticalSegment( XY_Point& p1, XY_Point& p2, bool convex ) {
		
		// create new pair, which will be the new segment
		std::pair<LinearSegment*, EllipticalSegment*> * segment = new std::pair<LinearSegment*, EllipticalSegment*>();
		
		// set linear member to null
		segment->first = NULL;
		
		// determine ellipse parameters
		double A = p2.getX() - p1.getX();
		double B = p2.getY() - p1.getY();
		
		// determine center
		double x_center;
		double y_center;
		if ( convex ) {
			x_center = p2.getX();
			y_center = p1.getY();			
		} else {
			x_center = p1.getX();
			y_center = p2.getY();
		}
		XY_Point center( x_center, y_center );
		
		// create elliptical member
		segment->second = new EllipticalSegment( A, B, center );
		
		// add new segment
		this->segments->push_back( segment );
		
	}
	
	std::ostream& operator<<( std::ostream& out, const Path& path ) {
		for ( size_t i=0; i<path.segments->size(); i++ ) {
			if ( path.segments->at(i) != NULL ) {
				if ( path.segments->at(i)->first != NULL ) {
					out << "[" << *path.segments->at(i)->first << "] ";
				} else if ( path.segments->at(i)->second != NULL ) {
					out << "[" << *path.segments->at(i)->second << "] ";
				}
			}
		}
		return out;
	}
	
} // end PVTP namespace

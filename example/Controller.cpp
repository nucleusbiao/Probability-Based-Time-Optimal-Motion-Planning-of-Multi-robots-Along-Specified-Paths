#include <PVTP/Controller.hpp>

namespace SCIMP_Scenario {
	
	Controller::Controller () {
		this->control_sequence = new std::vector< std::pair<double, double>* >();
		
		// add initial control
		this->control_sequence->push_back( new std::pair<double, double>(0., 0.) );
		
		this->control_index = 0;
	}

	Controller::Controller ( Controller& controller ) {
		this->control_sequence = new std::vector< std::pair<double, double>* >();
		this->control_index = controller.control_index;
		for ( size_t i=0; i<controller.control_sequence->size(); i++ ) {
			std::pair<double, double> * control_pair = new std::pair<double, double>( controller.control_sequence->at(i)->first, controller.control_sequence->at(i)->second );
			this->control_sequence->push_back( control_pair );
		}
	}

	Controller::~Controller () {
		for ( size_t i=0; i<this->control_sequence->size(); i++ ) {
			delete( this->control_sequence->at(i) );
		}
		this->control_sequence->clear();
		delete( this->control_sequence );
	}

    void  Controller::initSet(const Controller& controller ) {
        this->control_sequence = new std::vector< std::pair<double, double>* >();
        this->control_index = controller.control_index;
        for ( size_t i=0; i<controller.control_sequence->size(); i++ ) {
            std::pair<double, double> * control_pair = new std::pair<double, double>( controller.control_sequence->at(i)->first, controller.control_sequence->at(i)->second );
            this->control_sequence->push_back( control_pair );
        }
    }

	double Controller::getControl( double time ) {
		
		// we assume non-negative time
		if ( time < 0 ) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		
		// get the index of the the control nearest this time without
		// going below
		ssize_t new_index = this->seek( time );
		
		// if we're beyond the control sequence, return the last control
		if ( new_index >= (ssize_t)this->control_sequence->size() ) {
			this->control_index = this->control_sequence->size() - 1;
			return this->getControlControl();
		}
		
		double index_time = this->getIndexTime( new_index );
		
		// if we found it, we're done
		if ( time == index_time ) {
			this->control_index = new_index;
			return this->getControlControl();
		}
		
		/**
		 * At this point, time is guaranteed to be < index_time, so assume
		 * the control for the given time is a continuation of the control
		 * at the previous index
		 */
		
		// decrement index
		new_index--;
		
		if ( new_index < 0 ) {
			return std::numeric_limits<double>::quiet_NaN();
		}
		
		// return control
		this->control_index = new_index;
		return this->getControlControl();
		
	}
	
	void Controller::addControl( double time, double control ) {
		
		// we assume non-negative time
		if ( time < 0. ) {
			return;
		}
		
		// build new control
		std::pair<double, double> * control_point = new std::pair<double, double>( time, control );
		
		// get index for control
		ssize_t new_index = this->seek( time );
		
		// add new control to end of sequence
		if ( new_index == (ssize_t)this->control_sequence->size() ) {
			this->control_sequence->push_back( control_point );
			return;
		}
		
		// replace existing control if times are equal
		if ( time == this->getIndexTime(new_index) ) {
			delete( control_point );
			this->control_sequence->at(new_index)->second = control;
			return;
		}
		
		// for insertion operations, get an iterator
		std::vector< std::pair<double, double>* >::iterator it;
		it = this->control_sequence->begin();
		
		// insert control
		this->control_sequence->insert( it + new_index, control_point );
	}
	
	double Controller::getControlTime() const {
		return this->getIndexTime( this->control_index );
	}
	
	double Controller::getIndexTime( size_t index ) const {
		return this->control_sequence->at(index)->first;
	}
	
	double Controller::getControlControl() const {
		return this->getIndexControl( this->control_index );
	}
	
	double Controller::getIndexControl( size_t index ) const {
		return this->control_sequence->at(index)->second;
	}
	
	ssize_t Controller::seek( double time ) {
		
		ssize_t new_index = (ssize_t)this->control_index;
		ssize_t sequence_size = (ssize_t)this->control_sequence->size();
		
		// seek forward
		if ( time > this->getIndexTime(new_index) ) {
			
			while ( new_index < sequence_size ) {
				if ( time < this->getIndexTime(new_index) ) {
					break;
				}
				new_index++;
			}
		
		// seek backward
		} else {
			
			while ( new_index >= 0 ) {
				if ( time > this->getIndexTime(new_index) ) {
					break;
				}
				new_index--;
			}
			
			// increment new_index by 1, since the calling function assumes
			// it points to the member immediately following the
			// given time
			new_index++;
			
		}
		
		return new_index;
	}
	
	std::ostream& operator<<( std::ostream& out, const Controller& controller ) {
		for ( size_t i=0; i<controller.control_sequence->size(); i++ ) {
			out << "[" << controller.control_sequence->at(i)->first << ", " << controller.control_sequence->at(i)->second << "]" << std::endl;
		}
		return out;
	}

} // end SCIMP_Scenario namespace

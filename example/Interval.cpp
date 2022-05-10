#include <utility>
#include <stack>
#include <PVTP/Interval.hpp>
#include <PVTP/Constants.hpp>

namespace PVTP {

Interval::Interval ( void ) {
    this->setEmpty( true );
}

Interval::Interval ( Interval& i ) {
    if ( i.isEmpty() ) {
        this->setEmpty( true );
    } else {
        this->init( i.getMin(), i.getMax() );
    }
}

Interval::Interval ( double min, double max ) {
    this->init( min, max );
}
void Interval::init( double min, double max ) {
    if ( this->setBounds( min, max ) );
    else
        this->setEmpty( true );
}
void Interval::initSet( double min, double max) {
    this->init( min, max );
}
void Interval::initSet( const Interval& i) {
    if ( i.isEmpty() ) {
        this->setEmpty( true );
    } else {
        this->init( i.getMin(), i.getMax() );
    }
}

bool Interval::setBounds( double min, double max ) {
    if ( this->boundsValid(min, max) ) {
        this->min = min;
        this->max = max;
        this->setEmpty( false );
        return true;
    }
    return false;
}

bool Interval::contains( double num, Constraints& c ) const {
    if ( this->isEmpty() ) {
        return false;
    }
    double epsilon = c.getEpsilon();
    return Maths::approxGe( num, this->getMin(), epsilon )
            && Maths::approxLe( num, this->getMax(), epsilon );
}

void Interval::intersect( Interval& B, Constraints& c ) {
    if ( B.isEmpty() ) {
        this->setEmpty( true );
        return;
    }
    //if ( (B.getMin() > this->getMax()) || (this->getMin() > B.getMax()) ) {
    if ( Maths::approxGt(B.getMin(), this->getMax(), c.getEpsilon())
         || Maths::approxGt(this->getMin(), B.getMax(), c.getEpsilon()) ) {
        this->setEmpty( true );
        return;
    }
    double range_min = std::max( this->getMin(), B.getMin() );
    double range_max = std::min( this->getMax(), B.getMax() );
    this->setBounds( std::min(range_min, range_max), std::max(range_min, range_max) );
}

void Interval::intersect( Interval& result, Interval& A, Interval& B, Constraints& c ) {
    if ( A.isEmpty() || B.isEmpty() ) {
        result.setEmpty( true );
        return;
    }
    //if ( (B.getMin() > A.getMax()) || (A.getMin() > B.getMax()) ) {
    if ( Maths::approxGt(B.getMin(), A.getMax(), c.getEpsilon())
         || Maths::approxGt(A.getMin(), B.getMax(), c.getEpsilon()) ) {
        result.setEmpty( true );
        return;
    }
    double range_min = std::max( A.getMin(), B.getMin() );
    double range_max = std::min( A.getMax(), B.getMax() );
    result.init( std::min(range_min, range_max), std::max(range_min, range_max) );
}

void Interval::intersect( std::vector<Interval*>& result,
                          Interval& A,
                          std::vector<Interval*>& B,
                          Constraints& c ) {
    if ( A.isEmpty() ) {
        return;
    }
    for ( size_t i=0; i<B.size(); i++ ) {
        if ( B.at(i)->isEmpty() ) {
            continue;
        }
        Interval * B_int = new Interval();
        intersect( *B_int, A, *B.at(i), c );
        if ( B_int->isEmpty() ) {
            delete( B_int );
            continue;
        }
        result.push_back( B_int );
    }
}

void Interval::intersect( std::vector<Interval*>& result,
                          std::vector<Interval*>& A,
                          std::vector<Interval*>& B,
                          Constraints& c ) {
    if ( A.empty() || B.empty() ) {
        return;
    }

    for ( size_t i=0; i<A.size(); i++ ) {
        intersect( result, *A.at(i), B, c );
    }
}

void Interval::specialUnion( std::pair<Interval*,
                             Interval*>& result,
                             Interval& A,
                             Interval& B,
                             Constraints& c ) {
    double epsilon = c.getEpsilon();
    if ( A.isSubset(B, c) ) {
        result.first->init( B.getMin(), B.getMax() );
        result.second->setEmpty( true );
        return;
    }
    if ( B.isSubset(A, c) ) {
        result.first->init( A.getMin(), A.getMax() );
        result.second->setEmpty( true );
        return;
    }
    if ( Maths::approxLe(A.getMin(), B.getMin(), epsilon)
         && Maths::approxGe(A.getMax(), B.getMin(), epsilon) ) {
        result.first->init( A.getMin(), B.getMax() );
        result.second->setEmpty( true );
        return;
    }
    if ( Maths::approxLe(B.getMin(), A.getMin(), epsilon)
         && Maths::approxGe(B.getMax(), A.getMin(), epsilon) ) {
        result.first->init( B.getMin(), A.getMax() );
        result.second->setEmpty( true );
        return;
    }
    result.first->init( A.getMin(), A.getMax() );
    result.second->init( B.getMin(), B.getMax() );
}

void Interval::specialUnionOfIntervals( std::vector<Interval*>& result,
                                        std::vector<Interval*>& intervals ) {
    if ( intervals.empty() ) {
        return;
    }

    size_t i = 0;
    size_t j = 0;
    std::vector< std::pair<double, int> > values( 2*intervals.size() );
    for ( i=0; i<intervals.size(); i++ ) {
        if ( intervals.at(i)->isEmpty() ) {
            continue;
        }
        values.at(j++) = std::make_pair( intervals.at(i)->getMin(), Constants::INTERVAL_MIN_MARKER );
        values.at(j++) = std::make_pair( intervals.at(i)->getMax(), Constants::INTERVAL_MAX_MARKER );
    }

    // Construct a comparator for sorting
    struct IntervalComparator comp;

    // Sort by value, then by type (min or max) descendingly
    std::sort( values.begin(), values.end(), comp );

    // The stack will be used to construct the merged intervals
    std::stack< std::pair<double, int> > Stk;
    for ( size_t i=0; i<values.size(); i++ ) {

        // If the stack is empty, add the next value; should always be max
        if ( Stk.empty() ) {
            Stk.push( values.at(i) );
            continue;
        }

        // If the stack is not empty, do a peek:
        // If the top is a max value...
        if ( Stk.top().second == Constants::INTERVAL_MAX_MARKER ) {

            // ...if the incoming value is a min, pop...
            if ( values.at(i).second == Constants::INTERVAL_MIN_MARKER ) {

                std::pair<double, int> tmp = Stk.top();
                Stk.pop();

                // If that pop resulted in an empty stack, save interval
                if ( Stk.empty() ) {
                    result.push_back( new Interval(values.at(i).first, tmp.first) );
                }

                // ...otherwise, add value to stack
            } else {

                Stk.push( values.at(i) );

            }

        }

        // It should never be the case that a min element is at the top of the stack
    }

    // result now contains the merged intervals
}

bool Interval::isSubset( Interval& B, Constraints& c ) const {
    double epsilon = c.getEpsilon();

    if ( this->isEmpty() ) {
        return true;
    }
    if ( B.isEmpty() ) {
        return false;
    }
    if ( Maths::approxGe(this->getMin(), B.getMin(), epsilon)
         && Maths::approxLe(this->getMax(), B.getMax(), epsilon) ) {
        return true;
    }
    return false;
}

bool Interval::boundsValid( double min, double max ) {
    if ( min > max ) {
        return false;
    }
    return true;
}

void Interval::setEmpty( bool empty ) {
    if ( empty ) {
        this->min = std::numeric_limits<double>::max();
        this->max = -std::numeric_limits<double>::max();
    }
    this->empty = empty;
}

bool Interval::isEqual( Interval& B, Constraints& c ) const {
    double epsilon = c.getEpsilon();
    return Maths::approxEq( this->getMin(), B.getMin(), epsilon )
            && Maths::approxEq( this->getMax(), B.getMax(), epsilon );
}

bool Interval::isEmpty() const {
    return this->empty;
}

double Interval::getMin() const {
    return this->min;
}

double Interval::getMax() const {
    return this->max;
}

double Interval::getMidPoint() const {
    return (this->getMax() + this->getMin()) / 2.;
}

std::ostream& operator<<(std::ostream& out, const Interval& i) {
    if ( i.isEmpty() ) {
        return out << "(empty)";
    }
    out << "[";
    if ( i.getMin() == -std::numeric_limits<double>::max() ) {
        out << "-Inf";
    } else {
        out << i.getMin();
    }
    out << ", ";
    if ( i.getMax() == std::numeric_limits<double>::max() ) {
        out << "+Inf";
    } else {
        out << i.getMax();
    }
    out << "]";
    return out;
}
} // end PVTP namespace

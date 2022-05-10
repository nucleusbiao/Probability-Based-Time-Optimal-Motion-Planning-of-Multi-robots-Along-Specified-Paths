#include <cmath>
#include <limits>
#include <PVTP/Maths.hpp>
#include <PVTP/Constraints.hpp>

namespace PVTP {

Constraints::Constraints ( Constraints& c, bool mirrored ) {
    if ( mirrored ) {
        this->init( c.getXLimit(), c.getTLimit(), c.getVMin(), c.getVMax(), -c.getAMax(), -c.getAMin(), c.getEpsilon() );
    } else {
        this->init( c.getXLimit(), c.getTLimit(), c.getVMin(), c.getVMax(), c.getAMin(), c.getAMax(), c.getEpsilon() );
    }

    // init increments epsilon by machine epsilon; undo that, since this is copy constructor
    this->setEpsilon( c.getEpsilon() );
}
Constraints::Constraints ( ){}
Constraints::Constraints ( double x_limit, double t_limit, double v_min, double v_max, double a_min, double a_max, double epsilon ) {
    if ( epsilon < 0. ) {
        std::cerr << "epsilon: " << epsilon << ". ";
        throw 4;
    }
    if ( !((a_min < -epsilon) && (a_max > epsilon)) ) {
        std::cerr << "a_min: " << a_min << " a_max: " << a_max << ". ";
        throw 0;
    }
    if ( v_min > v_max ) {
        std::cerr << "v_min: " << v_min << " v_max: " << v_max << ". ";
        throw 1;
    }
    if ( x_limit < epsilon ) {
        std::cerr << "x_limit: " << x_limit << ". ";
        throw 2;
    }
    if ( t_limit <= epsilon ) {
        std::cerr << "t_limit: " << t_limit << ". ";
        throw 3;
    }
    this->init( x_limit, t_limit, v_min, v_max, a_min, a_max, epsilon );
}

void Constraints::initSet(const Constraints& c) {

    this->init( c.getXLimit(), c.getTLimit(), c.getVMin(), c.getVMax(), c.getAMin(), c.getAMax(), c.getEpsilon() );
    // init increments epsilon by machine epsilon; undo that, since this is copy constructor
    this->setEpsilon( c.getEpsilon() );
}
void Constraints::init( double x_limit, double t_limit, double v_min, double v_max, double a_min, double a_max, double epsilon ) {
    this->x_limit = x_limit;
    this->t_limit = t_limit;
    this->v_max = v_max;
    this->v_min = v_min;
    this->a_max = a_max;
    this->a_min = a_min;

    // Pad epsilon with the machine-dependent rounding error
    epsilon += std::numeric_limits<double>::epsilon();

    this->epsilon = epsilon;
}

bool Constraints::validX( double x ) const {
    return Maths::approxGe( x, 0., this->getEpsilon() )
            && Maths::approxLe( x, this->getXLimit(), this->getEpsilon() );
}

bool Constraints::validT( double t ) const {
    return Maths::approxGe( t, 0., this->getEpsilon() )
            && Maths::approxLe( t, this->getTLimit(), this->getEpsilon() );

}

bool Constraints::validV( double v ) const {
    return Maths::approxGe( v, this->getVMin(), this->getEpsilon() )
            && Maths::approxLe( v, this->getVMax(), this->getEpsilon() );
}

bool Constraints::validA( double a ) const {
    return Maths::approxGe( a, this->getAMin(), this->getEpsilon() )
            && Maths::approxLe( a, this->getAMax(), this->getEpsilon() );
}

double Constraints::getXLimit() const {
    return this->x_limit;
}

double Constraints::getTLimit() const {
    return this->t_limit;
}

double Constraints::getVMax() const {
    return this->v_max;
}

double Constraints::getVMin() const {
    return this->v_min;
}

double Constraints::getAMax() const {
    return this->a_max;
}

double Constraints::getAMin() const {
    return this->a_min;
}

double Constraints::getEpsilon() const {
    return this->epsilon;
}

void Constraints::setXLimit( double x_limit ) {
    this->x_limit = x_limit;
}

void Constraints::setTLimit( double t_limit ) {
    this->t_limit = t_limit;
}

void Constraints::setVMax( double v_max ) {
    this->v_max = v_max;
}

void Constraints::setVMin( double v_min ) {
    this->v_min = v_min;
}

void Constraints::setAMax( double a_max ) {
    this->a_max = a_max;
}

void Constraints::setAMin( double a_min ) {
    this->a_min = a_min;
}

void Constraints::setEpsilon( double epsilon ) {
    this->epsilon = epsilon;
}

void Constraints::exceptionMessage( int e ) {
    std::cerr << "Constraint construction failed: ";
    switch ( e ) {
    case 0:
        std::cerr << "a_min must be < 0, a_max must be > 0. " << std::endl;
        break;
    case 1:
        std::cerr << "v_min must be < v_max." << std::endl;
        break;
    case 2:
        std::cerr << "x_limit must be > 0." << std::endl;
        break;
    case 3:
        std::cerr << "t_limit must be > 0." << std::endl;
        break;
    case 4:
        std::cerr << "epsilon must be >= 0." << std::endl;
        break;
    }
    std::cout << std::endl;
}

std::ostream& operator<<(std::ostream& out, const Constraints& c) {
    return out << "x limit: " << c.getXLimit() << ", t limit: " << c.getTLimit() << ", [v_min, v_max]: [" << c.getVMin() << ", " << c.getVMax() << "], [a_min, a_max]: [" << c.getAMin() << ", " << c.getAMax() << "]" << ", epsilon: " << c.getEpsilon();
}

} // end PVTP namespace

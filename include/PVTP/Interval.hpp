#ifndef PVTP_INTERVAL_H
#define PVTP_INTERVAL_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <PVTP/Maths.hpp>
#include <PVTP/Constraints.hpp>

namespace PVTP {
	
	/**
	 * Interval class. This class is for contructing and handling intervals and
	 * interval operations. Operations typically take a constraints parameter
	 * in order to perform fuzzy comparisons.
	 */
	class Interval {
		
		/**
		 * Overload the output operator
		 */
		friend std::ostream& operator<<(std::ostream& out, const Interval& i);
		
	public:
		
		/**
		 * Interval constructor: Empty interval
		 */
        Interval();
		/**
		 * Copy constructor
		 */
		Interval ( Interval& i );

		/**
		 * Interval contructor: Specified interval
		 */
		Interval ( double min, double max );

        void initSet( double min, double max);
        void initSet( const Interval& i);
		/**
		 * Set the bounds of this interval. This assignment fails if the
		 * bounds are invalid.
		 */
		bool setBounds( double min, double max );
		
		/**
		 * Whether this interval contains a given real.
		 */
		bool contains( double x, Constraints& c ) const;
		
		/**
		 * Intersect this interval with another interval.
		 */
		void intersect( Interval& B, Constraints& c );
		
		/**
		 * Intersect an interval A with another interval B, store in result.
		 *
		 * The resulting interval gets A's constraint set.
		 */
		static void intersect( Interval& result, Interval& A, Interval& B, Constraints& c );
		
		/**
		 * Intersect an interval with a set of intervals
		 */
		static void intersect( std::vector<Interval*>& result,
							  Interval& A,
							  std::vector<Interval*>& B,
							  Constraints& c );
		
		/**
		 * Intersect a set of disjoint intervals with a set of disjoing intervals
		 */
		static void intersect( std::vector<Interval*>& result,
							  std::vector<Interval*>& A,
							  std::vector<Interval*>& B,
							  Constraints& c );
		
		/**
		 * Union an interval A with another interval B, store in result.
		 * If A and B are disjoint, the result shall be a disjoint range. For
		 * this reason, the result is a vector of two intervals. If the result
		 * is not disjoing, the second interval is set to empty.
		 *
		 * The resulting interval gets A's constraint set.
		 */
		static void specialUnion( std::pair<Interval*, Interval*>& result,
								 Interval& A,
								 Interval& B,
								 Constraints& c );
		
		/**
		 * Merge a set of unions such that the result is a disjoing union of
		 * intervals where the resulting disjoing union contains exactly those
		 * elements that appeared in at least one of the original intervals.
		 *
		 * The resulting intervals get intervals.front()'s constraint set.
		 */
		static void specialUnionOfIntervals( std::vector<Interval*>& result,
											std::vector<Interval*>& intervals );
		
		/**
		 * Determine whether the interval A is a subset of a given interval B
		 */
		bool isSubset( Interval& B, Constraints& c ) const;
		
		/**
		 * Test given bounds for validity.
		 */
		bool boundsValid( double min, double max );
		
		/**
		 * Set whether this interval is empty or not
		 */
		void setEmpty( bool empty );
		
		/**
		 * Determine whether the interval A is epsilon equal to a given interval B
		 */
		bool isEqual( Interval& B, Constraints& c ) const;
		
		/**
		 * Whether this interval is empty
		 */
		bool isEmpty() const;
		
		/**
		 * Accessor for min bound
		 */
		double getMin() const;
		
		/**
		 * Accessor for max bound
		 */
		double getMax() const;
		
		/**
		 * Get midpoint of interval
		 */
		double getMidPoint() const;
		
		/**
		 * Initialize an interval object.
		 */
		void init( double min, double max );
	
	private:
		
		/**
		 * The interval minimum
		 */
		double min;
		
		/**
		 * The interval maximum
		 */
		double max;
		
		/**
		 * Whether this interval is empty.
		 */
		bool empty;
	};
	
	/**
	 * A function object used as a comparator in specialUnionOfIntervals.
	 *
	 * Sort order: Descending.
	 */
	struct IntervalComparator {
		bool operator() (std::pair<double, int> A, std::pair<double, int> B) {
			if ( A.first > B.first ) {
				return true;
			} else if ( A.first == B.first ) {
				if ( A.second > B.second ) {
					return true;
				}
			}
			return false;
		}
	};

} // end PVTP namespace

#endif

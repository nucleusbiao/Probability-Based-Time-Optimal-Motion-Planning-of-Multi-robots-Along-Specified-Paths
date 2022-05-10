#ifndef PVTP_MATHS_H
#define PVTP_MATHS_H

#include <math.h>
#include <limits>
#include <algorithm>

namespace PVTP {
	
	/**
	 * This namespace contains math functions specific to the library.
	 * All operations are performed in PT space.
	 * All functions in this namespace take the epsilon parameter as their last
	 * parameter. The epsilon parameter is used when fuzzy comparisons must be
	 * made.
	 */
	namespace Maths {
		
		/**
		 * Test for NaN. According to IEEE standards, comparisons involving
		 * NaN will always evaluate to false. Therefore, NaN will be the only
		 * value that fails a test for whether it is equal to itself. This
		 * routine exploits that by using the != operator.
		 */
		bool isNaN( double num );
		
		/**
		 * Test whether two floating point numbers are equal within
		 * some pre-defined epsilon: A ~= B.
		 *
		 * This comparison forms the basis for the other approximate
		 * comparisons. It is assumed that the fp implementation uses round
		 * to nearest for number representation. Epsilon is padded with the
		 * machine-dependent rounding error before being compared with the
		 * difference between A and B.
		 *
		 * This ensures that [0, epsilon_padded] contains the intended epsilon,
		 * which ensures that approxEq will always return true when expected to.
		 * 
		 * However, under this scheme it is possible that true will be returned
		 * when false is expected. For that error to occur on a 64-bit machine,
		 * it must be that:
		 *
		 *			epsilon < abs(A-B) < epsilon + 1.5*(2.22045e-16)
		 * 
		 */
		bool approxEq( double A, double B, double epsilon );
		
		/**
		 * Test whether two floating point numbers are not equal within
		 * some pre-defined epsilon: A ~!= B
		 */
		bool approxNe( double A, double B, double epsilon );
		
		/**
		 * Test whether two floating point numbers are less-than within
		 * some pre-defined epsilon: A ~< B
		 */
		bool approxLt( double A, double B, double epsilon );
		
		/**
		 * Test whether two floating point numbers are less-than-or-equal
		 * within some pre-defined epsilon: A ~<= B
		 */
		bool approxLe( double A, double B, double epsilon );
		
		/**
		 * Test whether two floating point numbers are greater-than within
		 * some pre-defined epsilon: A ~> B
		 */
		bool approxGt( double A, double B, double epsilon );
		
		/**
		 * Test whether two floating point numbers are greater-than-or-equal
		 * within some pre-defined epsilon: A ~>= B
		 */
		bool approxGe( double A, double B, double epsilon );
		
		/**
		 * Take numbers very close to zero and round them to zero.
		 */
		double clipToZero( double num, double range );
		
		/**
		 * Given two 2D points, determine slope going from p1 -> p2
		 */
		double slope( std::pair<double, double> p1, std::pair<double, double> p2, double epsilon );
		
		/**
		 * Given coordinates for two 2D points, determine slope going from p1 -> p2
		 */
		double slope( double x1, double t1, double x2, double t2, double epsilon );
		
		/**
		 * Calculate average velocity given initial and final position, and
		 * initial and final time.
		 */
		double avgVelFromX1_X2_T1_T2( double p1, double p2, double t1, double t2, double epsilon );
		
		/**
		 * Calculate the distance coordinate at which a parabola inflects given
		 * the first derivative, v, of your current point, and the second
		 * derivative, a, of the parabola.
		 */
		double parabolaDelX_FromV1_V2_A( double v1, double v2, double acc, double epsilon );
		
		/**
		 * Similar to above, but here calculate an initial first derivative, v1,
		 * that results in a first derivative, v2, over some x, delta_x, given a
		 * second derivative a.
		 */
		double parabolaV1_FromV2_A( double v2, double delta_x, double acc, double epsilon );
		
		/**
		 * Solve a quadratic. By convention, the first root is preferred, in
		 * descending order of priority, to be minimum if both are positive,
		 * positive, or real.
		 */
		void quadratic( std::pair<double, double>& roots,
					   double a,
					   double b,
					   double c,
					   double epsilon );

		/**
		 * Identical to above, but roots returned with strict ordering:
		 * first = -b + sqrt(disc), second = -b - sqrt(disc)
		 */
		void quadraticOrdered( std::pair<double, double>& roots,
							  double a,
							  double b,
							  double c,
							  double epsilon );
		
		/**
		 * Equation of motion. Get initial velocity from change in position,
		 * change in time, and accelerations.
		 */
		double motionV1_FromX1_X2_T1_T2_A( double x1,
										  double x2,
										  double t1,
										  double t2,
										  double acc,
										  double epsilon );
		
		/**
		 * Equation of motion. Get change in position from initial velocity,
		 * change in time, and acceleration.
		 */
		double motionX_FromV1_T1_T2_A( double v1, double t1, double t2, double acc, double epsilon );

		/**
		 * Equation of motion. Get acceleration from initial velocity,
		 * change in time, and change in position.
		 */
		double motionA_FromV1_X1_X2_T1_T2( double v1,
										  double x1,
										  double x2,
										  double t1,
										  double t2,
										  double epsilon );
		
		/**
		 * Equation of motion. Get time from initial velocity, change in position,
		 * and acceleration.
		 */
		double motionT_FromV1_X1_X2_A( double v1, double x1, double x2, double acc, double epsilon );
		
		/**
		 * Equation of motion. Get distance from initial and final velocities, acceleration, and time.
		 */
		double motionX_FromV1_V2_T1_T2_A( double v1, double v2, double t1, double t2, double acc, double epsilon );
		
		/**
		 * Final velocity given initial velocity, time, and acceleration
		 */
		double V2_FromV1_T_A( double v1, double t, double acc, double epsilon );

		/**
		 * Acceleration given rinal velocity given initial velocity, and time
		 */
		double A_FromV1_V2_T1_T2( double v1, double v2, double t1, double t2, double epsilon );
		
		/**
		 * Change in time given initial velocity, final velocity, and acceleration
		 */
		double T_FromV1_V2_A( double v1, double v2, double acc, double epsilon );
		
		/**
		 * Change in time given initial velocity, final velocity, and change in
		 * position.
		 */
		double T_FromV1_V2_X1_X2( double v1, double v2, double x1, double x2, double epsilon );
		
		/**
		 * Find time coordinate, t_tangent, of tangent point of line to parabola
		 * that passes through an external point (x2, t2) given that external point,
		 * initial first derivative, v1, and second derivative, acc.
		 */
		double parabolaT_TangentFromV1_X2_T2_A( double v1,
											   double x2,
											   double t2,
											   double acc,
											   double epsilon );
		
		/**
		 * Find tangent points of line tangent to two parabolas, where one
		 * is centered at the origin, and the other at (x_star, t_star).
		 */
		double parabolasTangentLine( std::pair< std::pair<double, double>,
									std::pair<double, double> >& points,
									double v1,
									double a1,
									double a2,
									double x_star,
									double t_star,
									double epsilon );
		
		/**
		 * Find tangent points of line tangent to two parabolas, where one
		 * is centered at the origin, and the intersects (x2, t2) with velocity
		 * v_tang
		 *
		 * Function returns the slope of the tangent line; calculating the slope
		 * after the fact from the points risks introducing significant floating point
		 * discretization errors.
		 */
		void parabolasTangentLine( std::pair< std::pair<double, double>,
								  std::pair<double, double> >& points,
								  double v1,
								  double v_tang,
								  double a1,
								  double a2,
								  double x2,
								  double t2,
								  double epsilon );
		
		/**
		 * Find the offset from the origin of the point on the parabola specified
		 * by second derivative acc that has first derivative v. In 'origin', the
		 * first member is offset in path, second is offset in time.
		 */
		bool parabolaOriginOffset( std::pair<double, double>& origin,
								  double v,
								  double acc,
								  double epsilon );
		
		/**
		 * Calculate Euclidean distance between two ordered pairs
		 */
		double euclideanDistance( std::pair<double, double>& p1,
								 std::pair<double, double>& p2,
								 double epsilon );
		
		/**
		 * Calculate the approximate quarter arc length of an ellipse using
		 * the Cantrell-Ramanujan approximation:
		 *
		 * http://www.ebyte.it/library/docs/math05a/EllipsePerimeterApprox05.html#C
		 *
		 * This method assumes a and b are positive.
		 */
		double approxEllipsePerimeter( double a, double b, double epsilon );
		
		/**
		 * Calculate the arc length of an ellipse between two point on the 
		 * semi-major axis. Assume that these points are contained within 
		 * the same quadrant.
		 *
		 * http://mathworld.wolfram.com/Ellipse.html
		 *
		 * This method assumes the ellipse is centered at the origin, that the
		 * arc is contained entirely within a single quadrant, and that x2 >= x1.
		 *
		 * This method uses GSL to compute the incomplete elliptical integral,
		 * and uses a fast approximation that is accurate to 5 x 10^-4
		 */
		double ellipseArcLength( double x1, double x2, double a, double b, double epsilon );
		
	} // end Maths namespace
	
} // end PVTP namespace

#endif
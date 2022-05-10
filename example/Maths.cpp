#include <iostream>
#include <stdio.h>
#include <PVTP/Maths.hpp>
#include <PVTP/Constants.hpp>

#define _USE_MATH_DEFINES

namespace PVTP {
	
	namespace Maths {
		
		bool isNaN( double num ) {
			return num != num;
		}
	
		bool approxEq( double A, double B, double epsilon ) {

			// To ensure consistency, always subtract smaller from bigger
			double diff;
			if ( A > B ) {
				diff = A - B;
			} else {
				diff = B - A;
			}
			
			return diff <= epsilon;
		}
		
		bool approxNe( double A, double B, double epsilon ) {
			return !approxEq( A, B, epsilon );
		}
		
		bool approxLt( double A, double B, double epsilon ) {
			if ( approxEq(A, B, epsilon) ) {
				return false;
			}
			return A < B;
		}
		
		bool approxLe( double A, double B, double epsilon ) {
			if ( approxEq(A, B, epsilon) ) {
				return true;
			}
			return A < B;
		}
		
		bool approxGt( double A, double B, double epsilon ) {
			if ( approxEq(A, B, epsilon) ) {
				return false;
			}
			return A > B;
		}
		
		bool approxGe( double A, double B, double epsilon ) {
			if ( approxEq(A, B, epsilon) ) {
				return true;
			}
			return A > B;
		}
		
		double clipToZero( double num, double range ) {
			if ( fabs(num) < range ) {
				return 0.;
			}
			return num;
		}
		
		double slope( std::pair<double, double> p1, std::pair<double, double> p2, double epsilon ) {
			return slope( p1.first, p1.second, p2.first, p2.second, epsilon );
		}
		
		double slope( double x1, double y1, double x2, double y2, double epsilon ) {
			double delta_x = x2 - x1;
			double delta_y = y2 - y1;
			
			if ( delta_x == 0. ) {
				return 0.;
			}
			if ( delta_y == 0. ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::slope: denominator 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return delta_x / delta_y;
		}
		
		double avgVelFromX1_X2_T1_T2( double x1, double x2, double t1, double t2, double epsilon) {
			double delta_t = t2 - t1;
			if ( delta_t == 0 ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::avgVelFromX1_X2_T1_T2: delta_t = 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return (x2 - x1) / delta_t;
		}
		
		double parabolaDelX_FromV1_V2_A( double v1, double v2, double acc, double epsilon ) {
			if ( acc == 0 ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::parabolaInflectionPoint: a = 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return -0.5 * ((v2*v2 - v1*v1) / acc);
		}
		
		double parabolaV1_FromV2_A( double v2, double delta_x, double acc, double epsilon ) {
			double discriminant = v2 * v2 - 2 * acc * delta_x;
			if ( discriminant < 0. ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::parabolaV1_FromV2_A: discriminant < 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return sqrt( discriminant );
		}
		
		void quadratic( std::pair<double, double>& roots, double a, double b, double c, double epsilon ) {
			quadraticOrdered( roots, a, b, c, epsilon );
			double x1 = roots.first;
			double x2 = roots.second;

			// by convention, if both are positive, first < second
			if ( !signbit(x1) && !signbit(x2) ) {
				if ( x1 > x2 ) {
					roots.first = x2;
					roots.second = x1;
				} else {
					roots.first = x1;
					roots.second = x2;
				}
				return;
			}
			
			// by convention, positive or real root first
			if ( !signbit(x1) || isNaN(x2) ) {
				roots.first = x1;
				roots.second = x2;
				return;
			}
			
			// otherwise, if some mix of negatives or NaN, order unimportant
			roots.first = x2;
			roots.second = x1;
		}
		
		void quadraticOrdered( std::pair<double, double>& roots, double a, double b, double c, double epsilon ) {
			
			// trivial case
			if ( (a == 0.) && (b == 0) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::quadratic: coeffients are zero" << std::endl;
#endif
				roots.first = std::numeric_limits<double>::quiet_NaN();
				roots.second = std::numeric_limits<double>::quiet_NaN();
				return;
			}
			
			double q, x1, x2;
			
			// linear case
			if ( a == 0. ) {
				x1 = -c / b;
				x2 = x1;
				roots.first = x2;
				roots.second = x1;
				return;
			}
			
			// sort of linear case
			if ( c == 0. ) {
				if ( b < 0. ) {
					roots.second = 0.;
					roots.first = -b / a;
				} else {
					roots.first = 0.;
					roots.second = -b / a;
				}
				return;
			}
			
			// simple squared case
			double discriminant;
			if ( b == 0. ) {
				discriminant = -a * c;
				if ( discriminant < 0. ) {
					discriminant = clipToZero( -c / a, Constants::CLIP_TO_ZERO );
					if ( discriminant < 0. ) {
#ifdef SHOW_COMMENTS
						std::cout << "WARNING IN Maths::quadratic: discriminant < 0 (1)" << std::endl;
#endif
						roots.first = std::numeric_limits<double>::quiet_NaN();
						roots.second = std::numeric_limits<double>::quiet_NaN();
						return;
					}
				}
				x1 = sqrt(discriminant) / a;
				x2 = -x1;
				roots.first = x1;
				roots.second = x2;
				return;
			}
			
			// quadratic case
			discriminant = b * b - 4 * a * c;
			if ( discriminant < 0. ) {
				discriminant = clipToZero( discriminant, Constants::CLIP_TO_ZERO );
				if ( discriminant < 0. ) {
#ifdef SHOW_COMMENTS
					std::cout << "WARNING IN Maths::quadratic: discriminant = " << discriminant << " < 0 (2)" << std::endl;
#endif
					roots.first = std::numeric_limits<double>::quiet_NaN();
					roots.second = std::numeric_limits<double>::quiet_NaN();
					return;
				}
			}
			
			// UNIQUE SOLUTION
			if ( discriminant == 0. ) {
				x1 = -b / (2 * a);
				x2 = x1;
				roots.first = x2;
				roots.second = x1;
				return;
			}
			
			// NON-UNIQUE SOLUTION
			q = -0.5 * (b + ((b<0.)?-1:1) * sqrt(discriminant));
			
			if ( b < 0. ) {
				roots.first = q / a;
				roots.second = c / q;
			} else {
				roots.first = c / q;
				roots.second = q / a;
			}
			return;
		}
		
		double motionV1_FromX1_X2_T1_T2_A( double x1, double x2, double t1, double t2, double acc, double epsilon ) {
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			if ( delta_t == 0. ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::motionV_FromX1_X2_T1_T2_A: delta_t == 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return (delta_x / delta_t) - 0.5 * acc * delta_t;
		}
		
		double motionX_FromV1_T1_T2_A( double v1, double t1, double t2, double acc, double epsilon ) {
			double delta_t = t2 - t1;
			return delta_t * (v1 + 0.5 * acc * delta_t);
		}
		
		double motionX_FromV1_V2_T1_T2_A( double v1, double v2, double t1, double t2, double acc, double epsilon ) {
			double delta_t = t2 - t1;
			if ( Maths::approxEq(delta_t, 0., epsilon) ) {
				return 0;
			}
			return 0.5 * (v2 + v1) * delta_t;
		}
		
		double motionA_FromV1_X1_X2_T1_T2( double v1, double x1, double x2, double t1, double t2, double epsilon ) {
			double delta_x = x2 - x1;
			double delta_t = t2 - t1;
			return 2 * (delta_x - v1 * delta_t) / (delta_t * delta_t);
		}
		
		double motionT_FromV1_X1_X2_A( double v1, double x1, double x2, double acc, double epsilon ) {
			std::pair<double, double> roots;
			quadratic( roots, acc, 2 * v1, -2 * (x2 - x1), epsilon );
			return roots.first;
		}
		
		double V2_FromV1_T_A( double v1, double t, double acc, double epsilon ) {
			return v1 + acc * t;
		}
		
		double A_FromV1_V2_T1_T2( double v1, double v2, double t1, double t2, double epsilon ) {
			double delta_v = clipToZero( v2 - v1, Constants::CLIP_TO_ZERO );
			double delta_t = clipToZero( t2 - t1, Constants::CLIP_TO_ZERO );
			
			if ( delta_t == 0 ) {
				if ( delta_v != 0 ) {
#ifdef SHOW_COMMENTS
					std::cout << "WARNING IN Maths::A_FromV1_V2_T1_T2: delta_t = 0, delta_v != 0" << std::endl;
#endif
					return std::numeric_limits<double>::quiet_NaN();
				}
				return 0;
			}
			
			return delta_v / delta_t;
		}
		
		double T_FromV1_V2_A( double v1, double v2, double acc, double epsilon ) {
			if ( acc == 0. ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::T_FromV1_V2_A: acc = 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return (v2 - v1) / acc;
		}
		
		double T_FromV1_V2_X1_X2( double v1, double v2, double x1, double x2, double epsilon ) {
			double v_sum = v1 + v2;
			if ( v_sum == 0. ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::T_FromV1_V2_X1_X2: v1 + v2 = 0" << std::endl;
#endif
				return std::numeric_limits<double>::quiet_NaN();
			}
			return 2 * (x2 - x1) / v_sum;
		}
		
		double parabolaT_TangentFromV1_X2_T2_A( double v1, double x2, double t2, double acc, double epsilon ) {
			std::pair<double, double> roots;
			quadratic( roots, acc, -2 * acc * t2, 2 * (x2 - v1 * t2), epsilon );
			if ( Maths::isNaN(roots.first) && Maths::isNaN(roots.second) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::parabolaT_TangentFromV1_X2_T2_A: no solution to quadratic" << std::endl;
#endif
				return false;
			}
			return roots.first;
		}
		
		double parabolasTangentLine( std::pair< std::pair<double, double>, std::pair<double, double> >& points, double v1, double a1, double a2, double x_star, double t_star, double epsilon ) {		
			std::pair<double, double> roots;
			double q_a = 1 - (a2 / a1);
			double q_b = 2 * a2 * t_star + 2 * a2 / a1 * v1;
			double q_c = -(a2 / a1 * v1 * v1 + 2 * a2 * x_star);
			quadraticOrdered( roots, q_a, q_b, q_c, epsilon );

			if ( Maths::isNaN(roots.first) && Maths::isNaN(roots.second) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::parabolasTangentLine: no solution to quadratic" << std::endl;
#endif
				points.first.first = std::numeric_limits<double>::quiet_NaN();
				points.first.second = std::numeric_limits<double>::quiet_NaN();
				points.second.first = std::numeric_limits<double>::quiet_NaN();
				points.second.second = std::numeric_limits<double>::quiet_NaN();
				return std::numeric_limits<double>::quiet_NaN();
			}

			// slope of tangent line, for out purposes, a negative slope is unusable
			double v;
			if ( a2 < a1 ) {
				v = roots.second;
			} else {
				v = roots.first;
			}
			
			// x-intercept
			double v_diff = v1 - v;
			
			// find the first point
			double t1 = (-2 * v_diff) / (2 * a1);
			double x1 = motionX_FromV1_T1_T2_A( v1, 0., t1, a1, epsilon );
			points.first.first = x1;
			points.first.second = t1;
			
			// find second point
			double t2 = 2 * (a2 * t_star + v) / (2 * a2);
			double x2 = x_star + motionX_FromV1_T1_T2_A( 0., 0., t_star-t2, a2, epsilon );
			points.second.first = x2;
			points.second.second = t2;
			
			return v;
		}
		
		void parabolasTangentLine( std::pair< std::pair<double, double>, std::pair<double, double> >& points, double v1, double v_tang, double a1, double a2, double x2, double t2, double epsilon ) {
			
			// find intersection point of line to first parabola
			double t_i = T_FromV1_V2_A( v1, v_tang, a1, epsilon );
			double x_i = motionX_FromV1_T1_T2_A( v1, 0., t_i, a1, epsilon );
			points.first.first = x_i;
			points.first.second = t_i;

			// find x intercept of tangent line
			double x_int = x_i - v_tang * t_i;
			
			// find offset from origin of second parabola where the line is tangent
			double t_o = T_FromV1_V2_A( 0., v_tang, a2, epsilon );
			double x_o = motionX_FromV1_T1_T2_A( 0., 0., t_o, a2, epsilon );
			
			// find (x_star, t_star), which is the origin of the second parabola
			double q_a = 1.;
			double q_b = 2 * ((v_tang / a2) - t2);
			double q_c = t2 * t2 - t_o * t_o + 2 * (v_tang * t_o + x_int - x2) / a2;
			std::pair<double, double> roots;
			quadraticOrdered( roots, q_a, q_b, q_c, epsilon );
			if ( Maths::isNaN(roots.first) && Maths::isNaN(roots.second) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::parabolasTangentLine: no solution to quadratic" << std::endl;
#endif
				points.first.first = std::numeric_limits<double>::quiet_NaN();
				points.first.second = std::numeric_limits<double>::quiet_NaN();
				points.second.first = std::numeric_limits<double>::quiet_NaN();
				points.second.second = std::numeric_limits<double>::quiet_NaN();
				return;
			}
			
			double t_star;
			t_star = roots.second;
			double x_star = x2 - motionX_FromV1_T1_T2_A( 0., 0., t_star - t2, a2, epsilon );
			
			// find intersection point of line to second parabola
			double x_prime = x_star + x_o;
			double t_prime = t_star + t_o;
			points.second.first = x_prime;
			points.second.second = t_prime;
		}
		
		bool parabolaOriginOffset( std::pair<double, double>& origin, double v, double acc, double epsilon ) {
			if ( acc == 0. ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::OriginFinalP_Plus: acceleration = 0, no inflection point." << std::endl;
#endif
				origin.first = std::numeric_limits<double>::quiet_NaN();
				origin.second = origin.first;
				return false;
			}
			origin.first = -parabolaDelX_FromV1_V2_A( v, 0., acc, epsilon );
			if ( isNaN(origin.first) ) {
#ifdef SHOW_COMMENTS
				std::cout << "WARNING IN Maths::OriginFinalP_Plus: parabolaDelX_FromV1_V2_A failed." << std::endl;
#endif
				origin.first = std::numeric_limits<double>::quiet_NaN();
				origin.second = origin.first;
				return false;
			}
			origin.second = -v / acc;
			return true;
		}
		
		double euclideanDistance( std::pair<double, double>& p1,
								 std::pair<double, double>& p2,
								 double epsilon ) {
			
			double x1 = p1.first;
			double y1 = p1.second;
			double x2 = p2.first;
			double y2 = p2.second;
			
			bool x_equal = Maths::approxEq( x1, x2, epsilon );
			bool y_equal = Maths::approxEq( y1, y2, epsilon );
			
			if ( x_equal && y_equal ) {
				return 0.0;
			}
			
			double y_diff = y2 - y1;
			
			if ( x_equal ) {
				return fabs( y_diff );
			}
			
			double x_diff = x2 - x1;
			
			if ( y_equal ) {
				return fabs( x_diff );
			}
			
			return sqrt( x_diff * x_diff + y_diff * y_diff );
			
		}
		
		double approxEllipsePerimeter( double a, double b, double epsilon ) {
			
			double h_root = (a - b) / (a + b);
			double h = h_root * h_root;
			double h_times_3 = 3 * h;

			return M_PI * (a + b) * (1 + (h_times_3 / (10 + sqrt(4 - h_times_3))) + ((4 / M_PI) - (14 / 11)) * pow(h, 12));
			
		}
		
#ifdef BUILDGSL
#if BUILDGSL
		double ellipseArcLength( double x1, double x2, double a, double b, double epsilon ) {
			
			// get coordinates on ellipse of arc end points
			double ratio = x1 / a;
			double y1 = b * sqrt( 1 - ratio * ratio );
			ratio = x2 / a;
			double y2 = b * sqrt( 1 - ratio * ratio );
			
			// calculate angles for endoints
			double phi_1 = atan( y1 / x1 );
			double phi_2 = atan( y2 / x2 );
			
			// ellipse modulus (eccentricity)
			double k = sqrt( 1 - ((b * b) / (a * a)) );
			
			// calculate lengths
			double arc_1 = gsl_sf_ellint_E( phi_1, k, Constants::GSL_MODE );
			double arc_2 = gsl_sf_ellint_E( phi_2, k, Constants::GSL_MODE );
			
			return a * ( arc_2 - arc_1 );
		}
#endif
#endif
		
	} // end Maths namespace
	
} // end PVTP namespace

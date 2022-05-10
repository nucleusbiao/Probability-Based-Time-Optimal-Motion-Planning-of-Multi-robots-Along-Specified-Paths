#ifndef PVTP_CONSTANTS_H
#define PVTP_CONSTANTS_H

#ifdef BUILDGSL
#if BUILDGSL
#include <gsl/gsl_sf_ellint.h>
#endif
#endif

namespace PVTP {
	
	/**
	 * This namespace contains constants specific to the library.
	 */
	namespace Constants {
		
		/**
		 * Used by specialUnionOfIntervals to mark a number as an interval min
		 */
		static const int INTERVAL_MIN_MARKER = -1;
		
		/**
		 * Used by specialUnionOfIntervals to mark a number an an interval max
		 */
		static const int INTERVAL_MAX_MARKER = 1;
		
		/**
		 * Used by PVT_ObstaclePoint to indicate that a trajectory must pass
		 * above a vertex
		 */
		static const char H_CLASS_ABOVE = 1;

		/**
		 * Used by PVT_ObstaclePoint to indicate that a trajectory must pass
		 * below a vertex
		 */
		static const char H_CLASS_BELOW = 0;
		
		/**
		 * The origin needs a homotopic classification; by convention, set it here.
		 */
		static const char H_CLASS_ORIGIN = H_CLASS_BELOW;
		
		/**
		 * A default value for epsilon: I've found this value to be reasonably
		 * good in practice; making the value too small exacerbates problems due
		 * to discretization error, and making it too big makes it more likely
		 * that correctness assumptions that the algorithm relies on for proper
		 * operation are violated.
		 */
		static const double DEFAULT_EPSILON = 0.00000001;
		
		/**
		 * Zero clipping value
		 */
		static const double CLIP_TO_ZERO = DEFAULT_EPSILON;
		
		/**
		 * For debugging purposes a really high output precision is needed,
		 * I've found this value to be good
		 */
		static const double DEBUGGING_OUTPUT_PRECISION = 30;
		
#ifdef BUILDGSL
#if BUILDGSL
		/**
		 * Accuracy mode for GSL functions
		 */
		static const gsl_mode_t GSL_MODE = GSL_PREC_APPROX;
#endif
#endif

		/**
		 * For trajectories to be able to be executed, the durations of their
		 * segments must be integer multiples of a given time step. This constant
		 * defines a minimum remainder a segment must have to be considered 
		 * a multiple.
		 */
		static const double DURATION_SAFE_REMAINDER = 0.01;
		
	} // end Constants namespace

} // end PVTP namespace

#endif
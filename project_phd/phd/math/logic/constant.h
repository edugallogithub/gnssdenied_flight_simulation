#ifndef MATH_CONSTANT_H
#define MATH_CONSTANT_H

#include "../math.h"
#include "logic.h"
#include <limits>
#include <cmath>

namespace math {

// CONSTANT
// ========
// ========

class MATH_API constant {
private:
    /**< number PI */
    static const double _PI;
    /**< number PI/2 */
    static const double _PIHALF;
    /**< infinite */
    static const double _INF;
    /**< epsilon */
    static const double _EPS;
    /**< conversion from degrees to radians */
    static const double _D2R;
    /**< conversion from radians to degrees */
    static const double _R2D;
    /**< default maximum number of iterations */
    static const unsigned short _NUM_ITER;
    /**< tolerance [isu units] for differences between numbers */
	static const double _TOL;
    /**< tolerance [isu units] for differences between numbers in equispaced tables. */
	static const double _DIFF;
    /**< minimum rotation in exponential map (from rotation vector to quaternion) below
     * which a truncated formula shall be employed, being faster and safer. Value comes
     * from Tso3:test_exp_log_small. */
    static const double _SMALL_ROT;
    /**< default tolerance [rad] for geodesic computations */
    static const double _GEODESIC_TOL;
    /**< default longitude and latitude tolerance [rad] to establish if two points are the same */
    static const double _SAME_POINT_TOL;
public:
    /**< returns number PI */
    static const double& PI()				{return _PI;}
    /**< returns number PI / 2 */
    static const double& PIHALF()			{return _PIHALF;}
    /**< returns infinite */
    static const double& INF()				{return _INF;}
    /**< returns epsilon (machine floating point precision) */
    static const double& EPS()				{return _EPS;}
    /**< returns conversion from degrees to radians */
    static const double& D2R()				{ return _D2R; }
    /**< returns conversion from radians to degrees */
    static const double& R2D()				{ return _R2D; }
    /**< returns default maximum number of iterations */
    static const unsigned short& NUM_ITER() {return _NUM_ITER;}
    /**< returns tolerance [isu units] for differences between numbers */
	static const double& TOL()			    {return _TOL;}
    /**< returns tolerance [isu units] for differences between numbers in equispaced tables. */
	static const double& DIFF()			    {return _DIFF;}
    /**< returns minimum rotation in exponential map (from rotation vector to quaternion) below
     * which a truncated formula shall be employed, being faster and safer. Value comes
     * from Tso3:test_exp_log_small. */
    static const double & SMALL_ROT()   {return _SMALL_ROT;}
    /**< return default tolerance [rad] for geodesic computations */
    static const double& GEODESIC_TOL()    {return _GEODESIC_TOL;}
    /**< return default longitude and latitude tolerance [rad] to establish if two points are the same */
    static const double& SAME_POINT_TOL()   {return _SAME_POINT_TOL;}

}; // closes class constant

} // closes namespace math

#endif



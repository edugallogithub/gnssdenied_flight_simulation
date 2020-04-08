#include "constant.h"

// CONSTANT
// ========
// ========

const double math::constant::_PI  = acos(-1.);
/* number PI */

const double math::constant::_PIHALF  = acos(-1.) / 2.0;
/* number PI / 2 */

const double math::constant::_INF = std::numeric_limits<double>::infinity();
/* infinite */

const double math::constant::_EPS = std::numeric_limits<double>::epsilon();
/* epsilon (machine floating point precision) */

const double math::constant::_D2R = acos(-1.) / 180.;
/* conversion from degrees to radians */

const double math::constant::_R2D = 180. / acos(-1.);
/* conversion from radians to degrees */

const unsigned short math::constant::_NUM_ITER = 100;
/* default maximum number of iterations */

const double math::constant::_TOL = 1e-8;
/* tolerance [isu units] for differences between numbers */

const double math::constant::_DIFF = 1e-12;
/* tolerance [isu units] for differences between numbers in equispaced tables */

const double math::constant::_SMALL_ROT = 1e-14;
/* minimum rotation in exponential map (from rotation vector to quaternion) below
 * which a truncated formula shall be employed, being faster and safer. Value of 3e-8 comes
 * from Tso3:test_exp_log_small. I set it much lower. */

const double math::constant::_GEODESIC_TOL = 1e-12;
/* default tolerance [rad] for geodesic computations */

const double math::constant::_SAME_POINT_TOL = 1e-10;
/* default longitude and latitude tolerance [rad] to establish if two points are the same */

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////








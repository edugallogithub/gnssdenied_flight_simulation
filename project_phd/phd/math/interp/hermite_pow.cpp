#include "hermite_pow.h"

// CLASS HERMITE_POW
// =================
// =================

math::hermite_pow::hermite_pow(const double& point0,
								 const double& point1,
								 const double& value0,
								 const double& value1,
								 const double& slope0,
								 const double& slope1)
: _coeffs(4) {
	double point_diff = point1 - point0;
	double value_diff = value1 - value0;
	_coeffs[0] = value0;
	_coeffs[1] = slope0;
	_coeffs[2] = 3. * value_diff / pow(point_diff, 2.) 
				- 2. * slope0 / point_diff - slope1 / point_diff;
	_coeffs[3] = - 2. * value_diff / pow(point_diff, 3.) 
				+ (slope0 + slope1) / pow(point_diff, 2.);
}
/* constructor based on the two consecutive input values, their corres
ponding outpus, and the slopes at both points */

math::hermite_pow::hermite_pow(double coeff0,
								 double coeff1,
								 double coeff2, 
								 double coeff3)
: _coeffs(4) {
	_coeffs[0] = coeff0;
	_coeffs[1] = coeff1;
	_coeffs[2] = coeff2;
	_coeffs[3] = coeff3;
}
/* private constructor based on the four coefficients, only 
employed by the arithmetical operators */

math::hermite_pow::hermite_pow()
: _coeffs(4, 0.) {
}
/* empty constructor that sets the four coefficients to 0 */

math::hermite_pow::hermite_pow(const hermite_pow& op2)
: _coeffs(op2._coeffs) {
}
/* copy constructor */

math::hermite_pow& math::hermite_pow::operator=(const hermite_pow& op2) {
	_coeffs = op2._coeffs;
	return *this;
}
/* overloaded operator = (assignment) */

math::hermite_pow::~hermite_pow(){
}
/**< destructor */

const math::hermite_pow math::hermite_pow::operator+(const hermite_pow& op2) const {
	return hermite_pow(_coeffs[0] + op2._coeffs[0],
					   _coeffs[1] + op2._coeffs[1],
					   _coeffs[2] + op2._coeffs[2],
					   _coeffs[3] + op2._coeffs[3]);
}
const math::hermite_pow math::hermite_pow::operator-(const hermite_pow& op2) const {
	return hermite_pow(_coeffs[0] - op2._coeffs[0],
					   _coeffs[1] - op2._coeffs[1],
					   _coeffs[2] - op2._coeffs[2],
					   _coeffs[3] - op2._coeffs[3]);
}
const math::hermite_pow math::hermite_pow::operator*(const hermite_pow& op2) const {
	return hermite_pow(_coeffs[0] * op2._coeffs[0],
					   _coeffs[1] * op2._coeffs[1],
					   _coeffs[2] * op2._coeffs[2],
					   _coeffs[3] * op2._coeffs[3]);
}
const math::hermite_pow math::hermite_pow::operator/(const hermite_pow& op2) const {
	return hermite_pow(_coeffs[0] / op2._coeffs[0],
					   _coeffs[1] / op2._coeffs[1],
					   _coeffs[2] / op2._coeffs[2],
					   _coeffs[3] / op2._coeffs[3]);
}
const math::hermite_pow math::hermite_pow::operator*(const double& op2) const {
	return hermite_pow(_coeffs[0] * op2,
					   _coeffs[1] * op2,
					   _coeffs[2] * op2,
					   _coeffs[3] * op2);
}

const math::hermite_pow math::hermite_pow::operator/(const double& op2) const {
	return hermite_pow(_coeffs[0] / op2,
					   _coeffs[1] / op2,
					   _coeffs[2] / op2,
					   _coeffs[3] / op2);
}
/* arithmetical operators */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE_POW2
// ==================
// ==================

math::hermite_pow2::hermite_pow2(const double& point0,
								   const double& point1,
								   const math::hermite_pow& herm0,
								   const math::hermite_pow& herm1,
								   const math::hermite_pow& slope0,
								   const math::hermite_pow& slope1)
: _coeffs(4) {
	double point_diff = point1 - point0;
	hermite_pow value_diff = herm1 - herm0;
	_coeffs[0] = herm0;
	_coeffs[1] = slope0;
	_coeffs[2] = value_diff * 3. / pow(point_diff, 2.) 
					- slope0 * 2. / point_diff
					- slope1 / point_diff;	
	_coeffs[3] = (slope0 + slope1) / pow(point_diff, 2.)
					- value_diff * 2. / pow(point_diff, 3.);

}
/* constructor based on the two consecutive input values, their corres
ponding outputs (Hermite coefficients), and the differentials at both
points (also Hermite coefficients) */

math::hermite_pow2::hermite_pow2(const hermite_pow& coeff0,
								   const hermite_pow& coeff1,
								   const hermite_pow& coeff2,
								   const hermite_pow& coeff3)
: _coeffs(4) {
	_coeffs[0] = coeff0;
	_coeffs[1] = coeff1;
	_coeffs[2] = coeff2;
	_coeffs[3] = coeff3;								   								   
}
/* private constructor based on the four coefficients, only 
employed by the arithmetical operators */

math::hermite_pow2::hermite_pow2()
: _coeffs(4) {
}
/* empty constructor that sets the 16 coefficients to 0 */

math::hermite_pow2::hermite_pow2(const hermite_pow2& op2)
: _coeffs(op2._coeffs) {
}
/* copy constructor */

math::hermite_pow2& math::hermite_pow2::operator=(const hermite_pow2& op2) {
	_coeffs = op2._coeffs;
	return *this;
}
/* overloaded operator = (assignment) */

math::hermite_pow2::~hermite_pow2() {
}
/**< destructor */

const math::hermite_pow2 math::hermite_pow2::operator+(const hermite_pow2& op2) const {
	return hermite_pow2(_coeffs[0] + op2._coeffs[0],
					   _coeffs[1] + op2._coeffs[1],
					   _coeffs[2] + op2._coeffs[2],
					   _coeffs[3] + op2._coeffs[3]);
}
const math::hermite_pow2 math::hermite_pow2::operator-(const hermite_pow2& op2) const {
	return hermite_pow2(_coeffs[0] - op2._coeffs[0],
					   _coeffs[1] - op2._coeffs[1],
					   _coeffs[2] - op2._coeffs[2],
					   _coeffs[3] - op2._coeffs[3]);
}
const math::hermite_pow2 math::hermite_pow2::operator*(const hermite_pow2& op2) const {
	return hermite_pow2(_coeffs[0] * op2._coeffs[0],
					   _coeffs[1] * op2._coeffs[1],
					   _coeffs[2] * op2._coeffs[2],
					   _coeffs[3] * op2._coeffs[3]);
}
const math::hermite_pow2 math::hermite_pow2::operator/(const hermite_pow2& op2) const {
	return hermite_pow2(_coeffs[0] / op2._coeffs[0],
					   _coeffs[1] / op2._coeffs[1],
					   _coeffs[2] / op2._coeffs[2],
					   _coeffs[3] / op2._coeffs[3]);
}
const math::hermite_pow2 math::hermite_pow2::operator*(const double& op2) const {
	return hermite_pow2(_coeffs[0] * op2,
					   _coeffs[1] * op2,
					   _coeffs[2] * op2,
					   _coeffs[3] * op2);
}

const math::hermite_pow2 math::hermite_pow2::operator/(const double& op2) const {
	return hermite_pow2(_coeffs[0] / op2,
					   _coeffs[1] / op2,
					   _coeffs[2] / op2,
					   _coeffs[3] / op2);
}
/* arithmetical operators */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE_POW3
// ==================
// ==================

math::hermite_pow3::hermite_pow3(const double& point0,
								   const double& point1,
								   const math::hermite_pow2& herm0,
								   const math::hermite_pow2& herm1,
								   const math::hermite_pow2& slope0,
								   const math::hermite_pow2& slope1)
: _coeffs(4) {
	double point_diff = point1 - point0;
	hermite_pow2 value_diff = herm1 - herm0;
	_coeffs[0] = herm0;
	_coeffs[1] = slope0;
	_coeffs[2] = value_diff * 3. / pow(point_diff, 2.) 
					- slope0 * 2. / point_diff
					- slope1 / point_diff;	
	_coeffs[3] = (slope0 + slope1) / pow(point_diff, 2.)
					- value_diff * 2. / pow(point_diff, 3.);

}
/* constructor based on the two consecutive input values, their corres
ponding outputs (Hermite coefficients), and the differentials at both
points (also Hermite coefficients) */

math::hermite_pow3::hermite_pow3(const hermite_pow2& coeff0,
								   const hermite_pow2& coeff1,
								   const hermite_pow2& coeff2,
								   const hermite_pow2& coeff3)
: _coeffs(4) {
	_coeffs[0] = coeff0;
	_coeffs[1] = coeff1;
	_coeffs[2] = coeff2;
	_coeffs[3] = coeff3;								   								   
}
/* private constructor based on the four coefficients, only 
employed by the arithmetical operators */

math::hermite_pow3::hermite_pow3()
: _coeffs(4) {
}
/* empty constructor that sets the 64 coefficients to 0 */

math::hermite_pow3::hermite_pow3(const hermite_pow3& op2)
: _coeffs(op2._coeffs) {
}
/* copy constructor */

math::hermite_pow3& math::hermite_pow3::operator=(const hermite_pow3& op2) {
	_coeffs = op2._coeffs;
	return *this;
}
/* overloaded operator = (assignment) */

math::hermite_pow3::~hermite_pow3() {
}
/**< destructor */

const math::hermite_pow3 math::hermite_pow3::operator+(const hermite_pow3& op2) const {
	return hermite_pow3(_coeffs[0] + op2._coeffs[0],
					   _coeffs[1] + op2._coeffs[1],
					   _coeffs[2] + op2._coeffs[2],
					   _coeffs[3] + op2._coeffs[3]);
}
const math::hermite_pow3 math::hermite_pow3::operator-(const hermite_pow3& op2) const {
	return hermite_pow3(_coeffs[0] - op2._coeffs[0],
					   _coeffs[1] - op2._coeffs[1],
					   _coeffs[2] - op2._coeffs[2],
					   _coeffs[3] - op2._coeffs[3]);
}
const math::hermite_pow3 math::hermite_pow3::operator*(const hermite_pow3& op2) const {
	return hermite_pow3(_coeffs[0] * op2._coeffs[0],
					   _coeffs[1] * op2._coeffs[1],
					   _coeffs[2] * op2._coeffs[2],
					   _coeffs[3] * op2._coeffs[3]);
}
const math::hermite_pow3 math::hermite_pow3::operator/(const hermite_pow3& op2) const {
	return hermite_pow3(_coeffs[0] / op2._coeffs[0],
					   _coeffs[1] / op2._coeffs[1],
					   _coeffs[2] / op2._coeffs[2],
					   _coeffs[3] / op2._coeffs[3]);
}
const math::hermite_pow3 math::hermite_pow3::operator*(const double& op2) const {
	return hermite_pow3(_coeffs[0] * op2,
					   _coeffs[1] * op2,
					   _coeffs[2] * op2,
					   _coeffs[3] * op2);
}

const math::hermite_pow3 math::hermite_pow3::operator/(const double& op2) const {
	return hermite_pow3(_coeffs[0] / op2,
					   _coeffs[1] / op2,
					   _coeffs[2] / op2,
					   _coeffs[3] / op2);
}
/* arithmetical operators */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE_POW4
// ==================
// ==================

math::hermite_pow4::hermite_pow4(const double& point0,
								   const double& point1,
								   const math::hermite_pow3& herm0,
								   const math::hermite_pow3& herm1,
								   const math::hermite_pow3& slope0,
								   const math::hermite_pow3& slope1)
: _coeffs(4) {
	double point_diff = point1 - point0;
	hermite_pow3 value_diff = herm1 - herm0;
	_coeffs[0] = herm0;
	_coeffs[1] = slope0;
	_coeffs[2] = value_diff * 3. / pow(point_diff, 2.) 
					- slope0 * 2. / point_diff
					- slope1 / point_diff;	
	_coeffs[3] = (slope0 + slope1) / pow(point_diff, 2.)
					- value_diff * 2. / pow(point_diff, 3.);

}
/* constructor based on the two consecutive input values, their corres
ponding outputs (Hermite coefficients), and the differentials at both
points (also Hermite coefficients) */

math::hermite_pow4::hermite_pow4(const hermite_pow3& coeff0,
								   const hermite_pow3& coeff1,
								   const hermite_pow3& coeff2,
								   const hermite_pow3& coeff3)
: _coeffs(4) {
	_coeffs[0] = coeff0;
	_coeffs[1] = coeff1;
	_coeffs[2] = coeff2;
	_coeffs[3] = coeff3;								   								   
}
/* private constructor based on the four coefficients, only 
employed by the arithmetical operators */

math::hermite_pow4::hermite_pow4()
: _coeffs(4) {
}
/* empty constructor that sets the 256 coefficients to 0 */

math::hermite_pow4::hermite_pow4(const hermite_pow4& op2)
: _coeffs(op2._coeffs) {
}
/* copy constructor */

math::hermite_pow4& math::hermite_pow4::operator=(const hermite_pow4& op2) {
	_coeffs = op2._coeffs;
	return *this;
}
/* overloaded operator = (assignment) */

math::hermite_pow4::~hermite_pow4() {
}
/**< destructor */

const math::hermite_pow4 math::hermite_pow4::operator+(const hermite_pow4& op2) const {
	return hermite_pow4(_coeffs[0] + op2._coeffs[0],
					   _coeffs[1] + op2._coeffs[1],
					   _coeffs[2] + op2._coeffs[2],
					   _coeffs[3] + op2._coeffs[3]);
}
const math::hermite_pow4 math::hermite_pow4::operator-(const hermite_pow4& op2) const {
	return hermite_pow4(_coeffs[0] - op2._coeffs[0],
					   _coeffs[1] - op2._coeffs[1],
					   _coeffs[2] - op2._coeffs[2],
					   _coeffs[3] - op2._coeffs[3]);
}
const math::hermite_pow4 math::hermite_pow4::operator*(const hermite_pow4& op2) const {
	return hermite_pow4(_coeffs[0] * op2._coeffs[0],
					   _coeffs[1] * op2._coeffs[1],
					   _coeffs[2] * op2._coeffs[2],
					   _coeffs[3] * op2._coeffs[3]);
}
const math::hermite_pow4 math::hermite_pow4::operator/(const hermite_pow4& op2) const {
	return hermite_pow4(_coeffs[0] / op2._coeffs[0],
					   _coeffs[1] / op2._coeffs[1],
					   _coeffs[2] / op2._coeffs[2],
					   _coeffs[3] / op2._coeffs[3]);
}
const math::hermite_pow4 math::hermite_pow4::operator*(const double& op2) const {
	return hermite_pow4(_coeffs[0] * op2,
					   _coeffs[1] * op2,
					   _coeffs[2] * op2,
					   _coeffs[3] * op2);
}

const math::hermite_pow4 math::hermite_pow4::operator/(const double& op2) const {
	return hermite_pow4(_coeffs[0] / op2,
					   _coeffs[1] / op2,
					   _coeffs[2] / op2,
					   _coeffs[3] / op2);
}
/* arithmetical operators */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////














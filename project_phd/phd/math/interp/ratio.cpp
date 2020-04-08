#include "ratio.h"

// CLASS RATIO
// ===========
// ===========

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_LINEAR
// ==================
// ==================

math::ratio_linear::ratio_linear()
: _ratio12(0.) {}
/* empty constructor */

math::ratio_linear::ratio_linear(const ratio_linear& op2)
: _ratio12(op2._ratio12){}
/* copy constructor */

math::ratio_linear& math::ratio_linear::operator=(const ratio_linear& op2) {
	_ratio12 = op2._ratio12;	
	return *this;
}
/* overloaded operator = (assignment) */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_QUADRATIC
// =====================
// =====================

math::ratio_quadratic::ratio_quadratic()
: _ratio12(0.), _ratio23(0.), _ratio13(0.) {}
/* empty constructor */

math::ratio_quadratic::ratio_quadratic(const ratio_quadratic& op2)
: _ratio12(op2._ratio12), _ratio23(op2._ratio23), _ratio13(op2._ratio13) {}
/* empty constructor */

math::ratio_quadratic& math::ratio_quadratic::operator=(const ratio_quadratic& op2) {
	_ratio12 = op2._ratio12;	
	_ratio23 = op2._ratio23;	
	_ratio13 = op2._ratio13;	
	return *this;
}
/* overloaded operator = (assignment) */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_CUBIC
// =================
// =================

math::ratio_cubic::ratio_cubic()
: _ratio12(0.), _ratio23(0.), _ratio34(0.), _ratio13(0.), _ratio24(0.), _ratio14(0.) {}
/* empty constructor */

math::ratio_cubic::ratio_cubic(const ratio_cubic& op2)
: _ratio12(op2._ratio12), _ratio23(op2._ratio23), _ratio34(op2._ratio34),
_ratio13(op2._ratio13), _ratio24(op2._ratio24), _ratio14(op2._ratio14) {}
/* empty constructor */

math::ratio_cubic& math::ratio_cubic::operator=(const ratio_cubic& op2) {
	_ratio12 = op2._ratio12;	
	_ratio23 = op2._ratio23;	
	_ratio34 = op2._ratio34;	
	_ratio13 = op2._ratio13;	
	_ratio24 = op2._ratio24;	
	_ratio14 = op2._ratio14;
	return *this;
}
/* overloaded operator = (assignment) */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_HERMITE
// ===================
// ===================

math::ratio_hermite::ratio_hermite()
: _ratio12(0.) {}
/* empty constructor */

math::ratio_hermite::ratio_hermite(const ratio_hermite& op2)
: _ratio12(op2._ratio12){}
/* copy constructor */

math::ratio_hermite& math::ratio_hermite::operator=(const ratio_hermite& op2) {
	_ratio12 = op2._ratio12;	
	return *this;
}
/* overloaded operator = (assignment) */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_SPLINE
// ==================
// ==================

math::ratio_spline::ratio_spline()
: _ratio12(0.) {}
/* empty constructor */

math::ratio_spline::ratio_spline(const ratio_spline& op2)
: _ratio12(op2._ratio12){}
/* copy constructor */

math::ratio_spline& math::ratio_spline::operator=(const ratio_spline& op2) {
	_ratio12 = op2._ratio12;	
	return *this;
}
/* overloaded operator = (assignment) */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




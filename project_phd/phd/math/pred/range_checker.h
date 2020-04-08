#ifndef MATH_RANGE_CHECKER_H
#define MATH_RANGE_CHECKER_H

#include "../math.h"
#include "../vec/vec1.h"

/*
This file contains the functor in charge of verifying whether a given input
magnitude fits within a given range. It is used by table functions to
check if obtaining the result requires extrapolation or interpolation.

Two options are present: active (verifies the range) and inactive (no action).
*/

namespace math {

// CLASS RANGE_CHECKER
// ===================
// ===================

class MATH_API range_checker {
public:
	virtual void check_range(const vec1&, const double&) const = 0;
	/**< checks if the input magnitude is within the range defined by the input
	vector of magnitudes. If not, returns an exception. */
	virtual range_checker* clone() const = 0;
	/**< cloner */
}; // closes class range_checker

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RANGE_CHECKER_INACTIVE
// ============================
// ============================

class MATH_API range_checker_inactive : public range_checker {
	void check_range(const vec1&, const double&) const {}
	/**< checks if the input magnitude is within the range defined by the input
	vector of magnitudes. If not, returns an exception. */
	range_checker_inactive* clone() const;
	/**< cloner */
}; // closes class range_checker_inactive

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RANGE_CHECKER_ACTIVE
// ==========================
// ==========================

class MATH_API range_checker_active : public range_checker {
	void check_range(const vec1&, const double&) const;
	/**< checks if the input magnitude is within the range defined by the input
	vector of magnitudes. If not, returns an exception. */
	range_checker_active* clone() const;
	/**< cloner */
}; // closes class range_checker_active

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

} // closes namespace math

#endif



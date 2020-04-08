#include "range_checker.h"

// CLASS RANGE_CHECKER_INACTIVE
// ============================
// ============================

math::range_checker_inactive* math::range_checker_inactive::clone() const {
	return new math::range_checker_inactive();
}
/* cloner */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RANGE_CHECKER_ACTIVE
// ==========================
// ==========================

void math::range_checker_active::check_range(const vec1& Ovec1,
												const double& o) const {
	bool flag = false;
    if (math::check_range(Ovec1.front(), Ovec1.back(), o, flag) == false) {
        throw std::runtime_error("Out of range.");
	}
}
/* checks if the input magnitude is within the range defined by the input
vector of magnitudes. If not, returns an exception. */

math::range_checker_active* math::range_checker_active::clone() const {
	return new math::range_checker_active();
}
/* cloner */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////








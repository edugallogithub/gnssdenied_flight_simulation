#ifndef MATH_PRED_H
#define MATH_PRED_H

#include "../math.h"
#include "../vec/classifiers.h"
#include "range_checker.h"
#include "pos_finder.h"
#include "../interp/interp_lagrange_third.h"
#include "../interp/interp_lagrange_second.h"
#include "../interp/interp_lagrange_first.h"

/*
This file contains the base class for all predicates.
*/

namespace math {

// CLASS PRED
// ==========
// ==========

class MATH_API pred {
public:
	virtual const logic::PRED_NAME& get_name() const = 0;
	/**< returns the specific predicate name */
	virtual const std::string& get_st_name() const = 0;
	/**< returns string containing the predicate name */
	virtual pred* clone() const = 0;
	/**< cloner */
	virtual ~pred() {}
    /**< destructor */
	static const std::string describe_pred(const math::logic::PRED_NAME index);
	/**< returns a string containing the name of the predicate in the
	PRED_NAME enumeration */
	static const math::logic::PRED_NAME reverse_describe_pred(const std::string&);
	/**< returns the name of the predicate in the PRED_NAME enumeration based on a 
	string describing it */
}; // closes class pred

} // closes namespace math

#endif




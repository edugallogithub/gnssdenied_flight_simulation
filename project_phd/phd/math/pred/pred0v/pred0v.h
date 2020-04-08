#ifndef MATH_PRED0V_H
#define MATH_PRED0V_H

#include "../../math.h"
#include "../pred.h"

/*
This file contains the base class pred0v for those function predicates that do
not have any independent magnitudes. The derivate classes are in other files
in this same folder. It is structured as a virtual base class (pred0v) with its
derived implementations: f_null (returning 0) and f_constant (returning a
constant value f0).

No matter the inserted output units, they are immediately converted to standard
ones. When employing the value method, it is already assummed that the units of
the output magnitude are set OK (standard units), as this function does not
make any unit checks.

They all share the methods value() and differential().
Their results are:
- f_null		value() = 0		d_dt() = 0
- f_constant	value() = _f0	d_dt() = 0

SLOW FUNCTIONS:
- None

ACCURACY:
- No method presents any degradation in accuracy

MODE OF USE:
Employ the constructors to create instances of both classes:
- f_null pred creates an object predicate of class f_null, without any
  input parameters.
- f_constant pred(f0) creates an object predicate of class f_constant
  with f0 as its parameter, expressed in standard units.
  f_constant pred(f0, unit) shall be used when f0 is in a different unit.
The functions value() and d_dt() can then be directly used, although it
is more common for these objects to be the first input parameters of the fun
constructors.
*/

namespace math {

// CLASS PRED0V
// ============
// ============

class MATH_API pred0v : public pred {
private:


public:
	virtual double value() const = 0;
	/**< evaluates the function at any point. */
	virtual double d_dt() const = 0;
	/**< evaluates the function differential with time at any point. */
	virtual const logic::PRED_NAME& get_name() const = 0;
	/**< see virtual function of class pred above */
	virtual const std::string& get_st_name() const = 0;
	/**< see virtual function of class pred above */
	virtual bool operator==(const pred0v& op2) const = 0;
	/**< overloaded operator == (equal) */
	inline virtual bool operator!=(const pred0v& op2) const	{return !(*this == op2);}
	/**< overloaded operator != (not equal) */
	virtual ~pred0v() {}
    /**< destructor */

}; // closes class pred0v

} // closes namespace math

#endif




#ifndef MATH_PRED2V_H
#define MATH_PRED2V_H

#include "../../math.h"
#include "../pred.h"

/*
This file contains the base class pred1v for those function predicates that have
two independent magnitude2. The derivate classes are in other files in this
same folder. It is structured as a virtual base class (pred2v) with
its derived implementations.
- f_lineal_double containing a polynomial of grade 1 in both variables.
- f_table2V containing (n) 1st input magnitudes, (m) 2nd input magnitudes,
  and (nxm) output magnitudes representing the function value at each point.
  Different interpolation methods allowed..
- f_tabular2V is a variation of f_table2V that contains a single vector
  of input magnitudes plus a vector of unidimensional tables, representing
  the possible values at each input. Different interpolation methods allowed.

Two important issues shall be noted. First, the f_lineal_double function
is entirely numeric, while the rest provide dependencies between input magnitudes
and output magnitudes. This causes no differences when employed by the user.

Second, for the entirely numeric function (f_lineal_double), no matter the
inserted input and output units, they are immediately converted to standard
ones. For the others, they contain by definition standard units as the magnitudes
can not be otherwise. When employing the value method, it is already assummed that
the units of the output magnitude are set OK (standard units), as this function
does not make any unit checks.

They all share the methods value(x,y) and d_dt(x, y, dx_dt, dy_dt).
Their results are:
- f_lineal_double
	value(x,y)				= _f0 + _f1x * x + _f1y * y + _f1xy * x *y
	d_dt(x,y,dx_dt,dy_dt)	= (_f1x + _f1xy * y) * dx_dt +
							  (_f1y + _f1xy * x) * dy_dt
	Valid for any values of x and y.
- f_table2V
	value(x,y)				= employs linear interpolation between points.
	d_dt(x,y,dx_dt,dy_dt)	= d_dx * dx_dt + d_dy * dy_dt, where d_dx and
							  d_dy are the partial differentials computed
							  by linear interpolation
	Valid for any value of x and y ((extrapolates if outside from below,
	takes closest value if outside from above)
- f_tabular2V similar to f_table2V

SLOW FUNCTIONS:
- Determining the position within the _points1 and _points2 vectors of the
  input values is expensive for f_table2V, as two complete searches are required.
  It is much cheaper for f_table2Veq, as the points are equispaced. Nothing
  can be done to accelerate this.
- The position is determined independently when calling the value and d_dt
  methods, which is redundant. It may be possible to modify the interface
  so the search is only executed once if both methods are called.

ACCURACY:
- No method presents any degradation in accuracy
- The position is determined independently when calling the value and d_dt
  methods, which is redundant. It may be possible to modify the interface
  so the search is only executed once if both methods are called.
- The way of computing the differentials as constant steps between two inputs
  is prone to big errors, as is the case with the GPM32. Much better to 
  evaluate the table at two or three close points and come up with a continuous
  response. DO NOT DELETE THIS, CORRECT THE CODE.

MODE OF USE:
Employ the constructors to create instances of the classes:
- f_lineal_double pred(f0,f1x,f1y,f1xy) creates an object predicate of
  class f_lineal_double, with f0, f1x, f1y, and f1xy as parameters,
  expressed in standard units.
  f_lineal_double pred(f0,f1x,f1y,f1xy,output_unit,input_unit1,input_unit2)
  shall be used when f0, f1x, f1y, and f1xy correspond to different input
  and/or output units.
- f_table2V predicate(points1,points2,Values) creates an object predicate of
  class f_table2V, with points1 being an n size increasing order vector of
  independent magnitudes, points2 being a different m size increasing order
  vector of independent magnitudes, and Values an n size vector of
  size m vectors with the resulting magnitudes at each point. Linear
  interpolation is employed between points.

The functions value() and d_dt() can then be directly used, although it is more
common for these objects to be the first input parameters of the fun constructors.

SEE ALSO: f_table1V
*/

namespace math {

// CLASS PRED2V
// ============
// ============

class MATH_API pred2v : public pred {
public:
	virtual double value(const double& input1, const double& input2) const = 0;
	/**< evaluates the function at the reference magnitudes input1 and input2,
	and writes the result at the reference magnitude result. Only the magnitude value
	is inserted into result, the units are assummed to be OK. */
	virtual double d_dt(const double& input1, const double& input2,
					    const double& input1_dt, const double& input2_dt) const = 0;
	/**< evaluates the function differential with time at the reference
	magnitudes input1 and input2 and their differentials with time input1_dt
	and input2_dt, and writes the result at the reference magnitude result. */
	virtual const logic::PRED_NAME& get_name() const = 0;
	/**< see virtual function of class pred above */
	virtual const std::string& get_st_name() const = 0;
	/**< see virtual function of class pred above */
	virtual ~pred2v() {}
    /**< destructor */
	virtual bool operator==(const pred2v& op2) const = 0;
	/**< overloaded operator == (equal) */
	inline virtual bool operator!=(const pred2v& op2) const
	{return !(*this == op2);}
	/**< overloaded operator != (not equal) */

}; // closes class pred2v

} // closes namespace math

#endif




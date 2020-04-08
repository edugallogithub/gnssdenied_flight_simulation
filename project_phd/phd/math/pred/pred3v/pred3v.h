#ifndef MATH_PRED3V_H
#define MATH_PRED3V_H

#include "../../math.h"
#include "../pred.h"

/*
This file contains the base class pred1v for those function predicates that have
three independent magnitude2. The derivate classes are in other files in this
same folder.It is structured as a virtual base class (pred3v) with
its derived implementations.
- f_lineal_triple containing a polynomial of grade 1 in the three variables.
- f_table3V containing (l) 1st input magnitudes, (m) 2nd input magnitudes,
  (n) 3rd input magnitudes, and (lxmxn) output magnitudes representing the
  function value at each point. Different interpolation methods allowed.
- f_tabular3V is a variation of f_table3V that contains a single vector
  of input magnitudes plus a vector of bidimensional tables, representing
  the possible values at each input. Different interpolation methods allowed.
- f_supertabular3V is other variation of f_table3V that contains a single vector
  of input magnitudes plus a vector of tables of type f_tabular2V, representing
  the possible values at each input. Different interpolation methods allowed.

Two important issues shall be noted. First, the f_lineal_triple function
is entirely numeric, while the rest provide dependencies between input magnitudes
and output magnitudes. This causes no differences when employed by the user.

Second, for the entirely numeric function (f_lineal_triple), no matter the
inserted input and output units, they are immediately converted to standard
ones. For the others, they contain by definition standard units as the magnitudes
can not be otherwise. When employing the value method, it is already assummed that
the units of the output magnitude are set OK (standard units), as this function
does not make any unit checks.

They all share the methods value(x,y,z) and d_dt(x,y,z,dx_dt,dy_dt,dz_dt).
Their results are:
- f_lineal_triple
	value(x,y,z) = _f0 + _f1x * x + _f1y * y + _f1z * z +  _f1xy * x * y +
			_f1xz * x * z +	_f1yz * y * z + _f1xyz * x * y * z
	d_dt(x,y,z,dx_dt,dy_dt,dz_dt) =
			(_f1x + _f1xy * y + _f1xz * z + _f1xyz * y * z) * dx_dt +
			(_f1y + _f1xy * x + _f1yz * z + _f1xyz * x * z) * dy_dt +
			(_f1z + _f1xz * x + _f1yz * y + _f1xyz * x * y) * dz_dt
	Valid for any of x, y, and z.
- f_table3V
	value(x,y,z)				  = employs linear interpolation between points.
	d_dt(x,y,z,dx_dt,dy_dt,dz_dt) = d_dx * dx_dt + d_dy * dy_dt + d_dz * dz_dt,
									where d_dx, d_dy, and d_dz are the partial
									differentials computed by linear interpolation
	Valid for any value of x (which generally represents altitude) but error if
	y or z (which generally represent longitude and latitude) outside intervals
	defined by _points.
- f_tabular3V similar to f_table3V
- f_supertabular3V similar to f_table3V

SLOW FUNCTIONS:
- Determining the position within the _points1, _points2, and _points3 vectors of
  the input values is expensive for f_table3V, as three complete searches are
  required. It is much cheaper for f_table3Veq, as the points are equispaced.
  Nothing can be done to accelerate this.
- The position is determined independently when calling the value and d_dt
  methods, which is redundant. It may be possible to modify the interface
  so the search is only executed once if both methods are called.

ACCURACY:
- No method presents any degradation in accuracy

LEFT:
- The f_table3V and f_tabular3V predicates return an error if called with an input
  outside the predefined interval for their 2nd and 3rd independent variables.
  This may cause problems when iterating, in which the input may go outside the
  normal range.

MODE OF USE:
Employ the constructors to create instances of the classes:
- f_lineal_triple pred(f0,f1x,f1y,f1z,f1xy,f1xz,f1yz,f1xyz) creates an
  object predicate of class f_lineal_triple, with f0, f1x, f1y, f1z,
  f1xy, f1xz, f1yz, and f1xyz as parameters, expressed in standard units.
  f_lineal_triple pred(f0,f1x,f1y,f1z,f1xy,f1xz,f1yz,f1xyz, output_unit,
  input_unit1, input_unit2, input_unit3) shall be used when f0, f1x,
  f1y, f1z, f1xy, f1xz, f1yz, and f1xyz correspond to different input
  and/or output units.
- f_table3V predicate(points1,points2,points3,Values) creates an object
  predicate of class f_table3V, with points1 being an l size increasing order
  vector of independent magnitudes, points2 being a different m size increasing
  order vector of independent magnitudes, points3 being a different n size
  increasing order vector of independent magnitudes, and Values an l size vector
  of m size vectors of size n vectors with the resulting magnitudes at each point.
  Linear interpolation is employed between points.

The functions value() and d_dt() can then be directly used, although it is more
common for these objects to be the first input parameters of the fun constructors.

SEE ALSO: f_tabular2V
*/

namespace math {

// CLASS PRED3V
// ============
// ============

class MATH_API pred3v : public pred{
public:
	virtual double value(const double& input1, const double& input2, const double& input3) const = 0;
	/**< evaluates the function at the reference magnitudes input1, input2, and
	input3, and	writes the result at the reference magnitude result. Only the
	magnitude value is inserted into result, the units are assummed to be OK. */
	virtual double d_dt(const double& input1, const double& input2, const double& input3,
                        const double& input1_dt, const double& input2_dt, const double& input3_dt) const = 0;
	/**< evaluates the function differential with time at the reference
	magnitudes input1, input2, and input3 and their differentials with time
	input1_dt, input2_dt, and input3_dt, and writes the result at the reference
	magnitude result. */
	virtual const logic::PRED_NAME& get_name() const = 0;
	/**< see virtual function of class pred above */
	virtual const std::string& get_st_name() const = 0;
	/**< see virtual function of class pred above */
	virtual ~pred3v() {}
    /**< destructor */
	virtual bool operator==(const pred3v& op2) const = 0;
	/**< overloaded operator == (equal) */
	inline virtual bool operator!=(const pred3v& op2) const
	{return !(*this == op2);}
	/**< overloaded operator != (not equal) */

}; // closes pred3v class

} // closes namespace math

#endif






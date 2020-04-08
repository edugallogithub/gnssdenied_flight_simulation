#ifndef MATH_PRED1V_H
#define MATH_PRED1V_H

#include "../../math.h"
#include "../pred.h"

/*
This file contains the base class pred1v for those function predicates that have
a single independent magnitude. The derivate classes are in other files
in this same folder. It is structured as a virtual base class (pred1v) with its
derived implementations.
- f_lineal containing a polynomial of grade 1
- f_parabolic containing a polynomial of grade 2
- f_steps containing (n+1) magnitudes defining n intervals, and n magnitudes
  representing the function constant value at each interval.
- f_table1V containing (n) input magnitudes and (n) output magnitudes
  representing the function value at each point. Different interpolation
  methods allowed.
- f_cat concatenates other unidimensional predicates, which each one being
  applicable in a given interval.
- f_cat_partial is similar to f_cat, with the difference that each predicate is
  evaluated not with the independent magnitude, but with the independent
  magnitude difference to the beginning of the applicable interval.
- f_table1V_num containing (n) input doubles and (n) output doubles (assummuing
  standard units) representing the function value at each point. Linear 
  interpolation employed. TRY TO AVOID ITS USE, EMPLOY OTHER TABLES.
- f_table1V_spl containing (n) input doubles and (n) output doubles (assummuing
  standard units) representing the function value at each point. Spline 
  interpolation employed. TRY TO AVOID ITS USE, EMPLOY OTHER TABLES.

Two important issues shall be noted. First, the f_lineal and f_parabolic functions
are entirely numeric, while the rest provide dependencies between input magnitudes
and output magnitudes. This causes no differences when employed by the user.

Second, for the entirely numeric functions f_lineal and f_parabolic, no matter the
inserted input and output units, they are immediately converted to standard
ones. For the others, they contain by definition standard units as the magnitudes
can not be otherwise. When employing the value method, it is already assummed that
the units of the output magnitude are set OK (standard units), as this function
does not make any unit checks.

They all share the methods value(x) and d_dt(x, dx_dt).
Their results are:
- f_lineal		value(x)		= _f0 + _f1 * x
				d_dt(x,dx_dt)	= _f1 * dx_dt
				Valid for any value of x.
- f_parabolic	value(x)		= _f0 + _f1 * x + _f2 * x * x
                d_dt(x,dx_dt)	= _f1 * dx_dt + 2 * _f2 * x * dx_dt
				Valid for any value of x.
_ f_steps		value(x)		= depends on the interval
				d_dt(x,dx_dt)	= 0
				Valid for any value of x (if outside, takes closest one)
- f_table1V     value(x)		= employs different interpolation methods between points
				d_dt(x,dx_dt)	based on two points
				Valid for any value of x (extrapolates if outside)
- f_cat			Applies each individual function
				Error if x outside all intervals defined by _points.
- f_cat_partial Applies each individual function
				Error if x outside all intervals defined by _points.
- f_table1V_num value(x)		= employs linear interpolation between points
				d_dt(x,dx_dt)	= 0 by design (ask Marco LaCivita)
				Error if x outside all intervals defined by _points.
- f_table1V_spl value(x)		= employs spline interpolation between points
				d_dt(x,dx_dt)	= 0 by design (ask Marco LaCivita)
				Error if x outside all intervals defined by _points.

SLOW FUNCTIONS:
- Determining the position within the _points vector of the input value is
  expensive for the f_steps, f_table1V, f_cat, and f_cat_partial,
  as a complete search is required. It is much cheaper for f_table1Veq, as the
  points are equispaced. Nothing can be done to accelerate this.
- The position is determined independently when calling the value and d_dt
  methods, which is redundant. It may be possible to modify the interface
  so the search is only executed once if both methods are called.

ACCURACY:
- No method presents any degradation in accuracy

LEFT:
- The f_cat and f_cat_partial predicates return an exception if called with an
  input outside the predefined interval. This may cause problems when iterating,
  in which the input may go outside the normal range.

MODE OF USE:
Employ the constructors to create instances of the classes:
- f_lineal pred(f0,f1) creates an object predicate of class f_lineal, with
  f0 and f1 as parameters, expressed in standard units.
  f_lineal pred(f0,f1,output_unit,input_unit) shall be used when f0 and f1
  correspond to different input and/or output units.
- f_parabolic pred(f0,f1,f2) creates an object predicate of class
  f_parabolic, with f0, f1, and f2 as parameters, expressed in standard
  units.
  f_parabolic pred(f0,f1,f2,output_unit,input_unit) shall be used when f0,
  f1, and  f2 correspond to different input and/or output units.
- f_steps predicate(points,values) creates an object predicate of class
  f_steps, with points being an n+1 size increasing order vector of independent
  variable magnitudes, and values an n size vector of resulting magnitudes in
  the steps resulting from two consecutive points.
- f_table1V predicate(points,values) creates an object predicate of class
  f_table1V, with points being an n size increasing order vector of
  independent magnitudes, and values an n size vector of resulting
  magnitudes at each point. Linear interpolation is employed between points.
- f_cat predicate(points,pred) creates an object predicate of class f_cat,
  with points being a n+1 size vector of independent variable magnitudes, and
  pred an n size vector of pointers to other unidimensional predicates
  providing the function value at each interval.
- f_cat_partial predicate(points,pred) creates an object predicate of class
  f_cat_partial, with points being a n+1 size vector of independent variable
  magnitudes, and pred an n size vector of pointers to other unidimensional
  predicates providing the function value at each interval when evaluated with
  the difference to the begining of each interval.
- f_table1V_num predicate(points,values) creates an object predicate of class
  f_table1V_num, with points being an n size increasing order vector of
  doubles (standard units), and values an n size vector of resulting doubles
  (standard units) at each point. Linear interpolation is employed between points.
- f_table1V_spl predicate(points,values) creates an object predicate of class
  f_table1V_spl, with points being an n size increasing order vector of
  doubles (standard units), and values an n size vector of resulting doubles
  (standard units) at each point. Spline interpolation is employed between points.

The functions value() and d_dt() can then be directly used, although it is more
common for these objects to be the first input parameters of the fun constructors.
*/

namespace math {

// CLASS PRED1V
// ============
// ============

class MATH_API pred1v : public pred {
public:
	virtual double value(const double& input) const = 0;
	/**< evaluates the function at the reference magnitude input, and writes the
	result at the reference magnitude result. Only the magnitude value is
	inserted into result, the units are assummed to be OK. */
	virtual double d_dt(const double& input, const double& input_dt) const = 0;
	/**< evaluates the function differential with time at the reference
	magnitude input and its differential with time input_dt, and writes the
	result at the reference magnitude result. */
	virtual const logic::PRED_NAME& get_name() const = 0;
	/**< see virtual function of class pred above */
	virtual const std::string& get_st_name() const = 0;
	/**< see virtual function of class pred above */
	virtual bool operator==(const pred1v& op2) const = 0;
	/**< overloaded operator == (equal) */
	inline virtual bool operator!=(const pred1v& op2) const
	{return !(*this == op2);}
	/**< overloaded operator != (not equal) */
	virtual ~pred1v() {}
    /**< destructor */
	virtual bool is_identity() const {return false;}
	/**< fun1 functions with f_lineal as predicate, f0 == 0 and f1 ==1, are
	fully equivalent to a magnitude as they	always return the input magnitude,
	but they are difficult to detect from the outside. This method returns 
	false except for that specific case. Only overloaded where it applies. */

	virtual void compute_diff(double& result,
							  const int& pos1, 
							  const double& input1,
							  const double& input1_dt) const
	{throw std::runtime_error("Function not implemented.");}
	/**< function only overloaded for f_table1Vxx derivate classes. Not applicable
	to other derivate classes. */
	virtual void compute_value(double& result,
							   const int& pos1,
							   const math::ratio& ratio1) const
	{throw std::runtime_error("Function not implemented.");}
	/**< function only overloaded for f_table1Vxx derivate classes. Not applicable
	to other derivate classes. */
	virtual int compute_pos1(const double& input1) const
	{throw std::runtime_error("Function not implemented.");}
	/**< function only overloaded for f_table1Vxx derivate classes. Not applicable
	to other derivate classes. */
	virtual math::ratio* compute_ratio1(const double& input1,
											  const int& pos1) const
	{throw std::runtime_error("Function not implemented.");}
	/**< function only overloaded for f_table1Vxx derivate classes. Not applicable
	to other derivate classes. */

}; // closes class pred1v

} // closes namespace math

#endif




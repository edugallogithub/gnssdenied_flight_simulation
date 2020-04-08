#ifndef MATH_F_MATH1V_H
#define MATH_F_MATH1V_H

#include "../../math.h"
#include "pred1v.h"

/*
Derivate class of pred1v containing f_table1V objects. Refer to pred1v
for more info. Contains a one dimensional table.

Extrapolation when outside limits.

LEFT:
- Binary search could be accelerated if input from previous execution
  were added to the input. It would need to be stored however.
- Improve accuracy of differential computation by doing before and after point,
  instead of only after.
- For Hermite interpolation, there exists an explicit expression for the
  differential computation.

SLOW:
- Repetition of binary searches.
- The linear_precompute interpolation method requires precomputing slopes at
  construction time (slower) but the computation of differentials is faster. The
  spline interpolation method requires computing the spline second differentials
  at construction time (slow). Hermite also requires precomputation.
*/

namespace math {

	class table1V_diff; // employed before defined

// CLASS F_TABLE1V
// ===============
// ===============

class MATH_API f_table1V : public pred1v {
private:
    /**< classes that have access to private members */
	friend class table1V_diff_prec;
	friend class table1V_diff_real;

    /**< predicate name */
	static const math::logic::PRED_NAME _name;
    /**< predicate name string */
	static const std::string _st_name;
    /**< pointer to object of class vec1 (ordered vector of size n defining the table inputs). */
	math::vec1* _points1;
    /**< pointer to object of class vec1 (vector of size n containing the table output for each input). */
	math::vec1* _values;
    /**< flag that indicates if the _points and _values vectors shall be deleted
    by the destructor (true) or not (false) */
	bool _del_flag;
    /**< interpolation method name */
	math::logic::INTERP_MODE _interp_mode;
    /**< pointer to interpolation method */
	math::interp* _interp;
    /**< true if _points1 vector equispaced, false otherwise */
	bool _equi1;
    /**< difference between consecutive _points1, 0. if not equispaced */
	double _points1_diff;

    /**< pointer to functor that solves differentials */
	math::table1V_diff* _functor_diff;
    /**< pointer to size n vector of coefficients containing the differentials
    (slopes) at each input point. Only present if _interp_mode ==
    math::logic::lagrange_first_precompute or math::logic::hermite */
	math::vec1* _slopes;
    /**< pointer to Hermite coefficients (if applicable) */
	math::hermite1v* _herm;
    /**< pointer controlling out of range for _points1, inactive by default */
	math::range_checker* _checker1;
    /**< pointer determining position of input with respect to _points1 */
	math::pos_finder* _finder1;

    /**< fills up the _slopes attribute based on _points and _values */
	void fill_up_slopes_lagrange_first_precompute();
	void fill_up_slopes_hermite();

    /**< empty constructor not implemented */
	f_table1V();
    /**< overloaded operator = (assignment) not implemented */
	f_table1V& operator=(const f_table1V& op2);

    /**< initialization for constructors */
	void initialize();
    /**< destructor */
	void destroy();
public:
    /**< constructor based on pointers - deleted by destructor. */
	f_table1V(math::vec1* points,
			  math::vec1* values,
			  math::logic::INTERP_MODE interp_mode = math::logic::lagrange_first_precompute);
    /**< constructor based on references - not deleted by destructor. */
	f_table1V(math::vec1& points,
			  math::vec1& values,
			  math::logic::INTERP_MODE interp_mode = math::logic::lagrange_first_precompute);
    /**< copy constructor */
	f_table1V(const f_table1V&);
    /**< destructor */
	virtual ~f_table1V() {destroy();}

    /**< cloner */
	virtual f_table1V* clone() const;
    /**< overloaded operator == (equal) */
	bool operator==(const pred1v& op2) const;
	bool operator==(const f_table1V& op2) const;
    /**< overloaded operator != (not equal) */
	bool operator!=(const f_table1V& op2) const {return !(*this == op2);}

    /**< get the _points object as constant reference to read */
	const math::vec1& get_points1() const {return *_points1;}
    /**< get the _values object as constant reference to read */
	const math::vec1& get_values() const {return *_values;}
    /**< get predicate name to read */
	const logic::PRED_NAME& get_name() const {return _name;}
    /**< get predicate name to read */
	const std::string& get_st_name() const {return _st_name;}
    /**< get the interpolation mode to read */
	const math::logic::INTERP_MODE& get_interp_mode() const {return _interp_mode;}
    /**< get constant reference to interpolation method */
	const math::interp& get_interp() const {return *_interp;}
    /**< return _slopes vector to read. Only required to take a look at the
    _slopes vector in the unitary tests */
	const math::vec1& get_slopes() const {return *_slopes;}
    /**< returns true if _points1 vector equispaced, false otherwise */
	bool get_equi1() const {return _equi1;}
    /**< returns constant reference to finder object */
	const math::pos_finder& get_finder1() const {return *_finder1;}
    /**< returns difference between consecutive _points1, 0. if not equispaced */
	const double& get_points1_diff() const {return _points1_diff;}
    /**< returns points1 out of range checker */
    const math::range_checker& get_checker1() const {return *_checker1;}

    /**< activates or deactives the out of range verification for _points1, which
    is inactive by default */
	void activate_checker1();
	void deactivate_checker1();

    /**< Returns first position within _points1 vector that shall be employed when
    interpolating to obtain the result corresponding to the input magnitude. */
	int compute_pos1(const double& input1) const;
    /**< Returns ratio of input1 with respect to the two points1 identified by pos1 and pos1+1. */
	math::ratio* compute_ratio1(const double& input1, const int& pos1) const;
    /**< Fills up result magnitude by interpolating based on the input position and ratio */
	double compute_value(const int& pos1, const math::ratio& ratio1) const;
    /**< Fills up result differential based on position and ratio */
	double compute_diff(const int& pos1, const double& input1, const double& input1_dt) const;
    /**< evaluates table at input */
    double value(const double& input) const;
    /**< evaluates table time differential at input */
    double d_dt(const double& input, const double& input_dt) const;
}; // closes class f_table1V

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE1V_DIFF
// ==================
// ==================

class MATH_API table1V_diff {
public:
	virtual void compute_diff(double& result,
							  const int& pos1,
							  const double& input1,
							  const double& input1_dt) const = 0;
	/**< fill up result differential based on position, input, and its partial
	differential with time */
}; // closes class table1V_diff

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE1V_DIFF_PREC
// =======================
// =======================

class MATH_API table1V_diff_prec: public table1V_diff {
private:
	math::f_table1V* _pred;
	/**< weak pointer to f_table1V class */
	table1V_diff_prec();
	/**< empty constructor not implemented */
	table1V_diff_prec(const table1V_diff_prec&);
	/**< copy constructor not implemented */
	table1V_diff_prec& operator=(const table1V_diff_prec& op2);
	/**< overloaded operator = (assignment) not implemented */
public:
	table1V_diff_prec(math::f_table1V& pred);
	/**< constructor based on one dimensional table */
	~table1V_diff_prec();
	/**< destructor */
	void compute_diff(double& result,
					  const int& pos1,
					  const double& input1,
					  const double& input1_dt) const;
	/**< fill up result differential based on position, input, and its partial
	differential with time */
}; // closes class table1V_diff_prec

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE1V_DIFF_REAL
// =======================
// =======================

class MATH_API table1V_diff_real: public table1V_diff {
private:
	math::f_table1V* _pred;
	/**< weak pointer to f_table1V class */
	table1V_diff_real();
	/**< empty constructor not implemented */
	table1V_diff_real(const table1V_diff_real&);
	/**< copy constructor not implemented */
	table1V_diff_real& operator=(const table1V_diff_real& op2);
	/**< overloaded operator = (assignment) not implemented */
public:
	table1V_diff_real(math::f_table1V& pred);
	/**< constructor based on one dimensional table */
	~table1V_diff_real();
	/**< destructor */
	void compute_diff(double& result,
					  const int& pos1,
					  const double& input1,
					  const double& input1_dt) const;
	/**< fill up result differential based on position, input, and its partial
	differential with time */
}; // closes class table1V_diff_real

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

} // closes namespace math

#endif




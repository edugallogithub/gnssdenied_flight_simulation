#ifndef MATH_F_TABLE2V_H
#define MATH_F_TABLE2V_H

#include "../../math.h"
#include "pred2v.h"

/*
Derivate class of pred2v containing f_table2V objects. Refer to pred2v
for more info. Contains a two dimensional table.

Extrapolation when outside limits.

NOTE: Watch out when input magnitude is either a longitude or a bearing,
as a discontinuity in the results appears below the first input (inputs below
the first are considered as being after the last).

LEFT:
- Binary search could be accelerated if input from previous execution
  were added to the input. It would need to be stored however.
- Improve accuracy of differntial computation by doing before and after point,
  instead of only after.
- For Hermite interpolation, there exists an explicit expression for the
  differential computation.

SLOW:
- Repetition of binary searches.
- The linear_precompute interpolation method requires precomputing slopes at
  construction time (slower) but the computation of differentials is faster. The
  spline interpolation method requires computing the spline second differentials
  at construction time may times (slow), but also once at execution time (very slow).
  Hermite also requires precomputation.

*/

namespace math {

	class table2V_diff;

// CLASS F_TABLE2V
// ===============
// ===============

class MATH_API f_table2V : public pred2v {
private:
	friend class table2V_diff_prec;
	friend class table2V_diff_real;
	/**< classes that have access to private members */

	static const math::logic::PRED_NAME _name;
	/**< predicate name */
	static const std::string _st_name;
	/**< predicate name string */
	math::vec1* _points2;
	/**< pointer to object of class vec1 (ordered magnitude vector of size
	n defining the table 1st inputs). */
	math::vec1* _points1;
	/**< pointer to object of class vec1 (ordered magnitude vector of size
	m defining the table 2nd inputs). */
	math::vec2* _Values;
	/**< pointer to object of class vec2 (size n vector to pointers to
	size m vec1 objects) containing the table outputs */
	bool _del_flag;
	/**< flag that indicates if the _points2, _points1, and _Values vectors
	shall be deleted by the destructor (true) or not (false) */

	math::logic::INTERP_MODE _interp_mode;
	/**< interpolation method name */
	math::interp* _interp;
	/**< pointer to interpolation method */
	bool _equi2;
	/**< true if _points2 vector equispaced, false otherwise */
	bool _equi1;
	/**< true if _points1 vector equispaced, false otherwise */
	double _points2_diff;
	/**< difference between consecutive _points2, 0. if not equispaced */
	double _points1_diff;
	/**< difference between consecutive _points1, 0. if not equispaced */

	math::table2V_diff* _functor_diff;
	/**< pointer to functor that solves differentials */
	math::vec2* _slopes_d2;
	/**< pointer to object of class vec2 (size m vector to pointers to
	size n vec1 objects) containing the differentials (slopes) with respect to the
	1st independent magnitude. Derivate class coefficient always used. */
	math::vec2* _slopes_d1;
	/**< pointer to object of class vec2 (size n vector to pointers to
	size m vec1 objects) containing the differentials (slopes) with respect to the
	2nd independent magnitude. Derivate class coefficient always used. */
	math::hermite2v* _herm;
	/**< pointer to Hermite coefficients (if applicable) */

	math::range_checker* _checker2;
	/**< object controlling out of range for _points2, inactive by default */
	math::range_checker* _checker1;
	/**< object controlling out of range for _points1, inactive by default */
	math::pos_finder* _finder2;
	/**< object determining position of input with respect to _points2 */
	math::pos_finder* _finder1;
	/**< object determining position of input with respect to _points1 */

	void fill_up_slopes_lagrange_first_precompute();
	void fill_up_slopes_hermite();
	/**< fills up the _slopes_d2 and _slopes_d1 attributes based on _points2,
	_points1, and _Values */
	void fill_up_slopes_aux2_lagrange_first_precompute(math::vec2& slopes_d2,
													   unsigned short pos,
													   unsigned short siz);
	void fill_up_slopes_aux2_hermite(math::vec2& slopes_d2,
									 unsigned short pos,
									 unsigned short siz);
	/**< fills up row "pos" of the _slopes_d2 matrix of differentials with respect to the
	first independent magnitude with a vector of size "siz" */
	void fill_up_slopes_aux1_lagrange_first_precompute(math::vec2& slopes_d1,
													   unsigned short pos,
													   unsigned short siz);
	void fill_up_slopes_aux1_hermite(math::vec2& slopes_d1,
									 unsigned short pos,
									 unsigned short siz);
	/**< fills up row "pos" of the _slopes_d1 matrix of differentials with respect
	to the second independent magnitude with a vector of size "siz" */

	f_table2V();
	/**< empty constructor not implemented */
	f_table2V& operator=(const f_table2V& op2);
	/**< overloaded operator = (assignment) not implemented */

	void initialize();
	/**< initialization for constructors */
	void destroy();
	/**< destructor */
public:
	f_table2V(math::vec1* points2,
			  math::vec1* points1,
			  math::vec2* Values,
			  math::logic::INTERP_MODE interp_mode = math::logic::lagrange_first_precompute);
	/**< constructor based on pointer to size n vector points2, pointer
	to a size m vector points1, and pointer to a size n vector of size m
	vectors Values - deleted by destructor. */
	f_table2V(math::vec1& points2,
			  math::vec1& points1,
			  math::vec2& Values,
			  math::logic::INTERP_MODE interp_mode = math::logic::lagrange_first_precompute);
	/**< constructor based on reference to size n vector points2, reference
	to a size m vector points1, and reference to a size n vector of size m
	vectors Values - not deleted by destructor. */
	f_table2V(const f_table2V&);
	/**< copy constructor */
	virtual ~f_table2V() {destroy();}
	/**< destructor */
	virtual f_table2V* clone() const;
	/**< cloner */
	bool operator==(const pred2v& op2) const;
	bool operator==(const f_table2V& op2) const;
	/**< overloaded operator == (equal) */
	bool operator!=(const f_table2V& op2) const
	{return !(*this == op2);}
	/**< overloaded operator != (not equal) */

	const math::vec1& get_points2() const {return *_points2;}
	/**< get the _points2 object as constant reference to read */
	const math::vec1& get_points1() const {return *_points1;}
	/**< get the _points1 object as constant reference to read */
	const math::vec2& get_Values() const {return *_Values;}
	/**< get the _Values object as constant reference to read */
	const logic::PRED_NAME& get_name() const {return _name;}
	/**< see virtual function of class pred2v above */
	const std::string& get_st_name() const {return _st_name;}
	/**< see virtual function of class pred2v above */
	const math::logic::INTERP_MODE& get_interp_mode() const {return _interp_mode;}
	/**< get the interpolation mode to read */
	const math::interp& get_interp() const {return *_interp;}
	/**< get constant reference to interpolation method */
	const math::vec2& get_slopes_d2() const {return *_slopes_d2;}
	/**< return _slopes_d2 vector to read. Only required to take a look at the
	_slopes_d2 vector in the unitary tests */
	const math::vec2& get_slopes_d1() const {return *_slopes_d1;}
	/**< return _slopes_d1 vector to read. Only required to take a look at the
	_slopes_d1 vector in the unitary tests */
	bool get_equi2() const {return _equi2;}
	/**< returns true if _points2 vector equispaced, false otherwise */
	bool get_equi1() const {return _equi1;}
	/**< returns true if _points1 vector equispaced, false otherwise */
	const math::pos_finder& get_finder2() const {return *_finder2;}
	/**< returns constant reference to finder object */
	const math::pos_finder& get_finder1() const {return *_finder1;}
	/**< returns constant reference to finder object */
	const double& get_points2_diff() const {return _points2_diff;}
	/**< returns difference between consecutive _points2, 0. if not equispaced */
	const double& get_points1_diff() const {return _points1_diff;}
	/**< returns difference between consecutive _points1, 0. if not equispaced */
    const math::range_checker& get_checker1() const {return *_checker1;}
    /**< returns points1 out of range checker */
    const math::range_checker& get_checker2() const {return *_checker2;}
    /**< returns points2 out of range checker */

	void activate_checker2();
	void deactivate_checker2();
	/**< Activates or deactives the out of range verification for _points2, which
	is inactive by default */
	void activate_checker1();
	void deactivate_checker1();
	/**< Activates or deactives the out of range verification for _points1, which
	is inactive by default */

	int compute_pos2(const double& input2) const;
	/**< Returns first position within _points2 vector that shall be employed when
	interpolating to obtain the result corresponding to the input magnitude. */
	int compute_pos1(const double& input1) const;
	/**< Returns first position within _points1 vector that shall be employed when
	interpolating to obtain the result corresponding to the input magnitude. */
	math::ratio* compute_ratio2(const double& input2,
									  const int& pos2) const;
	/**< Returns ratio of input2 with respect to the two points2 identified by pos2 and
	pos2+1. */
	math::ratio* compute_ratio1(const double& input1,
									  const int& pos1) const;
	/**< Returns ratio of input1 with respect to the two points1 identified by pos1 and
	pos1+1. */
	double compute_value(const int& pos2,
                         const int& pos1,
                         const math::ratio& ratio2,
                         const math::ratio& ratio1) const;
	/**< Fills up result magnitude by interpolating based on the input positions
	and ratios */
	double compute_diff(const int& pos2,
					  const int& pos1,
					  const double& input2,
					  const double& input1,
					  const double& input2_dt,
					  const double& input1_dt) const;
	/**< Fills up result differential based on positions and ratios */

    double value(const double& input2,
                 const double& input1) const;
	/**< see virtual function of class predicate2v above. */
    double d_dt(const double& input2,
                const double& input1,
                const double& input2_dt,
                const double& input1_dt) const;
	/**< see virtual function of class pred1v above. */
	double d_d2(const double& input2,
				const double& input1) const;
	/**< computes partial variation of Values with respect to 1st independent
	magnitude, based on the values of the two independent magnitudes. Crashes
	if _interp_mode != math::logic::lagrange_first_precompute. */
	double d_d1(const double& input2,
				const double& input1) const;
	/**< computes partial variation of Values with respect to 2nd independent
	magnitude, based on the values of the two independent magnitudes. Crashes
	if _interp_mode != math::logic::lagrange_first_precompute.*/

}; // closes class f_table2V

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE2V_DIFF
// ==================
// ==================

class MATH_API table2V_diff {
public:
	virtual void compute_diff(double& result,
							  const int& pos2,
							  const int& pos1,
							  const double& input2,
							  const double& input1,
							  const double& input2_dt,
							  const double& input1_dt) const = 0;
	/**< fill up result differential based on positions, inputs, and its partial
	differentials with time */
}; // closes class table2V_diff

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE2V_DIFF_PREC
// =======================
// =======================

class MATH_API table2V_diff_prec: public table2V_diff {
private:
	math::f_table2V* _pred;
	/**< weak pointer to f_table2V class */
	table2V_diff_prec();
	/**< empty constructor not implemented */
	table2V_diff_prec(const table2V_diff_prec&);
	/**< copy constructor not implemented */
	table2V_diff_prec& operator=(const table2V_diff_prec& op2);
	/**< overloaded operator = (assignment) not implemented */
public:
	table2V_diff_prec(math::f_table2V& pred);
	/**< constructor based on two dimensional table */
	~table2V_diff_prec();
	/**< destructor */
	void compute_diff(double& result,
					  const int& pos2,
					  const int& pos1,
					  const double& input2,
					  const double& input1,
					  const double& input2_dt,
					  const double& input1_dt) const;
	/**< fill up result differential based on positions, inputs, and its partial
	differentials with time */
}; // closes class table2V_diff_prec

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE2V_DIFF_REAL
// =======================
// =======================

class MATH_API table2V_diff_real: public table2V_diff {
private:
	math::f_table2V* _pred;
	/**< weak pointer to f_table2V class */
	table2V_diff_real();
	/**< empty constructor not implemented */
	table2V_diff_real(const table2V_diff_real&);
	/**< copy constructor not implemented */
	table2V_diff_real& operator=(const table2V_diff_real& op2);
	/**< overloaded operator = (assignment) not implemented */
public:
	table2V_diff_real(math::f_table2V& pred);
	/**< constructor based on two dimensional table */
	~table2V_diff_real();
	/**< destructor */
	void compute_diff(double& result,
					  const int& pos2,
					  const int& pos1,
					  const double& input2,
					  const double& input1,
					  const double& input2_dt,
					  const double& input1_dt) const;
	/**< fill up result differential based on positions, inputs, and its partial
	differentials with time */
}; // closes class table2V_diff_real

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

} // closes namespace math

#endif




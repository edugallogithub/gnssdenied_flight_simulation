#ifndef MATH_F_TABLE3V_H
#define MATH_F_TABLE3V_H

#include "../../math.h"
#include "pred3v.h"

/*
Derivate class of pred3v containing f_table3V objects. Refer to pred3v
for more info. Contains a three dimensional table.

Extrapolation when outside limits. However, tables are protected for
all dimensions except first (generally altitude), so an exception is
launched if extrapolation is required in the 2nd or 3rd dimensions.

NOTE: Watch out when input magnitude is either a longitude or a bearing,
as a discontinuity in the results appears below the first input (inputs below
the first are considered as being after the last).

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
  spline interpolation method is not allowed for three dimensions.
  Hermite also requires precomputation.
*/

namespace math {

	class table3V_diff;

// CLASS F_TABLE3V
// ===============
// ===============

class MATH_API f_table3V : public pred3v {
private:
	friend class table3V_diff_prec;
	friend class table3V_diff_real;
	/**< classes that have access to private members */

	static const math::logic::PRED_NAME _name;
	/**< predicate name */
	static const std::string _st_name;
	/**< predicate name string */
	math::vec1* _points3;
	/**< pointer to object of class vec1 (ordered magnitude vector of size
	l defining the table 1st inputs). */
	math::vec1* _points2;
	/**< pointer to object of class vec1 (ordered magnitude vector of size
	m defining the table 2nd inputs). */
	math::vec1* _points1;
	/**< pointer to object of class vec1 (ordered magnitude vector of size
	n defining the table 3rd inputs). */
	math::vec3* _VValues;
	/**< pointer to object of class vec3 (size l vector to pointers to
	size m vectors to pointers to size n vector of vec1 objects
	containing the table outputs) */
	bool _del_flag;
	/**< flag that indicates if the _points3, _points2, _points1 and _Values vectors
	shall be deleted by the destructor (true) or not (false) */

	math::logic::INTERP_MODE _interp_mode;
	/**< interpolation method name */
	math::interp* _interp;
	/**< pointer to interpolation method */
	bool _equi3;
	/**< true if _points3 vector equispaced, false otherwise */
	bool _equi2;
	/**< true if _points2 vector equispaced, false otherwise */
	bool _equi1;
	/**< true if _points1 vector equispaced, false otherwise */
	double _points3_diff;
	/**< difference between consecutive _points3, 0. if not equispaced */
	double _points2_diff;
	/**< difference between consecutive _points2, 0. if not equispaced */
	double _points1_diff;
	/**< difference between consecutive _points1, 0. if not equispaced */

	math::table3V_diff* _functor_diff;
	/**< pointer to functor that solves differentials */
	math::vec3* _slopes_d3;
	/**< pointer to object of class vec3 (size l vector to pointer to
	size m vectors to pointers to size n vector of vec1 objects) containing
	the differentials (slopes) with respect to the 1st independent
	magnitude. Derivate class coefficient always used. */
	math::vec3* _slopes_d2;
	/**< pointer to object of class vec3 (size l vector to pointer to
	size m vectors to pointers to size n vector of vec1 objects) containing
	the differentials (slopes) with respect to the 2nd independent
	magnitude. Derivate class coefficient always used. */
	math::vec3* _slopes_d1;
	/**< pointer to object of class vec3 (size l vector to pointer to
	size m vectors to pointers to size n vector of vec1 objects) containing
	the differentials (slopes) with respect to the 3rd independent
	magnitude. Derivate class coefficient always used. */
	math::hermite3v* _herm;
	/**< pointer to Hermite coefficients (if applicable) */

	math::range_checker* _checker3;
	/**< object controlling out of range for _points3, inactive by default */
	math::range_checker* _checker2;
	/**< object controlling out of range for _points2, active by default */
	math::range_checker* _checker1;
	/**< object controlling out of range for _points1, active by default */
	math::pos_finder* _finder3;
	/**< object determining position of input with respect to _points3 */
	math::pos_finder* _finder2;
	/**< object determining position of input with respect to _points2 */
	math::pos_finder* _finder1;
	/**< object determining position of input with respect to _points1 */

	void fill_up_slopes_lagrange_first_precompute();
	void fill_up_slopes_hermite();
	/**< fills up the _slopes_d3, _slopes_d2, and _slopes_d1 attributes based on
	_points3, _points2, _points1, and _VValues */
	void fill_up_slopes_aux3_lagrange_first_precompute(math::vec3& slopes_d3,
													   unsigned short pos,
													   unsigned short siz2,
													   unsigned short siz3);
	void fill_up_slopes_aux3_hermite(math::vec3& slopes_d3,
									 unsigned short index2,
									 unsigned short index1);
	/**< fills up order "pos" of the _slopes_d3 3Dmatrix of differentials with respect to the
	first independent magnitude with a matrix of size "siz2" by "siz3" */
	void fill_up_slopes_aux2_lagrange_first_precompute(math::vec3& slopes_d2,
													   unsigned short pos,
													   unsigned short siz2,
													   unsigned short siz3);
	void fill_up_slopes_aux2_hermite(math::vec3& slopes_d2,
									 unsigned short index3,
									 unsigned short index1);
	/**< fills up order "pos" of the _slopes_d2 3Dmatrix of differentials with respect
	to the second independent magnitude with a matrix of size "siz2" by "siz3" */
	void fill_up_slopes_aux1_lagrange_first_precompute(math::vec3& slopes_d1,
													   unsigned short pos,
													   unsigned short siz2,
													   unsigned short siz3);
	void fill_up_slopes_aux1_hermite(math::vec3& slopes_d1,
									 unsigned short index3,
									 unsigned short index2);
	/**< fills up order "pos" of the _slopes_d1 3Dmatrix of differentials with respect
	to the third independent magnitude with a matrix of size "siz2" by "siz3" */

	f_table3V();
	/**< empty constructor not implemented */
	f_table3V& operator=(const f_table3V& op2);
	/**< overloaded operator = (assignment) not implemented */

	void initialize();
	/**< initialization for constructors */
	void destroy();
	/**< destructor */
public:
	f_table3V(math::vec1* points3,
			  math::vec1* points2,
			  math::vec1* points1,
			  math::vec3* VValues,
			  math::logic::INTERP_MODE interp_mode = math::logic::lagrange_first_precompute);
	/**< constructor based on pointer to size l vector points3, pointer	to a
	size m vector points2, pointer to a size n vector points1, and pointer to
	a size l vector of size m vectors of size n vectors VValues - deleted
	by destructor. */
	f_table3V(math::vec1& points3,
			  math::vec1& points2,
			  math::vec1& points1,
			  math::vec3& VValues,
			  math::logic::INTERP_MODE interp_mode = math::logic::lagrange_first_precompute);
	/**< constructor based on reference to size l vector points3, reference	to a
	size m vector points2, reference to a size n vector points1, and reference to
	a size l vector of size m vectors of size n vectors VValues - not deleted
	by destructor. */
	f_table3V(const f_table3V&);
	/**< copy constructor */
	virtual ~f_table3V() {destroy();}
	/**< destructor */
	virtual f_table3V* clone() const;
	/**< cloner */
	bool operator==(const pred3v& op2) const;
	bool operator==(const f_table3V& op2) const;
	/**< overloaded operator == (equal) */
	bool operator!=(const f_table3V& op2) const
	{return !(*this == op2);}
	/**< overloaded operator != (not equal) */

	const math::vec1& get_points3() const {return *_points3;}
	/**< get the _points3 object as constant reference to read */
	const math::vec1& get_points2() const {return *_points2;}
	/**< get the _points2 object as constant reference to read */
	const math::vec1& get_points1() const {return *_points1;}
	/**< get the _points1 object as constant reference to read */
	const math::vec3& get_VValues() const {return *_VValues;}
	/**< get the _VValues object as constant reference to read */
	const logic::PRED_NAME& get_name() const {return _name;}
	/**< see virtual function of class pred3v above */
	const std::string& get_st_name() const {return _st_name;}
	/**< see virtual function of class pred3v above */
	const math::logic::INTERP_MODE& get_interp_mode() const {return _interp_mode;}
	/**< get the interpolation mode to read */
	const math::interp& get_interp() const {return *_interp;}
	/**< get constant reference to interpolation method */
	const math::vec3& get_slopes_d3() const {return *_slopes_d3;}
	/**< return _slopes_d3 matrix to read. Only required to take a look at the
	_slopes_d3 matrix in the unitary tests */
	const math::vec3& get_slopes_d2() const {return *_slopes_d2;}
	/**< return _slopes_d2 matrix to read. Only required to take a look at the
	_slopes_d2 matrix in the unitary tests */
	const math::vec3& get_slopes_d1() const {return *_slopes_d1;}
	/**< return _slopes_d1 matrix to read. Only required to take a look at the
	_slopes_d1 matrix in the unitary tests */
	bool get_equi3() const {return _equi3;}
	/**< returns true if _points3 vector equispaced, false otherwise */
	bool get_equi2() const {return _equi2;}
	/**< returns true if _points2 vector equispaced, false otherwise */
	bool get_equi1() const {return _equi1;}
	/**< returns true if _points1 vector equispaced, false otherwise */
	const math::pos_finder& get_finder3() const {return *_finder3;}
	/**< returns constant reference to finder object */
	const math::pos_finder& get_finder2() const {return *_finder2;}
	/**< returns constant reference to finder object */
	const math::pos_finder& get_finder1() const {return *_finder1;}
	/**< returns constant reference to finder object */
	const double& get_points3_diff() const {return _points3_diff;}
	/**< returns difference between consecutive _points3, 0. if not equispaced */
	const double& get_points2_diff() const {return _points2_diff;}
	/**< returns difference between consecutive _points2, 0. if not equispaced */
	const double& get_points1_diff() const {return _points1_diff;}
	/**< returns difference between consecutive _points1, 0. if not equispaced */
    const math::range_checker& get_checker1() const {return *_checker1;}
    /**< returns points1 out of range checker */
    const math::range_checker& get_checker2() const {return *_checker2;}
    /**< returns points2 out of range checker */
    const math::range_checker& get_checker3() const {return *_checker3;}
    /**< returns points3 out of range checker */

	void activate_checker3();
	void deactivate_checker3();
	/**< Activates or deactives the out of range verification for _points3, which
	is inactive by default */
	void activate_checker2();
	void deactivate_checker2();
	/**< Activates or deactives the out of range verification for _points2, which
	is active by default */
	void activate_checker1();
	void deactivate_checker1();
	/**< Activates or deactives the out of range verification for _points1, which
	is active by default */

	int compute_pos3(const double& input3) const;
	/**< Returns first position within _points3 vector that shall be employed when
	interpolating to obtain the result corresponding to the input magnitude. */
	int compute_pos2(const double& input2) const;
	/**< Returns first position within _points2 vector that shall be employed when
	interpolating to obtain the result corresponding to the input magnitude. */
	int compute_pos1(const double& input1) const;
	/**< Returns first position within _points1 vector that shall be employed when
	interpolating to obtain the result corresponding to the input magnitude. */
	math::ratio* compute_ratio3(const double& input3,
									  const int& pos3) const;
	/**< Returns ratio of input3 with respect to the two points3 identified by pos3
	and pos3+1. */
	math::ratio* compute_ratio2(const double& input2,
									  const int& pos2) const;
	/**< Returns ratio of input2 with respect to the two points2 identified by pos2
	and pos2+1. */
	math::ratio* compute_ratio1(const double& input1,
									  const int& pos1) const;
	/**< Returns ratio of input1 with respect to the two points1 identified by pos1
	and pos1+1. */
	double compute_value(const int& pos3,
					   const int& pos2,
					   const int& pos1,
					   const math::ratio& ratio3,
					   const math::ratio& ratio2,
					   const math::ratio& ratio1) const;
	/**< Fills up result magnitude by interpolating based on the input positions
	and ratios */
	double compute_diff(const int& pos3,
					  const int& pos2,
					  const int& pos1,
					  const double& input3,
					  const double& input2,
					  const double& input1,
					  const double& input3_dt,
					  const double& input2_dt,
					  const double& input1_dt) const;
	/**< Fills up result differential based on positions and ratios */

    double value(const double& input3,
			   const double& input2,
			   const double& input1) const;
	/**< see virtual function of class predicate3v above. */
    double d_dt(const double& input3, const double& input2, const double& input1,
                const double& input3_dt, const double& input2_dt, const double& input1_dt) const;
	/**< see virtual function of class pred3v above. */

}; // closes class f_table3V

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE3V_DIFF
// ==================
// ==================

class MATH_API table3V_diff {
public:
	virtual void compute_diff(double& result,
							  const int& pos3,
							  const int& pos2,
							  const int& pos1,
							  const double& input3,
							  const double& input2,
							  const double& input1,
							  const double& input3_dt,
							  const double& input2_dt,
							  const double& input1_dt) const = 0;
	/**< fill up result differential based on positions, inputs, and its partial
	differentials with time */
}; // closes class table3V_diff

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE3V_DIFF_PREC
// =======================
// =======================

class MATH_API table3V_diff_prec: public table3V_diff {
private:
	math::f_table3V* _pred;
	/**< weak pointer to f_table3V class */
	table3V_diff_prec();
	/**< empty constructor not implemented */
	table3V_diff_prec(const table3V_diff_prec&);
	/**< copy constructor not implemented */
	table3V_diff_prec& operator=(const table3V_diff_prec& op2);
	/**< overloaded operator = (assignment) not implemented */
public:
	table3V_diff_prec(math::f_table3V& pred);
	/**< constructor based on three dimensional table */
	~table3V_diff_prec();
	/**< destructor */
	void compute_diff(double& result,
					  const int& pos3,
					  const int& pos2,
					  const int& pos1,
					  const double& input3,
					  const double& input2,
					  const double& input1,
					  const double& input3_dt,
					  const double& input2_dt,
					  const double& input1_dt) const;
	/**< fill up result differential based on positions, inputs, and its partial
	differentials with time */
}; // closes class table3V_diff_prec

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE3V_DIFF_REAL
// =======================
// =======================

class MATH_API table3V_diff_real: public table3V_diff {
private:
	math::f_table3V* _pred;
	/**< weak pointer to f_table3V class */
	table3V_diff_real();
	/**< empty constructor not implemented */
	table3V_diff_real(const table3V_diff_real&);
	/**< copy constructor not implemented */
	table3V_diff_real& operator=(const table3V_diff_real& op2);
	/**< overloaded operator = (assignment) not implemented */
public:
	table3V_diff_real(math::f_table3V& pred);
	/**< constructor based on three dimensional table */
	~table3V_diff_real();
	/**< destructor */
	void compute_diff(double& result,
					  const int& pos3,
					  const int& pos2,
					  const int& pos1,
					  const double& input3,
					  const double& input2,
					  const double& input1,
					  const double& input3_dt,
					  const double& input2_dt,
					  const double& input1_dt) const;
	/**< fill up result differential based on positions, inputs, and its partial
	differentials with time */
}; // closes class table3V_diff_real

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


} // closes namespace math

#endif






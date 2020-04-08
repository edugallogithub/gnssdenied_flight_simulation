#ifndef MATH_INTERP_H
#define MATH_INTERP_H

#include "../math.h"
#include "../vec/vec4.h"
#include "ratio_mgr.h"
#include "hermite.h"

/*
This file contains the interpolation (interp) base class, which declares the
abstract methods implemented in its derivate classes (lagrange 1st-2nd-3rd
order, splines, and hermite), and implements those shared by all of them.

The interpolation problem is that based on obtaining the value of a function
when provided with its input magnitudes when the function is unknown, but
however its outputs at a series of different inputs is known. This relationship
between the inputs and the outpus is provided by a table, which can be 
equispaced or not. The table inputs are provided by a series of vectors
of class vec1 (as many as the table dimensions), while the outputs are
provided by a vector of class vec1 (for 1 dimension), 2D matrix of class
vec2 (for 2 dimensions), 3D matrix of class vec3 (for 3 dimensions), or
4D matrix of class vec4 (for 4 dimensions).

When evaluating the table at the input magnitudes(s), the problem is called
interpolation if they are bracketed within the input vectors, or extrapolation
if they fall outside the vectors. Extrapolation is inherently dangerous as 
the behaviour of the function can not be assured.

The first step when evaluating the problem is always to determine which members
of the table shall be used for the computation, this is, to determine where do
the input magnitudes fall within the table inputs. This is done by a combinaation
of the search (which belong to the vec1 class) and find_index (which belong to
this class) methods. There are two search methods returning the proper position
of the input magnitude in the input vector: search_binary works when the vector
is not equispaced, while search_equispaced is much faster but only works for
equispaced vectors. Please read the SLOW and LEFT paragraphs below. The find_index
method, overloaded for each derivate class, takes the search result and returns
the specific members to be employed in the interpolation.

The second step is to compute the relative position of the input magnitudes
with respect to those vector members to be employed in the interpolation. This
is accomplished by the compute_ratio methods (overloaded), which return pointers
to the different ratio classes. These pointers shall later be returned with the
to_pool method.

The third step is the interpolation itself, which in addition to the table outputs
described below (1, 2, 3, or 4 dimensional table), requires the inputs of the two
previous steps, this is, the input position and ratios along each of the table
dimensions. These functions are also overloaded.

Internally, the Lagrange 1st, 2nd, and 3rd order classes do not contain
any attributes but make use of the base class basic_interp_lagrange_first, 
basic_interp_lagrange_second, and basic_interp_lagrange_third functions. The
spline interplation works slightly different: at this time it is only capable
of handling 1 or 2 dimensions, and it relies on a vector of second differentials 
that must be filled up at the construction time. The Hermite interpolation relies
on the differentials at the nodes that shall be computed independently

LEFT:
- Improve binary_search with position of previous execution. The problem is
  that this position needs to be stored somehow, which prevents multithreading
  capabilities.

SLOW:
- The search_binary function executes a binary search, and should be avoided
  is possible. It can not be accelerated.

MODE OF USE:
Employ the get_interp factories to obtain pointers to the interpolation 
derivate classes. They are based on the INTERP_MODE enumeration. Then
proceed as described above.

These classes are however not intended to be directly employed by the user,
but the table classes.

*/

namespace math {
	class f_table1V;
	class f_table2V;
	class f_tabular2V;

// CLASS INTERP
// ============
// ============

class MATH_API interp {
private:


public:
	virtual ~interp() {}
	/**< destructor (existance mandatory to declare it as virtual so derived
	destructors are called avoiding memory leaks) */
	virtual const math::logic::INTERP_MODE& get_interp_mode() const = 0;
	/**< get interpolation method enumerator to read */
	virtual const int& get_min_points() const = 0;
	/**< returns minimum number of points required for interpolation */
		
	virtual void complete_spline(const math::vec1& points1,
  								 const math::vec& values,
								 const unsigned short& values_begin = 0) {}
	/**< complements the empty consturctor based on an input vector, and output
	vector of equal or greater size, and a position within the output vector that
	corresponds to the first position in the input. Employed for one dimensional 
	interpolation. DOES NOT LIBERATE MEMORY, SO USE ONLY USE AFTER CONSTRUCTOR.
	Only overloaded for splines. */
	virtual void complete_spline(const math::vec1& points2,
								 const math::vec1& points1,
								 const math::vec2& Values) {}
	/**< complements the emtpy constructor based on two input vectors, the first of
	size n and the second of size m, plues an output matrix of size nxm. Employed for
	two dimensional interpolation. DOES NOT LIBERATE MEMORY, SO USE ONLY USE AFTER
	CONSTRUCTOR. Only overloaded for splines. */
	virtual void set_diffs(const double points1_diff) {};
	/**< sets the _diff attribute when the input vector is equispaced, so all members
	of the _diff vector are equal to each other. Only overloaded for splines. */
	virtual void set_diffs(const math::vec1& points1) {};
	/**< sets the _diff attribute when the input vector is NOT equispaced, so all
	members of the _diff vector differ from each other. Only overloaded for splines. */

	static math::interp* get_interp(math::logic::INTERP_MODE);
	/**< return pointer to interpolation method that corresponds to input enumeration.
	In the case of splines it needs to be followed by the complete_spline method */
	static math::interp* get_interp(math::logic::INTERP_MODE,
										  const math::vec1& points1,
										  const math::vec1& values);
	/**< return pointer to interpolation method that corresponds to input enumeration.
	In the case of splines it also needs two same size vectors of magnitudes, one
	with the inputs, another with the outputs */
	static math::interp* get_interp(math::logic::INTERP_MODE,
										  const math::vec1& points2,
										  const math::vec1& points1,
										  const math::vec2& Values);
	/**< return pointer to interpolation method that corresponds to input enumeration.
	In the case of splines it also needs two vectors of magnitudes with the inputs and
	one with the outputs */

	static const std::string describe_interpolation(math::logic::INTERP_MODE index);
	/**< returns a string containing the name of the interpolation mode based on the
	INTERP_MODE enumeration */
	static const math::logic::INTERP_MODE reverse_describe_interpolation(std::string&);
	/**< returns the name of the interpolation mode in the INTERP_MODE
	enumeration based on a string describing it */

	/**< ===== ===== ===== Search Methods ===== ===== ===== */
	/**< ================================================== */
	virtual int find_index(const int& j,
						   const int& n) const = 0;
	/**< Given the size of a vector "n" and the result "j" of any of the "search"
	methods of the base class above, it searches for a group of positions within
	the vector from which to execute the interpolation, returning the position of
	the first one. If it returns x, the members required for the interpolation 
	are x, x+1, ... x+_min_points-1. */

	/**< ===== ===== ===== Ratio Methods ===== ===== ===== */
	/**< ================================================= */
	virtual math::ratio* compute_ratio(const double& input,
											 const math::vec1& vec,
											 const int& pos) const = 0;
	/**< Given an input magnitude, a vector of magnitudes, and the result of the
	"find_index" method, it returns a pointer to the appropriate ratio class with
	the ratios between the different vector members involved in the interpolation. */
	virtual double compute_Dratio(math::ratio& rat,
								  const double& input,
								  const math::vec1& vec,
								  const int& pos) const = 0;
	/**< Given an input magnitude, a vector of magnitudes, and the position provided by
	the "find_index" method, it modifies the input ratio object so it can be employed
	to compute the differential together with that provided by the "compute_ratio" method.
	It computes the values corresponding to the input magnitude plus one ten-thousandth 
	(1e-4) of the difference between positions "pos+1" and "pos" of the input vector. 
	Returns this thousandth of difference by its later use computing the differential. */
	virtual math::ratio* copy_ratio(math::ratio* p) const = 0;
	/**< returns pointer to ratio equal to input */
	virtual void to_pool(math::ratio* p) const = 0;
	/**< returns the interpolation ratio pointer to the storage so it can be employed again */

	/**< ===== ===== ===== Interpolation Methods ===== ===== ===== */
	/**< ========================================================= */
	virtual void interp1(double& result,
						 const math::vec1& points,
						 const math::vec1& values,
						 const int& pos1,
						 const math::ratio& rat1,
						 const math::hermite1v& her) const = 0;
	/**< Fills up the input magnitude by interpolating in ONE dimension based on the
	input magnitudes included in the 1D matrix values with the lowest position being
	that identified by pos1, and the ratios contained in rat1 */
	virtual void interp2(double& result,
						 const math::vec1& points2,
						 const math::vec1& points1,
						 const math::vec2& Values,
						 const int& pos2,
						 const int& pos1,
						 const math::ratio& rat2,
						 const math::ratio& rat1,
						 const math::hermite2v& her) const = 0;
	/**< Fills up the input magnitude by interpolating in TWO dimensions based on the
	input magnitudes included in the 2D matrix Values with the lowest positions being
	those identified by pos1 and pos2, and the ratios contained in rat1 and rat2*/

	virtual void interp2(double& result,
						 const math::vec1& points2,
						 const std::vector<math::f_table1V*>& tables,
						 const int& pos2,
						 const math::ratio& rat2,
						 const double& input1,
						 const math::hermite2v& her) const = 0;
	/**< Fills up the input magnitude by interpolating in TWO dimensions based on the
	input magnitudes included in the 2D matrix Values with the lowest positions being
	those identified by pos1 and pos2, and the ratios contained in rat1 and rat2.
	Contains the input magnitudes, position, and ratio only for the second dimension, 
	plus a series of unidimensional tables.*/
	virtual void interp3(double& result,
						 const math::vec1& points3,
						 const math::vec1& points2,
						 const math::vec1& points1,
						 const math::vec3& VValues,
						 const int& pos3,
						 const int& pos2,
						 const int& pos1,
						 const math::ratio& rat3,
						 const math::ratio& rat2,
						 const math::ratio& rat1,
						 const math::hermite3v& her) const = 0;
	virtual void interp3(double& result,
						 const math::vec1& points3,
						 const std::vector<math::f_table2V*>& tables,
						 const int& pos3,
						 const math::ratio& rat3,
						 const double& input2,
						 const double& input1,
						 const math::hermite3v& her) const = 0;
	virtual void interp3(double& result,
						 const math::vec1& points3,
						 const std::vector<math::f_tabular2V*>& tables,
						 const int& pos3,
						 const math::ratio& rat3,
						 const double& input2,
						 const double& input1,
						 const math::hermite3v& her) const = 0;
	/**< Fills up the input magnitude by interpolating in THREE dimensions based on the
	input magnitudes included in the 3D matrix VValues with the lowest positions being
	those identified by pos1, pos2, and pos3, and the ratios contained in rat1, rat2,
	and rat3. The second version contains the input magnitudes, position, and ratio only
	for the third dimension, plus a series of bidimensional tables. The third version 
	contains the input magnitudes, position, and ratio only for the third dimension, 
	plus a series of bidimensional tabular objects.*/
	virtual void interp4(double& result,
						 const math::vec1& points4,
						 const math::vec1& points3,
						 const math::vec1& points2,
						 const math::vec1& points1,
						 const math::vec4& VVValues,
						 const int& pos4,
						 const int& pos3,
						 const int& pos2,
						 const int& pos1,
						 const math::ratio& rat4,
						 const math::ratio& rat3,
						 const math::ratio& rat2,
						 const math::ratio& rat1,
						 const math::hermite4v& her) const = 0;
	/**< Fills up the input magnitude by interpolating in FOUR dimensions based on the
	input magnitudes included in the 4D matrix VVValues with the lowest positions being
	those identified by pos1, pos2, pos3, and pos4, and the ratios contained in rat1, rat2,
	rat3 and rat4 */

	/**< ===== ===== ===== Generic Static Methods ===== ===== ===== */
	/**< ========================================================== */
	static void basic_interp_lagrange_first(double& result,
									const double& op1,
									const double& op2,
									const double& ratio);
	static void basic_interp_lagrange_first(double& result,
									const math::vec& Values,
									const int& Values_begin,
									const double& ratio);
	/**< fills up the input magnitude result with the linear interpolation between
	the input magnitudes op1 and op2, based on the input ratio (interpolation if 
	ratio between 0 and 1, extrapolation otherwise). ratio = 0 returns op1 while
	ratio = 1 returns op2. Unpredictable results if the three input magnitudes do 
	not belong to the same derivate class. Instead of four magnitudes, it is also
	possible to provide a multidimensional vector of magnitudes plus a position, 
	taking two consecutive magnitudes starting at that position. */
	static void basic_interp_lagrange_second(double& result,
									   const double& op1,
									   const double& op2,
									   const double& op3,
									   const double& ratio12,
									   const double& ratio23,
									   const double& ratio13);
	static void basic_interp_lagrange_second(double& result,
									   const math::vec& VValues,
									   const int& VValues_begin,
									   const double& ratio12,
									   const double& ratio23,
									   const double& ratio13);
	/**< fills up the input magnitude result with the quadratic interpolation between
	the input magnitudes op1, op2, and op3, based on the input ratios (interpolation if 
	ratio between 0 and 1, extrapolation otherwise) between each two of them. Unpredictable
	results if the four input magnitudes do not belong to the same derivate class.
	Instead of four magnitudes, it is also possible to provide a multidimensional vector
	of magnitudes plus a position, taking three consecutive magnitudes
	starting at that position. */
	static void basic_interp_lagrange_third(double& result,
								   const double& op1,
								   const double& op2,
								   const double& op3,
								   const double& op4,
								   const double& ratio12,
								   const double& ratio23,
								   const double& ratio34,
								   const double& ratio13,
								   const double& ratio24,
								   const double& ratio14);
	static void basic_interp_lagrange_third(double& result,
									const math::vec& VVValues,
									const int& VVValues_begin,
									const double& ratio12,
									const double& ratio23,
									const double& ratio34,
									const double& ratio13,
									const double& ratio24,
									const double& ratio14);
	/**< fills up the input magnitude result with the cubic interpolation between
	the input magnitudes op1, op2, op3, and op4, based on the input ratios (interpolation if 
	ratio between 0 and 1, extrapolation otherwise) between each two of them. Unpredictable
	results if the five input magnitudes do not belong to the same mag derivate class.
	Instead of four magnitudes, it is also possible to provide a multidimensional vector
	of magnitudes plus a position, taking four consecutive magnitudes
	starting at that position. */
	static void basic_interp_lagrange_third_diff(double& diff_first,
										double& diff_last,
										const math::vec1& inp,
										const math::vec& out,
										const unsigned short& out_begin = 0);
	/**< Given an input vector inp, an output vector out of equal or greater size, and
	a position within the output vector that corresponds to the first position in the
	input, the function fills up the differential at the first and final position of
	the input assumming a cubic interpolation. Required for the computation
	of splines. */
	static void basic_interp_biparabolic(double& result,
								   const double& op1,
								   const double& op2,
								   const double& op3,
								   const double& op4,
								   const double& ratio12,
								   const double& ratio23,
								   const double& ratio34,
								   const double& ratio13,
								   const double& ratio24,
								   const double& ratio14);
	static void basic_interp_biparabolic(double& result,
									const math::vec& VVValues,
									const int& VVValues_begin,
									const double& ratio12,
									const double& ratio23,
									const double& ratio34,
									const double& ratio13,
									const double& ratio24,
									const double& ratio14);
	/**< fills up the input magnitude result with the biparabolic interpolation between
	the input magnitudes op1, op2, op3, and op4, based on the input ratios (interpolation if 
	ratio between 0 and 1, extrapolation otherwise) between each two of them. Unpredictable
	results if the five input magnitudes do not belong to the same mag derivate class.
	Instead of four magnitudes, it is also possible to provide a multidimensional vector
	of magnitudes plus a position, taking four consecutive magnitudes
	starting at that position. */
}; // closes class interp

} // closes namespace math

#endif









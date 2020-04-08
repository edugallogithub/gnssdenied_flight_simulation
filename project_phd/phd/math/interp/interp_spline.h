#ifndef MATH_INTERP_SPLINE_H
#define MATH_INTERP_SPLINE_H

#include "../math.h"
#include "interp.h"

/*
This file contains the interp_spline class, which implements cubic splines.
Of all the different types of cubic splines, this class implements those in
which the first differential at the borders coincides with those provided
by Lagrange 3rd order interpolation.

Results provided by splines are smooth in the first derivate and continuous
in the second. Their flipside is that the second differentials need to be 
computed and stored at construction, which takes time. As the construction
needs to be done in real time for multiple dimensions, they have been 
restricted to 2 or less.
*/

namespace math {

// CLASS INTERP_SPLINE
// ===================
// ===================

class MATH_API interp_spline: public interp {
private:
	static const math::logic::INTERP_MODE _interp_mode;
	/**< interpolation mode */
	static const int _min_points;
	/**< minimum number of points for interpolation */
	static const int _points_bracket;
	/**< number of points to compute the ratios */

	std::vector<double>* _diffdiff;
	/**< pointer to vector of size n containing the second differentials of the 
	functions. Employed for one dimensional interpolation. */
	std::vector<double>* _dist;
	/**< pointer to vector of doubles of size n-1 containing the distances
	between each pair of consecutive inputs. Employed for one dimensional 
	interpolation. */
	std::vector<math::interp_spline*>* _spline;
	/**< pointer to size m vector of objects of this same class. Employed
	for two dimensional interpolation. */ 
	const math::vec1* _points2;
	/**< weak pointer to first vector of inputs. Employed for two dimensional
	interpolation. */

	interp_spline(const interp_spline&);
	/**< copy constructor not implemented*/
	interp_spline& operator=(const interp_spline& op2) {return *this;}
	/**< overloaded operator = (assignment) not implemented */
public:
	interp_spline();
	/* empty constructor (must be followed by complete_spline method) */
	interp_spline(const math::vec1& points1,
				  const math::vec& values,
				  const unsigned short& values_begin = 0);
	/**< constructor based on an input vector, and output vector of equal or
	greater size, and a position within the output vector that corresponds to 
	the first position in the input. Employed for one dimensional 
	interpolation. */
	interp_spline(const math::vec1& points2,
				  const math::vec1& points1,
				  const math::vec2& Values);
	/**< constructor based on two input vectors, the first or size n and the second
	of size m, plues an output matrix of size nxm. Employed for two dimensional 
	interpolation. */
	~interp_spline();
	/**< destructor */
	const math::logic::INTERP_MODE& get_interp_mode() const
	{return _interp_mode;}
	/**< get interpolation method enumerator to read */
	const int& get_min_points() const
	{return _min_points;}
	/**< returns minimum number of points required for interpolation */

	void complete_spline(const math::vec1& points1,
						 const math::vec& values,
						 const unsigned short& values_begin = 0);
	/**< complements the empty consturctor based on an input vector, and output
	vector of equal or greater size, and a position within the output vector that
	corresponds to the first position in the input. Employed for one dimensional 
	interpolation. DOES NOT LIBERATE MEMORY, SO USE ONLY USE AFTER CONSTRUCTOR. */
	void complete_spline(const math::vec1& points2,
						 const math::vec1& points1,
						 const math::vec2& Values);
	/**< complements the emtpy constructor based on two input vectors, the first of
	size n and the second of size m, plues an output matrix of size nxm. Employed for
	two dimensional interpolation. DOES NOT LIBERATE MEMORY, SO USE ONLY USE AFTER
	CONSTRUCTOR. */

	void set_diffs(const double points1_diff);
	/**< sets the _diff attribute when the input vector is equispaced, so all members
	of the _diff vector are equal to each other. Employed for one dimensional 
	interpolation. */
	void set_diffs(const math::vec1& points1);
	/**< sets the _diff attribute when the input vector is NOT equispaced, so all
	members of the _diff vector differ from each other. Employed for one dimensional 
	interpolation. */

	/**< ===== ===== ===== Search Methods ===== ===== ===== */
	/**< ================================================== */
	int find_index(const int& j,
				   const int& n) const 
	{return std::min(std::max((j - (_points_bracket - 1) / 2), 0), n - _points_bracket);}
	/**< Given the size of a vector "n" and the result "j" of any of the "search"
	methods of the base class above, it searches for a group of positions within
	the vector from which to execute the interpolation, returning the position of
	the first one. If it returns x, the members required for the interpolation 
	are x, x+1, ... x+_points_bracket-1. */

	/**< ===== ===== ===== Ratio Methods ===== ===== ===== */
	/**< ================================================= */
	math::ratio* compute_ratio(const double& input,
									 const math::vec1& vec,
									 const int& pos) const;
	/**< Given an input magnitude, a vector of magnitudes, and the result of the
	"find_index" method, it returns a pointer to the appropriate ratio class with
	the ratios between the different vector members involved in the interpolation. */
	double compute_Dratio(math::ratio& rat,
						  const double& input,
						  const math::vec1& vec,
						  const int& pos) const;
	/**< Given an input magnitude, a vector of magnitudes, and the position provided by
	the "find_index" method, it modifies the input ratio object so it can be employed
	to compute the differential together with that provided by the "compute_ratio" method.
	It computes the values corresponding to the input magnitude plus one ten-thousandth 
	(1e-4) of the difference between positions "pos+1" and "pos" of the input vector. 
	Returns this thousandth of difference by its later use computing the differential. */
	math::ratio* copy_ratio(math::ratio* p) const;
	/**< returns pointer to ratio equal to input */
	void to_pool(math::ratio* p) const;
	/**< returns the interpolation ratio pointer to the storage so it can be employed again */

	/**< ===== ===== ===== Interpolation Methods ===== ===== ===== */
	/**< ========================================================= */
	void interp1(double& result,
				 const math::vec1& points,
				 const math::vec1& values,
				 const int& pos1,
				 const math::ratio& rat1,
				 const math::hermite1v& her) const;
	void interp1(double& result,
				 const math::vec& values,
				 const unsigned short& pos1,
				 const double& rat1) const;
	/**< Fills up the input magnitude by interpolating in ONE dimension based on the
	input magnitudes included in the 1D matrix values with the lowest position being
	that identified by pos1, and the ratios contained in rat1 */
	void interp2(double& result,
				 const math::vec1& points2,
				 const math::vec1& points1,
				 const math::vec2& Values,
				 const int& pos2,
				 const int& pos1,
				 const math::ratio& rat2,
				 const math::ratio& rat1,
				 const math::hermite2v& her) const;
	/**< Fills up the input magnitude by interpolating in TWO dimensions based on the
	input magnitudes included in the 2D matrix Values with the lowest positions being
	those identified by pos1 and pos2, and the ratios contained in rat1 and rat2*/
	void interp2(double& result,
				 const math::vec1& points2,
				 const std::vector<math::f_table1V*>& tables,
				 const int& pos2,
				 const math::ratio& rat2,
				 const double& input1,
				 const math::hermite2v& her) const;
	/**< Fills up the input magnitude by interpolating in TWO dimensions based on the
	input magnitudes included in the 2D matrix Values with the lowest positions being
	those identified by pos1 and pos2, and the ratios contained in rat1 and rat2.
	Contains the input magnitudes, position, and ratio only for the second dimension, 
	plus a series of unidimensional tables.*/
	void interp3(double& result,
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
				 const math::hermite3v& her) const;
	void interp3(double& result,
				 const math::vec1& points3,
				 const std::vector<math::f_table2V*>& tables,
				 const int& pos3,
				 const math::ratio& rat3,
				 const double& input2,
				 const double& input1,
				 const math::hermite3v& her) const;
	void interp3(double& result,
				 const math::vec1& points3,
				 const std::vector<math::f_tabular2V*>& tables,
				 const int& pos3,
				 const math::ratio& rat3,
				 const double& input2,
				 const double& input1,
				 const math::hermite3v& her) const;
	/**< Fills up the input magnitude by interpolating in THREE dimensions based on the
	input magnitudes included in the 3D matrix VValues with the lowest positions being
	those identified by pos1, pos2, and pos3, and the ratios contained in rat1, rat2,
	and rat3. The second version contains the input magnitudes, position, and ratio only
	for the third dimension, plus a series of bidimensional tables. The third version 
	contains the input magnitudes, position, and ratio only for the third dimension, 
	plus a series of bidimensional tabular objects. 
	METHODS NOT	IMPLEMENTED FOR SPLINES. EXCEPTION LAUNCHED. */
	void interp4(double& result,
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
				 const math::hermite4v& her) const;
	/**< Fills up the input magnitude by interpolating in FOUR dimensions based on the
	input magnitudes included in the 4D matrix VVValues with the lowest positions being
	those identified by pos1, pos2, pos3, and pos4, and the ratios contained in rat1, rat2,
	rat3 and rat4. METHOD NOT IMPLEMENTED FOR SPLINES. EXCEPTION LAUNCHED. */
}; // closes class interp_spline

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
	
} // closes namespace math

#endif









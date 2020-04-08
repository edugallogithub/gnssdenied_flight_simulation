#include "interp_hermite.h"
#include "../pred/pred1v/f_table1V.h"

// CLASS INTERP_HERMITE
// ====================
// ====================

const int math::interp_hermite::_min_points_first = 2;
const int math::interp_hermite::_min_points_second = 3;
/* minimum number of points for interpolation */

const int math::interp_hermite::_points_bracket = 2;
/* number of points to compute the ratios */

math::interp_hermite::interp_hermite(math::logic::INTERP_MODE interp_mode)
: _interp_mode(interp_mode) {
	_min_points = get_min_points();
}
/* constructor based on interpolation mode (only Hermite variants allowed) */

math::interp_hermite::interp_hermite(const interp_hermite& op2)
: _interp_mode(op2._interp_mode), _min_points(op2._min_points) {
}
/* copy constructor */

math::interp_hermite& math::interp_hermite::operator=(const interp_hermite& op2) {
	_interp_mode = op2._interp_mode;
	_min_points  = op2._min_points;
	return *this;	
}
/* overloaded operator = (assignment) */

math::interp_hermite::~interp_hermite() {
}
/* destructor */

const int& math::interp_hermite::get_min_points() const {
	if   (_interp_mode == math::logic::hermite_first) {return _min_points_first;}
	else                                            {return _min_points_second;}
}
/* returns minimum number of points required for interpolation */

/* ===== ===== ===== Ratio Methods ===== ===== ===== */
/* ================================================= */
math::ratio* math::interp_hermite::compute_ratio(const double& input,
															 const math::vec1& vec,
															 const int& pos) const {
	math::ratio_hermite* result = math::ratio_mgr::from_pool_hermite();
	result->set_ratio12( (input - vec[pos]) / (vec[pos+1] - vec[pos])     );
	return result;
}
/* Given an input magnitude, a vector of magnitudes, and the result of the
"find_index" method, it returns a pointer to the appropriate ratio class with
the ratios between the different vector members involved in the interpolation. */

double math::interp_hermite::compute_Dratio(math::ratio& rat,
												const double& input,
												const math::vec1& vec,
												const int& pos) const {
	math::ratio_hermite& rat_her = static_cast<math::ratio_hermite&>(rat);
	rat_her.set_ratio12(rat_her._ratio12 + 1e-4);
	return 1e-4 * (vec[pos+1] - vec[pos]);
}
/* Given an input magnitude, a vector of magnitudes, and the position provided by
the "find_index" method, it modifies the input ratio object so it can be employed
to compute the differential together with that provided by the "compute_ratio" method.
It computes the values corresponding to the input magnitude plus one ten-thousandth 
(1e-4) of the difference between positions "pos+1" and "pos" of the input vector. 
Returns this thousandth of difference by its later use computing the differential. */

math::ratio* math::interp_hermite::copy_ratio(math::ratio* p) const {
	return math::ratio_mgr::copy(static_cast<const math::ratio_hermite&>(*p));
}
/* returns pointer to ratio equal to input */

void math::interp_hermite::to_pool(math::ratio* p) const {
	math::ratio_mgr::to_pool_hermite(static_cast<math::ratio_hermite*>(p));
}
/* returns the interpolation ratio pointer to the storage so it can be employed again */

/* ===== ===== ===== Interpolation Methods ===== ===== ===== */
/* ========================================================= */
void math::interp_hermite::interp1(double& result,
										 const math::vec1& points,
										 const math::vec1& values,
										 const int& pos1,
										 const math::ratio& rat1,
										 const math::hermite1v& her) const {
	const math::ratio_hermite& rat1_her = static_cast<const math::ratio_hermite&>(rat1);
	double Delta = rat1_her._ratio12 * (points[pos1+1] - points[pos1]);
	result = her[pos1][0] +
					 her[pos1][1] * Delta +
					 her[pos1][2] * pow(Delta, 2.) +
					 her[pos1][3] * pow(Delta, 3.);
}
/* Fills up the input magnitude by interpolating in ONE dimension based on the
input magnitudes included in the 1D matrix values with the lowest position being
that identified by pos1, and the ratios contained in rat1 */

void math::interp_hermite::interp2(double& result,
										 const math::vec1& points2,
										 const math::vec1& points1,
										 const math::vec2& Values,
										 const int& pos2,
										 const int& pos1,
										 const math::ratio& rat2,
										 const math::ratio& rat1,
										 const math::hermite2v& her) const {
	const math::ratio_hermite& rat2_her = static_cast<const math::ratio_hermite&>(rat2);
	const math::ratio_hermite& rat1_her = static_cast<const math::ratio_hermite&>(rat1);
	double Delta2 = rat2_her._ratio12 * (points2[pos2+1] - points2[pos2]);
	double Delta1 = rat1_her._ratio12 * (points1[pos1+1] - points1[pos1]);
	double Delta1_sqr = pow(Delta1, 2.);
	double Delta1_cub = pow(Delta1, 3.);
	
	const math::hermite_pow2& Oher = *her[pos2][pos1];

	/////////////////////////////////////////////////////////////////////////////////
	// DO NOT DECOMMENT
	// The commented code below gives exactly the same results as what is implemented
	//result.set_value(Oher[0][0] + 
	//				 Oher[1][0] * Delta2 + 		
	//				 Oher[2][0] * pow(Delta2, 2.) +
	//				 Oher[3][0] * pow(Delta2, 3.) + 
	//				(Oher[0][1] + 
	//				 Oher[1][1] * Delta2 + 		
	//				 Oher[2][1] * pow(Delta2, 2.) +
	//				 Oher[3][1] * pow(Delta2, 3.)) * Delta1 + 
	//				(Oher[0][2] + 
	//				 Oher[1][2] * Delta2 + 		
	//				 Oher[2][2] * pow(Delta2, 2.) +
	//				 Oher[3][2] * pow(Delta2, 3.)) * pow(Delta1, 2.) + 
	//			    (Oher[0][3] + 
	//				 Oher[1][3] * Delta2 + 		
	//				 Oher[2][3] * pow(Delta2, 2.) +
	//				 Oher[3][3] * pow(Delta2, 3.)) * pow(Delta1, 3.));
	//////////////////////////////////////////////////////////////////////////////////

	result = Oher[0][0] +
					 Oher[0][1] * Delta1 + 		
					 Oher[0][2] * Delta1_sqr +
					 Oher[0][3] * Delta1_cub + 
					(Oher[1][0] + 
					 Oher[1][1] * Delta1 + 		
					 Oher[1][2] * Delta1_sqr +
					 Oher[1][3] * Delta1_cub) * Delta2 + 
					(Oher[2][0] + 
					 Oher[2][1] * Delta1 + 		
					 Oher[2][2] * Delta1_sqr +
					 Oher[2][3] * Delta1_cub) * pow(Delta2, 2.) + 
				    (Oher[3][0] + 
					 Oher[3][1] * Delta1 + 		
					 Oher[3][2] * Delta1_sqr +
					 Oher[3][3] * Delta1_cub) * pow(Delta2, 3.);
}
/* Fills up the input magnitude by interpolating in TWO dimensions based on the
input magnitudes included in the 2D matrix Values with the lowest positions being
those identified by pos1 and pos2, and the ratios contained in rat1 and rat2 */

void math::interp_hermite::interp2(double& result,
										const math::vec1& points2,
										const std::vector<math::f_table1V*>& tables,
										const int& pos2,
										const math::ratio& rat2,
										const double& input1,
										const math::hermite2v& her) const
{
	// TO DO

	//math::mag* temp1 = result.clone();
	//math::mag* temp2 = result.clone();

    //tables[pos2]->value(*temp1, input1);
	//tables[pos2+1]->value(*temp2, input1);

	//delete temp1;
	//delete temp2;
}
/*Fills up the input magnitude by interpolating in TWO dimensions. 
Contains the input magnitudes, position, and ratio only for the second dimension, 
plus a series of unidimensional tables.*/

void math::interp_hermite::interp3(double& result,
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
										 const math::hermite3v& her) const {
	const math::ratio_hermite& rat3_her = static_cast<const math::ratio_hermite&>(rat3);
	const math::ratio_hermite& rat2_her = static_cast<const math::ratio_hermite&>(rat2);
	const math::ratio_hermite& rat1_her = static_cast<const math::ratio_hermite&>(rat1);
	double Delta3 = rat3_her._ratio12 * (points3[pos3+1] - points3[pos3]);
	double Delta2 = rat2_her._ratio12 * (points2[pos2+1] - points2[pos2]);
	double Delta1 = rat1_her._ratio12 * (points1[pos1+1] - points1[pos1]);
	double Delta2_sqr = pow(Delta2, 2.);
	double Delta2_cub = pow(Delta2, 3.);
	double Delta1_sqr = pow(Delta1, 2.);
	double Delta1_cub = pow(Delta1, 3.);

	const math::hermite_pow3& Oher = *her[pos3][pos2][pos1];

	result = (Oher[0][0][0] +
					  Oher[0][0][1] * Delta1 + 		
					  Oher[0][0][2] * Delta1_sqr +
					  Oher[0][0][3] * Delta1_cub + 
					 (Oher[0][1][0] + 
					  Oher[0][1][1] * Delta1 + 		
					  Oher[0][1][2] * Delta1_sqr +
					  Oher[0][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[0][2][0] + 
					  Oher[0][2][1] * Delta1 + 		
					  Oher[0][2][2] * Delta1_sqr +
					  Oher[0][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[0][3][0] + 
					  Oher[0][3][1] * Delta1 + 		
					  Oher[0][3][2] * Delta1_sqr +
					  Oher[0][3][3] * Delta1_cub) * Delta2_cub) + 
					 (Oher[1][0][0] + 
					  Oher[1][0][1] * Delta1 + 		
					  Oher[1][0][2] * Delta1_sqr +
					  Oher[1][0][3] * Delta1_cub + 
					 (Oher[1][1][0] + 
					  Oher[1][1][1] * Delta1 + 		
					  Oher[1][1][2] * Delta1_sqr +
					  Oher[1][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[1][2][0] + 
					  Oher[1][2][1] * Delta1 + 		
					  Oher[1][2][2] * Delta1_sqr +
					  Oher[1][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[1][3][0] + 
					  Oher[1][3][1] * Delta1 + 		
					  Oher[1][3][2] * Delta1_sqr +
					  Oher[1][3][3] * Delta1_cub) * Delta2_cub) * Delta3 + 
					 (Oher[2][0][0] + 
					  Oher[2][0][1] * Delta1 + 		
					  Oher[2][0][2] * Delta1_sqr +
					  Oher[2][0][3] * Delta1_cub + 
					 (Oher[2][1][0] + 
					  Oher[2][1][1] * Delta1 + 		
					  Oher[2][1][2] * Delta1_sqr +
					  Oher[2][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[2][2][0] + 
					  Oher[2][2][1] * Delta1 + 		
					  Oher[2][2][2] * Delta1_sqr +
					  Oher[2][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[2][3][0] + 
					  Oher[2][3][1] * Delta1 + 		
					  Oher[2][3][2] * Delta1_sqr +
					  Oher[2][3][3] * Delta1_cub) * Delta2_cub) * pow(Delta3, 2.) + 
					 (Oher[3][0][0] + 
					  Oher[3][0][1] * Delta1 + 		
					  Oher[3][0][2] * Delta1_sqr +
					  Oher[3][0][3] * Delta1_cub + 
					 (Oher[3][1][0] + 
					  Oher[3][1][1] * Delta1 + 		
					  Oher[3][1][2] * Delta1_sqr +
					  Oher[3][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[3][2][0] + 
					  Oher[3][2][1] * Delta1 + 		
					  Oher[3][2][2] * Delta1_sqr +
					  Oher[3][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[3][3][0] + 
					  Oher[3][3][1] * Delta1 + 		
					  Oher[3][3][2] * Delta1_sqr +
					  Oher[3][3][3] * Delta1_cub) * Delta2_cub) * pow(Delta3, 3.);
}
void math::interp_hermite::interp3(double& result,
										 const math::vec1& points3,
										 const std::vector<math::f_table2V*>& tables,
										 const int& pos3,
										 const math::ratio& rat3,
										 const double& input2,
										 const double& input1,
										 const math::hermite3v&) const {

	// Not implemented

	//math::mag* temp1 = result.clone();
	//math::mag* temp2 = result.clone();
	//	tables[pos3]->value(*temp1, input2, input1);
	//	tables[pos3+1]->value(*temp2, input2, input1);
	//	math::interp::basic_interp_lagrange_first(result, *temp1, *temp2,
	//		static_cast<const math::ratio_linear&>(rat3)._ratio12);
	//delete temp1;
	//delete temp2;
}
void math::interp_hermite::interp3(double& result,
										 const math::vec1& points3,
										 const std::vector<math::f_tabular2V*>& tables,
										 const int& pos3,
										 const math::ratio& rat3,
										 const double& input2,
										 const double& input1,
										 const math::hermite3v& her) const {
	// NOT IMPLEMENTED
    throw std::runtime_error("Function not implemented.");
}
/**< Fills up the input magnitude by interpolating in THREE dimensions based on the
input magnitudes included in the 3D matrix VValues with the lowest positions being
those identified by pos1, pos2, and pos3, and the ratios contained in rat1, rat2,
and rat3. The second version contains the input magnitudes, position, and ratio only
for the third dimension, plus a series of bidimensional tables. The third version 
contains the input magnitudes, position, and ratio only for the third dimension, 
plus a series of bidimensional tabular objects.*/

void math::interp_hermite::interp4(double& result,
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
									   const math::hermite4v& her) const {
	const math::ratio_hermite& rat4_her = static_cast<const math::ratio_hermite&>(rat4);
	const math::ratio_hermite& rat3_her = static_cast<const math::ratio_hermite&>(rat3);
	const math::ratio_hermite& rat2_her = static_cast<const math::ratio_hermite&>(rat2);
	const math::ratio_hermite& rat1_her = static_cast<const math::ratio_hermite&>(rat1);
	double Delta4 = rat4_her._ratio12 * (points4[pos4+1] - points4[pos4]);
	double Delta3 = rat3_her._ratio12 * (points3[pos3+1] - points3[pos3]);
	double Delta2 = rat2_her._ratio12 * (points2[pos2+1] - points2[pos2]);
	double Delta1 = rat1_her._ratio12 * (points1[pos1+1] - points1[pos1]);
	double Delta3_sqr = pow(Delta3, 2.);
	double Delta3_cub = pow(Delta3, 3.);
	double Delta2_sqr = pow(Delta2, 2.);
	double Delta2_cub = pow(Delta2, 3.);
	double Delta1_sqr = pow(Delta1, 2.);
	double Delta1_cub = pow(Delta1, 3.);

	const math::hermite_pow4& Oher = *her[pos4][pos3][pos2][pos1];

	result = (Oher[0][0][0][0] + Oher[0][0][0][1] * Delta1 + Oher[0][0][0][2] * Delta1_sqr + Oher[0][0][0][3] * Delta1_cub +
					 (Oher[0][0][1][0] + Oher[0][0][1][1] * Delta1 + Oher[0][0][1][2] * Delta1_sqr + Oher[0][0][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[0][0][2][0] + Oher[0][0][2][1] * Delta1 + Oher[0][0][2][2] * Delta1_sqr + Oher[0][0][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[0][0][3][0] + Oher[0][0][3][1] * Delta1 + Oher[0][0][3][2] * Delta1_sqr + Oher[0][0][3][3] * Delta1_cub) * Delta2_cub) + 
					 (Oher[0][1][0][0] + Oher[0][1][0][1] * Delta1 + Oher[0][1][0][2] * Delta1_sqr + Oher[0][1][0][3] * Delta1_cub + 
					 (Oher[0][1][1][0] + Oher[0][1][1][1] * Delta1 + Oher[0][1][1][2] * Delta1_sqr + Oher[0][1][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[0][1][2][0] + Oher[0][1][2][1] * Delta1 + Oher[0][1][2][2] * Delta1_sqr + Oher[0][1][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[0][1][3][0] + Oher[0][1][3][1] * Delta1 + Oher[0][1][3][2] * Delta1_sqr + Oher[0][1][3][3] * Delta1_cub) * Delta2_cub) * Delta3 + 
					 (Oher[0][2][0][0] + Oher[0][2][0][1] * Delta1 + Oher[0][2][0][2] * Delta1_sqr + Oher[0][2][0][3] * Delta1_cub + 
					 (Oher[0][2][1][0] + Oher[0][2][1][1] * Delta1 + Oher[0][2][1][2] * Delta1_sqr + Oher[0][2][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[0][2][2][0] + Oher[0][2][2][1] * Delta1 + Oher[0][2][2][2] * Delta1_sqr + Oher[0][2][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[0][2][3][0] + Oher[0][2][3][1] * Delta1 + Oher[0][2][3][2] * Delta1_sqr + Oher[0][2][3][3] * Delta1_cub) * Delta2_cub) * Delta3_sqr + 
					 (Oher[0][3][0][0] + Oher[0][3][0][1] * Delta1 + Oher[0][3][0][2] * Delta1_sqr + Oher[0][3][0][3] * Delta1_cub + 
					 (Oher[0][3][1][0] + Oher[0][3][1][1] * Delta1 + Oher[0][3][1][2] * Delta1_sqr + Oher[0][3][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[0][3][2][0] + Oher[0][3][2][1] * Delta1 + Oher[0][3][2][2] * Delta1_sqr + Oher[0][3][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[0][3][3][0] + Oher[0][3][3][1] * Delta1 + Oher[0][3][3][2] * Delta1_sqr + Oher[0][3][3][3] * Delta1_cub) * Delta2_cub) * Delta3_cub 
					 +
					 ((Oher[1][0][0][0] + Oher[1][0][0][1] * Delta1 + Oher[1][0][0][2] * Delta1_sqr + Oher[1][0][0][3] * Delta1_cub + 
					 (Oher[1][0][1][0] + Oher[1][0][1][1] * Delta1 + Oher[1][0][1][2] * Delta1_sqr + Oher[1][0][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[1][0][2][0] + Oher[1][0][2][1] * Delta1 + Oher[1][0][2][2] * Delta1_sqr + Oher[1][0][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[1][0][3][0] + Oher[1][0][3][1] * Delta1 + Oher[1][0][3][2] * Delta1_sqr + Oher[1][0][3][3] * Delta1_cub) * Delta2_cub) + 
					 (Oher[1][1][0][0] + Oher[1][1][0][1] * Delta1 + Oher[1][1][0][2] * Delta1_sqr + Oher[1][1][0][3] * Delta1_cub + 
					 (Oher[1][1][1][0] + Oher[1][1][1][1] * Delta1 + Oher[1][1][1][2] * Delta1_sqr + Oher[1][1][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[1][1][2][0] + Oher[1][1][2][1] * Delta1 + Oher[1][1][2][2] * Delta1_sqr + Oher[1][1][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[1][1][3][0] + Oher[1][1][3][1] * Delta1 + Oher[1][1][3][2] * Delta1_sqr + Oher[1][1][3][3] * Delta1_cub) * Delta2_cub) * Delta3 + 
					 (Oher[1][2][0][0] + Oher[1][2][0][1] * Delta1 + Oher[1][2][0][2] * Delta1_sqr + Oher[1][2][0][3] * Delta1_cub + 
					 (Oher[1][2][1][0] + Oher[1][2][1][1] * Delta1 + Oher[1][2][1][2] * Delta1_sqr + Oher[1][2][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[1][2][2][0] + Oher[1][2][2][1] * Delta1 + Oher[1][2][2][2] * Delta1_sqr + Oher[1][2][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[1][2][3][0] + Oher[1][2][3][1] * Delta1 + Oher[1][2][3][2] * Delta1_sqr + Oher[1][2][3][3] * Delta1_cub) * Delta2_cub) * Delta3_sqr + 
					 (Oher[1][3][0][0] + Oher[1][3][0][1] * Delta1 + Oher[1][3][0][2] * Delta1_sqr + Oher[1][3][0][3] * Delta1_cub + 
					 (Oher[1][3][1][0] + Oher[1][3][1][1] * Delta1 + Oher[1][3][1][2] * Delta1_sqr + Oher[1][3][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[1][3][2][0] + Oher[1][3][2][1] * Delta1 + Oher[1][3][2][2] * Delta1_sqr + Oher[1][3][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[1][3][3][0] + Oher[1][3][3][1] * Delta1 + Oher[1][3][3][2] * Delta1_sqr + Oher[1][3][3][3] * Delta1_cub) * Delta2_cub) * Delta3_cub) * Delta4 
					 +
					 ((Oher[2][0][0][0] + Oher[2][0][0][1] * Delta1 + Oher[2][0][0][2] * Delta1_sqr + Oher[2][0][0][3] * Delta1_cub + 
					 (Oher[2][0][1][0] + Oher[2][0][1][1] * Delta1 + Oher[2][0][1][2] * Delta1_sqr + Oher[2][0][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[2][0][2][0] + Oher[2][0][2][1] * Delta1 + Oher[2][0][2][2] * Delta1_sqr + Oher[2][0][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[2][0][3][0] + Oher[2][0][3][1] * Delta1 + Oher[2][0][3][2] * Delta1_sqr + Oher[2][0][3][3] * Delta1_cub) * Delta2_cub) + 
					 (Oher[2][1][0][0] + Oher[2][1][0][1] * Delta1 + Oher[2][1][0][2] * Delta1_sqr + Oher[2][1][0][3] * Delta1_cub + 
					 (Oher[2][1][1][0] + Oher[2][1][1][1] * Delta1 + Oher[2][1][1][2] * Delta1_sqr + Oher[2][1][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[2][1][2][0] + Oher[2][1][2][1] * Delta1 + Oher[2][1][2][2] * Delta1_sqr + Oher[2][1][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[2][1][3][0] + Oher[2][1][3][1] * Delta1 + Oher[2][1][3][2] * Delta1_sqr + Oher[2][1][3][3] * Delta1_cub) * Delta2_cub) * Delta3 + 
					 (Oher[2][2][0][0] + Oher[2][2][0][1] * Delta1 + Oher[2][2][0][2] * Delta1_sqr + Oher[2][2][0][3] * Delta1_cub + 
					 (Oher[2][2][1][0] + Oher[2][2][1][1] * Delta1 + Oher[2][2][1][2] * Delta1_sqr + Oher[2][2][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[2][2][2][0] + Oher[2][2][2][1] * Delta1 + Oher[2][2][2][2] * Delta1_sqr + Oher[2][2][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[2][2][3][0] + Oher[2][2][3][1] * Delta1 + Oher[2][2][3][2] * Delta1_sqr + Oher[2][2][3][3] * Delta1_cub) * Delta2_cub) * Delta3_sqr + 
					 (Oher[2][3][0][0] + Oher[2][3][0][1] * Delta1 + Oher[2][3][0][2] * Delta1_sqr + Oher[2][3][0][3] * Delta1_cub + 
					 (Oher[2][3][1][0] + Oher[2][3][1][1] * Delta1 + Oher[2][3][1][2] * Delta1_sqr + Oher[2][3][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[2][3][2][0] + Oher[2][3][2][1] * Delta1 + Oher[2][3][2][2] * Delta1_sqr + Oher[2][3][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[2][3][3][0] + Oher[2][3][3][1] * Delta1 + Oher[2][3][3][2] * Delta1_sqr + Oher[2][3][3][3] * Delta1_cub) * Delta2_cub) * Delta3_cub) * pow(Delta4, 2.) 
					 +
					 ((Oher[3][0][0][0] + Oher[3][0][0][1] * Delta1 + Oher[3][0][0][2] * Delta1_sqr + Oher[3][0][0][3] * Delta1_cub + 
					 (Oher[3][0][1][0] + Oher[3][0][1][1] * Delta1 + Oher[3][0][1][2] * Delta1_sqr + Oher[3][0][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[3][0][2][0] + Oher[3][0][2][1] * Delta1 + Oher[3][0][2][2] * Delta1_sqr + Oher[3][0][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[3][0][3][0] + Oher[3][0][3][1] * Delta1 + Oher[3][0][3][2] * Delta1_sqr + Oher[3][0][3][3] * Delta1_cub) * Delta2_cub) + 
					 (Oher[3][1][0][0] + Oher[3][1][0][1] * Delta1 + Oher[3][1][0][2] * Delta1_sqr + Oher[3][1][0][3] * Delta1_cub + 
					 (Oher[3][1][1][0] + Oher[3][1][1][1] * Delta1 + Oher[3][1][1][2] * Delta1_sqr + Oher[3][1][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[3][1][2][0] + Oher[3][1][2][1] * Delta1 + Oher[3][1][2][2] * Delta1_sqr + Oher[3][1][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[3][1][3][0] + Oher[3][1][3][1] * Delta1 + Oher[3][1][3][2] * Delta1_sqr + Oher[3][1][3][3] * Delta1_cub) * Delta2_cub) * Delta3 + 
					 (Oher[3][2][0][0] + Oher[3][2][0][1] * Delta1 + Oher[3][2][0][2] * Delta1_sqr + Oher[3][2][0][3] * Delta1_cub + 
					 (Oher[3][2][1][0] + Oher[3][2][1][1] * Delta1 + Oher[3][2][1][2] * Delta1_sqr + Oher[3][2][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[3][2][2][0] + Oher[3][2][2][1] * Delta1 + Oher[3][2][2][2] * Delta1_sqr + Oher[3][2][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[3][2][3][0] + Oher[3][2][3][1] * Delta1 + Oher[3][2][3][2] * Delta1_sqr + Oher[3][2][3][3] * Delta1_cub) * Delta2_cub) * Delta3_sqr + 
					 (Oher[3][3][0][0] + Oher[3][3][0][1] * Delta1 + Oher[3][3][0][2] * Delta1_sqr + Oher[3][3][0][3] * Delta1_cub + 
					 (Oher[3][3][1][0] + Oher[3][3][1][1] * Delta1 + Oher[3][3][1][2] * Delta1_sqr + Oher[3][3][1][3] * Delta1_cub) * Delta2 + 
					 (Oher[3][3][2][0] + Oher[3][3][2][1] * Delta1 + Oher[3][3][2][2] * Delta1_sqr + Oher[3][3][2][3] * Delta1_cub) * Delta2_sqr + 
				     (Oher[3][3][3][0] + Oher[3][3][3][1] * Delta1 + Oher[3][3][3][2] * Delta1_sqr + Oher[3][3][3][3] * Delta1_cub) * Delta2_cub) * Delta3_cub) * pow(Delta4, 3.);
}
/* Fills up the input magnitude by interpolating in FOUR dimensions based on the
input magnitudes included in the 4D matrix VVValues with the lowest positions being
those identified by pos1, pos2, pos3, and pos4, and the ratios contained in rat1, rat2,
rat3 and rat4 */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////






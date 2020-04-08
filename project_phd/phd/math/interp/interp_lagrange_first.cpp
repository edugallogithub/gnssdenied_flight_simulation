#include "interp_lagrange_first.h"
#include "../pred/pred2v/f_table2V.h"
#include "../pred/pred1v/f_table1V.h"

// CLASS INTERP_LAGRANGE_FIRST
// ===========================
// ===========================

const math::logic::INTERP_MODE math::interp_lagrange_first::_interp_mode = math::logic::lagrange_first;
/* interpolation mode */

const int math::interp_lagrange_first::_min_points = 2;
/* minimum number of points for interpolation */

/* ===== ===== ===== Ratio Methods ===== ===== ===== */
/* ================================================= */
math::ratio* math::interp_lagrange_first::compute_ratio(const double& input,
															const math::vec1& vec,
															const int& pos) const {
	math::ratio_linear* result = math::ratio_mgr::from_pool_linear();
	result->set_ratio12((input - vec[pos]) / (vec[pos+1] - vec[pos]));
	return result;
}
/* Given an input magnitude, a vector of magnitudes, and the result of the
"find_index" method, it returns a pointer to the appropriate ratio class with
the ratios between the different vector members involved in the interpolation. */

double math::interp_lagrange_first::compute_Dratio(math::ratio& rat,
												 const double& input,
												 const math::vec1& vec,
												 const int& pos) const {
	math::ratio_linear& rat_lin = static_cast<math::ratio_linear&>(rat);
	rat_lin.set_ratio12(rat_lin._ratio12 + 1e-4);
	return 1e-4 * (vec[pos+1] - vec[pos]);
}
/* Given an input magnitude, a vector of magnitudes, and the position provided by
the "find_index" method, it modifies the input ratio object so it can be employed
to compute the differential together with that provided by the "compute_ratio" method.
It computes the values corresponding to the input magnitude plus one ten-thousandth 
(1e-4) of the difference between positions "pos+1" and "pos" of the input vector. 
Returns this thousandth of difference by its later use computing the differential. */

math::ratio* math::interp_lagrange_first::copy_ratio(math::ratio* p) const {
	return math::ratio_mgr::copy(static_cast<const math::ratio_linear&>(*p));
}
/* returns pointer to ratio equal to input */

void math::interp_lagrange_first::to_pool(math::ratio* p) const {
	math::ratio_mgr::to_pool_linear(static_cast<math::ratio_linear*>(p));
}
/* returns the interpolation ratio pointer to the storage so it can be employed again */

/* ===== ===== ===== Interpolation Methods ===== ===== ===== */
/* ========================================================= */
void math::interp_lagrange_first::interp1(double& result,
										const math::vec1&,
										const math::vec1& values,
										const int& pos1,
										const math::ratio& rat1,
										const math::hermite1v&) const {
	math::interp::basic_interp_lagrange_first(result, values, pos1,
					static_cast<const math::ratio_linear&>(rat1)._ratio12);
}
/* Fills up the input magnitude by interpolating in ONE dimension based on the
input magnitudes included in the 1D matrix values with the lowest position being
that identified by pos1, and the ratios contained in rat1 */

void math::interp_lagrange_first::interp2(double& result,
										const math::vec1&,
										const math::vec1&,
										const math::vec2& Values,
										const int& pos2,
										const int& pos1,
										const math::ratio& rat2,
										const math::ratio& rat1,
										const math::hermite2v&) const {
	double temp1, temp2;
	int pos  = Values.index(pos1, pos2); // siz1 * pos2 + pos1;
    math::interp::basic_interp_lagrange_first(temp2,
        Values, pos,
        static_cast<const math::ratio_linear&>(rat1)._ratio12);
    pos += Values.size1();
    math::interp::basic_interp_lagrange_first(temp1,
        Values, pos,
        static_cast<const math::ratio_linear&>(rat1)._ratio12);
    math::interp::basic_interp_lagrange_first(result, temp2, temp1, static_cast<const math::ratio_linear&>(rat2)._ratio12);
}
/* Fills up the input magnitude by interpolating in TWO dimensions based on the
input magnitudes included in the 2D matrix Values with the lowest positions being
those identified by pos1 and pos2, and the ratios contained in rat1 and rat2 */

void math::interp_lagrange_first::interp2(double& result,
										 const math::vec1& points2,
										 const std::vector<math::f_table1V*>& tables,
										 const int& pos2,
										 const math::ratio& rat2,
										 const double& input1,
										 const math::hermite2v&) const {
	double temp1, temp2;
    temp1 = tables[pos2]->value(input1);
    temp2 = tables[pos2+1]->value(input1);
    math::interp::basic_interp_lagrange_first(result, temp1, temp2,
        static_cast<const math::ratio_linear&>(rat2)._ratio12);
}
/**< Fills up the input magnitude by interpolating in TWO dimensions based on the
input magnitudes included in the 2D matrix Values with the lowest positions being
those identified by pos1 and pos2, and the ratios contained in rat1 and rat2.
Contains the input magnitudes, position, and ratio only for the second dimension, 
plus a series of unidimensional tables.*/

void math::interp_lagrange_first::interp3(double& result,
										const math::vec1&,
										const math::vec1&,
										const math::vec1&,
										const math::vec3& VValues,
										const int& pos3,
										const int& pos2,
										const int& pos1,
										const math::ratio& rat3,
										const math::ratio& rat2,
										const math::ratio& rat1,
										const math::hermite3v&) const {
	double temp11, temp12, temp21, temp22;
	int pos  = VValues.index(pos1, pos2, pos3); // _siz12 * pos3 + _siz1 * pos2 + pos1
    math::interp::basic_interp_lagrange_first(temp11,
        VValues, pos,
        static_cast<const math::ratio_linear&>(rat1)._ratio12);
    pos += VValues.size1();
    math::interp::basic_interp_lagrange_first(temp12,
        VValues, pos,
        static_cast<const math::ratio_linear&>(rat1)._ratio12);
    pos  = VValues.index(pos1, pos2, pos3+1);
    math::interp::basic_interp_lagrange_first(temp21,
        VValues, pos,
        static_cast<const math::ratio_linear&>(rat1)._ratio12);
    pos += VValues.size1();
    math::interp::basic_interp_lagrange_first(temp22,
        VValues, pos,
        static_cast<const math::ratio_linear&>(rat1)._ratio12);
    math::interp::basic_interp_lagrange_first(temp11, temp11, temp12, static_cast<const math::ratio_linear&>(rat2)._ratio12);
    math::interp::basic_interp_lagrange_first(temp21, temp21, temp22, static_cast<const math::ratio_linear&>(rat2)._ratio12);
    math::interp::basic_interp_lagrange_first(result, temp11, temp21, static_cast<const math::ratio_linear&>(rat3)._ratio12);
}
void math::interp_lagrange_first::interp3(double& result,
												const math::vec1& points3,
												const std::vector<math::f_table2V*>& tables,
												const int& pos3,
												const math::ratio& rat3,
												const double& input2,
												const double& input1,
												const math::hermite3v&) const {
	double temp1, temp2;
    temp1 = tables[pos3]->value(input2, input1);
    temp2 = tables[pos3+1]->value(input2, input1);
    math::interp::basic_interp_lagrange_first(result, temp1, temp2,
        static_cast<const math::ratio_linear&>(rat3)._ratio12);
}
void math::interp_lagrange_first::interp3(double& result,
												const math::vec1& points3,
												const std::vector<math::f_tabular2V*>& tables,
												const int& pos3,
												const math::ratio& rat3,
												const double& input2,
												const double& input1,
												const math::hermite3v& her) const {
    throw std::runtime_error("Function not implemented.");
}
/**< Fills up the input magnitude by interpolating in THREE dimensions based on the
input magnitudes included in the 3D matrix VValues with the lowest positions being
those identified by pos1, pos2, and pos3, and the ratios contained in rat1, rat2,
and rat3. The second version contains the input magnitudes, position, and ratio only
for the third dimension, plus a series of bidimensional tables. The third version 
contains the input magnitudes, position, and ratio only for the third dimension, 
plus a series of bidimensional tabular objects.*/

void math::interp_lagrange_first::interp4(double& result,
										const math::vec1&,
										const math::vec1&,
										const math::vec1&,
										const math::vec1&,
										const math::vec4& VVValues,
										const int& pos4,
										const int& pos3,
										const int& pos2,
										const int& pos1,
										const math::ratio& rat4,
										const math::ratio& rat3,
										const math::ratio& rat2,
										const math::ratio& rat1,
										const math::hermite4v&) const {
	const math::ratio_linear& rat2_lin = static_cast<const math::ratio_linear&>(rat2);
	const math::ratio_linear& rat1_lin = static_cast<const math::ratio_linear&>(rat1);
	double temp111, temp112, temp121, temp122, temp211, temp212, temp221, temp222;
	int pos  = VVValues.index(pos1,pos2,pos3,pos4); // siz123*pos4 + siz12*pos3 + siz1*pos2 + pos1
	int siz1 = VVValues.size1();  
    math::interp::basic_interp_lagrange_first(temp111,
        VVValues, pos,
        rat1_lin._ratio12);
    pos += siz1;
    math::interp::basic_interp_lagrange_first(temp112,
        VVValues, pos,
        rat1_lin._ratio12);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4);
    math::interp::basic_interp_lagrange_first(temp121,
        VVValues, pos,
        rat1_lin._ratio12);
    pos += siz1;
    math::interp::basic_interp_lagrange_first(temp122,
        VVValues, pos,
        rat1_lin._ratio12);
    pos = VVValues.index(pos1, pos2, pos3, pos4+1);
    math::interp::basic_interp_lagrange_first(temp211,
        VVValues, pos,
        rat1_lin._ratio12);
    pos += siz1;
    math::interp::basic_interp_lagrange_first(temp212,
        VVValues, pos,
        rat1_lin._ratio12);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4+1);
    math::interp::basic_interp_lagrange_first(temp221,
        VVValues, pos,
        rat1_lin._ratio12);
    pos += siz1;
    math::interp::basic_interp_lagrange_first(temp222,
        VVValues, pos,
        rat1_lin._ratio12);
    math::interp::basic_interp_lagrange_first(temp111, temp111, temp112, rat2_lin._ratio12);
    math::interp::basic_interp_lagrange_first(temp121, temp121, temp122, rat2_lin._ratio12);
    math::interp::basic_interp_lagrange_first(temp211, temp211, temp212, rat2_lin._ratio12);
    math::interp::basic_interp_lagrange_first(temp221, temp221, temp222, rat2_lin._ratio12);
    math::interp::basic_interp_lagrange_first(temp111, temp111, temp121, static_cast<const math::ratio_linear&>(rat3)._ratio12);
    math::interp::basic_interp_lagrange_first(temp211, temp211, temp221, static_cast<const math::ratio_linear&>(rat3)._ratio12);
    math::interp::basic_interp_lagrange_first(result, temp111, temp211, static_cast<const math::ratio_linear&>(rat4)._ratio12);
}
/* Fills up the input magnitude by interpolating in FOUR dimensions based on the
input magnitudes included in the 4D matrix VVValues with the lowest positions being
those identified by pos1, pos2, pos3, and pos4, and the ratios contained in rat1, rat2,
rat3 and rat4 */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////






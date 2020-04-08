#include "interp_lagrange_second.h"
#include "../pred/pred2v/f_table2V.h"

// CLASS INTERP_LAGRANGE_SECOND
// ============================
// ============================

const math::logic::INTERP_MODE math::interp_lagrange_second::_interp_mode = math::logic::lagrange_second;
/* interpolation mode */

const int math::interp_lagrange_second::_min_points = 3;
/* minimum number of points for interpolation */

/* ===== ===== ===== Ratio Methods ===== ===== ===== */
/* ================================================= */
math::ratio* math::interp_lagrange_second::compute_ratio(const double& input,
															   const math::vec1& vec,
															   const int& pos) const {
	math::ratio_quadratic* result = math::ratio_mgr::from_pool_quadratic();
	result->set_ratio12((input - vec[pos])   / (vec[pos+1] - vec[pos]));
	result->set_ratio23((input - vec[pos+1]) / (vec[pos+2] - vec[pos+1]));
	result->set_ratio13((input - vec[pos])   / (vec[pos+2] - vec[pos]));
	return result;
}
/* Given an input magnitude, a vector of magnitudes, and the result of the
"find_index" method, it returns a pointer to the appropriate ratio class with
the ratios between the different vector members involved in the interpolation. */

double math::interp_lagrange_second::compute_Dratio(math::ratio& rat,
													const double& input,
													const math::vec1& vec,
													const int& pos) const {
	math::ratio_quadratic& rat_qua = static_cast<math::ratio_quadratic&>(rat);
	double Pmag = input;
	double dif = 1e-4 * (vec[pos+1] - vec[pos]);
	Pmag = Pmag + dif;
	rat_qua.set_ratio12((Pmag - vec[pos])   / (vec[pos+1] - vec[pos]));
	rat_qua.set_ratio23((Pmag - vec[pos+1]) / (vec[pos+2] - vec[pos+1]));
	rat_qua.set_ratio13((Pmag - vec[pos])   / (vec[pos+2] - vec[pos]));
	return dif;
}
/* Given an input magnitude, a vector of magnitudes, and the position provided by
the "find_index" method, it modifies the input ratio object so it can be employed
to compute the differential together with that provided by the "compute_ratio" method.
It computes the values corresponding to the input magnitude plus one ten-thousandth 
(1e-4) of the difference between positions "pos+1" and "pos" of the input vector. 
Returns this thousandth of difference by its later use computing the differential. */

math::ratio* math::interp_lagrange_second::copy_ratio(math::ratio* p) const {
	return math::ratio_mgr::copy(static_cast<const math::ratio_quadratic&>(*p));
}
/* returns pointer to ratio equal to input */

void math::interp_lagrange_second::to_pool(math::ratio* p) const {
	math::ratio_mgr::to_pool_quadratic(static_cast<math::ratio_quadratic*>(p));
}
/* returns the interpolation ratio pointer to the storage so it can be employed again */

/* ===== ===== ===== Interpolation Methods ===== ===== ===== */
/* ========================================================= */
void math::interp_lagrange_second::interp1(double& result,
										   const math::vec1&,
										   const math::vec1& values,
										   const int& pos1,
										   const math::ratio& rat1,
										   const math::hermite1v&) const {
	const math::ratio_quadratic& rat1_qua = static_cast<const math::ratio_quadratic&>(rat1);
	math::interp::basic_interp_lagrange_second(result, values, pos1,
									rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);		
}
/* Fills up the input magnitude by interpolating in ONE dimension based on the
input magnitudes included in the 1D matrix values with the lowest position being
that identified by pos1, and the ratios contained in rat1 */

void math::interp_lagrange_second::interp2(double& result,
										   const math::vec1&,
										   const math::vec1&,
										   const math::vec2& Values,
										   const int& pos2,
										   const int& pos1,
										   const math::ratio& rat2,
										   const math::ratio& rat1,
										   const math::hermite2v&) const {
	const math::ratio_quadratic& rat2_qua = static_cast<const math::ratio_quadratic&>(rat2);
	const math::ratio_quadratic& rat1_qua = static_cast<const math::ratio_quadratic&>(rat1);
	double temp1, temp2, temp3;
	int pos  = Values.index(pos1, pos2); // siz1 * pos2 + pos1;
	int siz1 = Values.size1();  
    math::interp::basic_interp_lagrange_second(temp1,
                        Values, pos,
                        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp2,
                        Values, pos,
                        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp3,
                        Values, pos,
                        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    math::interp::basic_interp_lagrange_second(result,
                        temp1, temp2, temp3,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
}
/* Fills up the input magnitude by interpolating in TWO dimensions based on the
input magnitudes included in the 2D matrix Values with the lowest positions being
those identified by pos1 and pos2, and the ratios contained in rat1 and rat2 */

void math::interp_lagrange_second::interp2(double& result,
											const math::vec1& points2,
											const std::vector<math::f_table1V*>& tables,
											const int& pos2,
											const math::ratio& rat2,
											const double& input1,
											const math::hermite2v& her) const {
	// Not implemented
    throw std::runtime_error("Function not implemented.");
}
/**< Fills up the input magnitude by interpolating in TWO dimensions based on the
input magnitudes included in the 2D matrix Values with the lowest positions being
those identified by pos1 and pos2, and the ratios contained in rat1 and rat2.
Contains the input magnitudes, position, and ratio only for the second dimension, 
plus a series of unidimensional tables.*/
void math::interp_lagrange_second::interp3(double& result,
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
	const math::ratio_quadratic& rat3_qua = static_cast<const math::ratio_quadratic&>(rat3);
	const math::ratio_quadratic& rat2_qua = static_cast<const math::ratio_quadratic&>(rat2);
	const math::ratio_quadratic& rat1_qua = static_cast<const math::ratio_quadratic&>(rat1);
	double temp11, temp12, temp13, temp21, temp22, temp23, temp31, temp32, temp33;
	int pos  = VValues.index(pos1, pos2, pos3); // _siz12 * pos3 + _siz1 * pos2 + pos1
	int siz1 = VValues.size1();  
    math::interp::basic_interp_lagrange_second(temp11,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp12,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp13,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos  = VValues.index(pos1, pos2, pos3+1);
    math::interp::basic_interp_lagrange_second(temp21,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp22,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp23,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos  = VValues.index(pos1, pos2, pos3+2);
    math::interp::basic_interp_lagrange_second(temp31,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp32,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp33,
        VValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp11, temp11, temp12, temp13,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp21, temp21, temp22, temp23,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp31, temp31, temp32, temp33,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(result, temp11, temp21, temp31,
                        rat3_qua._ratio12, rat3_qua._ratio23, rat3_qua._ratio13);
}
void math::interp_lagrange_second::interp3(double& result,
												 const math::vec1& points3,
												 const std::vector<math::f_table2V*>& tables,
												 const int& pos3,
												 const math::ratio& rat3,
												 const double& input2,
												 const double& input1,
												 const math::hermite3v&) const {
	const math::ratio_quadratic& rat3_qua = static_cast<const math::ratio_quadratic&>(rat3);
    double temp1, temp2, temp3;
    temp1 = tables[pos3]->value(input2, input1);
    temp2 = tables[pos3+1]->value(input2, input1);
    temp3 = tables[pos3+2]->value(input2, input1);
    math::interp::basic_interp_lagrange_second(result, temp1, temp2, temp3,
                        rat3_qua._ratio12, rat3_qua._ratio23, rat3_qua._ratio13);
}
void math::interp_lagrange_second::interp3(double& result,
												 const math::vec1& points3,
												 const std::vector<math::f_tabular2V*>& tables,
												 const int& pos3,
												 const math::ratio& rat3,
												 const double& input2,
												 const double& input1,
												 const math::hermite3v& her) const {
	// Not impemented
    throw std::runtime_error("Function not implemented.");;
}
/**< Fills up the input magnitude by interpolating in THREE dimensions based on the
input magnitudes included in the 3D matrix VValues with the lowest positions being
those identified by pos1, pos2, and pos3, and the ratios contained in rat1, rat2,
and rat3. The second version contains the input magnitudes, position, and ratio only
for the third dimension, plus a series of bidimensional tables. The third version 
contains the input magnitudes, position, and ratio only for the third dimension, 
plus a series of bidimensional tabular objects.*/

void math::interp_lagrange_second::interp4(double& result,
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
	const math::ratio_quadratic& rat4_qua = static_cast<const math::ratio_quadratic&>(rat4);
	const math::ratio_quadratic& rat3_qua = static_cast<const math::ratio_quadratic&>(rat3);
	const math::ratio_quadratic& rat2_qua = static_cast<const math::ratio_quadratic&>(rat2);
	const math::ratio_quadratic& rat1_qua = static_cast<const math::ratio_quadratic&>(rat1);
    double temp111, temp112, temp113, temp121, temp122, temp123, temp131, temp132, temp133;
    double temp211, temp212, temp213, temp221, temp222, temp223, temp231, temp232, temp233;
    double temp311, temp312, temp313, temp321, temp322, temp323, temp331, temp332, temp333;

	int pos  = VVValues.index(pos1,pos2,pos3,pos4); // siz123*pos4 + siz12*pos3 + siz1*pos2 + pos1
	int siz1 = VVValues.size1();  
    math::interp::basic_interp_lagrange_second(temp111,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp112,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp113,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4);
    math::interp::basic_interp_lagrange_second(temp121,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp122,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp123,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos = VVValues.index(pos1, pos2, pos3+2, pos4);
    math::interp::basic_interp_lagrange_second(temp131,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp132,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp133,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp111, temp111, temp112, temp113,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp121, temp121, temp122, temp123,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp131, temp131, temp132, temp133,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp111, temp111, temp121, temp131,
                        rat3_qua._ratio12, rat3_qua._ratio23, rat3_qua._ratio13);

    pos = VVValues.index(pos1, pos2, pos3, pos4+1);
    math::interp::basic_interp_lagrange_second(temp211,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp212,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp213,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4+1);
    math::interp::basic_interp_lagrange_second(temp221,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp222,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp223,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos = VVValues.index(pos1, pos2, pos3+2, pos4+1);
    math::interp::basic_interp_lagrange_second(temp231,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp232,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp233,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp211, temp211, temp212, temp113,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp221, temp221, temp222, temp123,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp231, temp231, temp232, temp133,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp211, temp211, temp221, temp231,
                        rat3_qua._ratio12, rat3_qua._ratio23, rat3_qua._ratio13);

    pos = VVValues.index(pos1, pos2, pos3, pos4+2);
    math::interp::basic_interp_lagrange_second(temp311,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp312,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp313,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4+2);
    math::interp::basic_interp_lagrange_second(temp321,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp322,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp323,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos = VVValues.index(pos1, pos2, pos3+2, pos4+2);
    math::interp::basic_interp_lagrange_second(temp331,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp332,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    pos += siz1;
    math::interp::basic_interp_lagrange_second(temp333,
        VVValues, pos,
        rat1_qua._ratio12, rat1_qua._ratio23, rat1_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp311, temp311, temp312, temp313,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp321, temp321, temp322, temp323,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp331, temp331, temp332, temp333,
                        rat2_qua._ratio12, rat2_qua._ratio23, rat2_qua._ratio13);
    math::interp::basic_interp_lagrange_second(temp311, temp311, temp321, temp331,
                        rat3_qua._ratio12, rat3_qua._ratio23, rat3_qua._ratio13);

    math::interp::basic_interp_lagrange_second(result, temp111, temp211, temp311,
                        rat4_qua._ratio12, rat4_qua._ratio23, rat4_qua._ratio13);
}
/* Fills up the input magnitude by interpolating in FOUR dimensions based on the
input magnitudes included in the 4D matrix VVValues with the lowest positions being
those identified by pos1, pos2, pos3, and pos4, and the ratios contained in rat1, rat2,
rat3 and rat4 */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////







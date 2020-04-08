#include "interp_biparabolic.h"
#include "../pred/pred2v/f_table2V.h"

// CLASS INTERP_BIPARABOLIC
// ========================
// ========================

const math::logic::INTERP_MODE math::interp_biparabolic::_interp_mode = math::logic::biparabolic;
/* interpolation mode */

const int math::interp_biparabolic::_min_points = 4;
/* minimum number of points for interpolation */

/* ===== ===== ===== Ratio Methods ===== ===== ===== */
/* ================================================= */
math::ratio* math::interp_biparabolic::compute_ratio(const double& input,
														   const math::vec1& vec,
														   const int& pos) const {
	math::ratio_cubic* result = math::ratio_mgr::from_pool_cubic();
	result->set_ratio12((input - vec[pos])   / (vec[pos+1] - vec[pos]));
	result->set_ratio23((input - vec[pos+1]) / (vec[pos+2] - vec[pos+1]));
	result->set_ratio34((input - vec[pos+2]) / (vec[pos+3] - vec[pos+2]));
	result->set_ratio13((input - vec[pos])   / (vec[pos+2] - vec[pos]));
	result->set_ratio24((input - vec[pos+1]) / (vec[pos+3] - vec[pos+1]));
	result->set_ratio14((input - vec[pos])   / (vec[pos+3] - vec[pos]));
	return result;
}
/* Given an input magnitude, a vector of magnitudes, and the result of the
"find_index" method, it returns a pointer to the appropriate ratio class with
the ratios between the different vector members involved in the interpolation. */

double math::interp_biparabolic::compute_Dratio(math::ratio& rat,
												const double& input,
												const math::vec1& vec,
												const int& pos) const {
	math::ratio_cubic& rat_cub = static_cast<math::ratio_cubic&>(rat);
	double Pmag = input;
	double dif = 1e-4 * (vec[pos+1] - vec[pos]);
	Pmag = Pmag + dif;
	rat_cub.set_ratio12((Pmag - vec[pos])   / (vec[pos+1] - vec[pos]));
	rat_cub.set_ratio23((Pmag - vec[pos+1]) / (vec[pos+2] - vec[pos+1]));
	rat_cub.set_ratio34((Pmag - vec[pos+2]) / (vec[pos+3] - vec[pos+2]));
	rat_cub.set_ratio13((Pmag - vec[pos])   / (vec[pos+2] - vec[pos]));
	rat_cub.set_ratio24((Pmag - vec[pos+1]) / (vec[pos+3] - vec[pos+1]));
	rat_cub.set_ratio14((Pmag - vec[pos])   / (vec[pos+3] - vec[pos]));
	return dif;
}
/* Given an input magnitude, a vector of magnitudes, and the position provided by
the "find_index" method, it modifies the input ratio object so it can be employed
to compute the differential together with that provided by the "compute_ratio" method.
It computes the values corresponding to the input magnitude plus one ten-thousandth 
(1e-4) of the difference between positions "pos+1" and "pos" of the input vector. 
Returns this thousandth of difference by its later use computing the differential. */

math::ratio* math::interp_biparabolic::copy_ratio(math::ratio* p) const {
	return math::ratio_mgr::copy(static_cast<const math::ratio_cubic&>(*p));
}
/* returns pointer to ratio equal to input */

void math::interp_biparabolic::to_pool(math::ratio* p) const {
	math::ratio_mgr::to_pool_cubic(static_cast<math::ratio_cubic*>(p));
}
/* returns the interpolation ratio pointer to the storage so it can be employed again */

/* ===== ===== ===== Interpolation Methods ===== ===== ===== */
/* ========================================================= */
void math::interp_biparabolic::interp1(double& result,
									   const math::vec1&,
									   const math::vec1& values,
									   const int& pos1,
									   const math::ratio& rat1,
									   const math::hermite1v&) const {
	const math::ratio_cubic& rat1_cub = static_cast<const math::ratio_cubic&>(rat1);
	math::interp::basic_interp_biparabolic(result, values, pos1,
						rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
						rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);									
}
/* Fills up the input magnitude by interpolating in ONE dimension based on the
input magnitudes included in the 1D matrix values with the lowest position being
that identified by pos1, and the ratios contained in rat1 */

void math::interp_biparabolic::interp2(double& result,
										const math::vec1&,
										const math::vec1&,
										const math::vec2& Values,
										const int& pos2,
										const int& pos1,
										const math::ratio& rat2,
										const math::ratio& rat1,
										const math::hermite2v&) const {
	const math::ratio_cubic& rat2_cub = static_cast<const math::ratio_cubic&>(rat2);
	const math::ratio_cubic& rat1_cub = static_cast<const math::ratio_cubic&>(rat1);
	double temp1, temp2, temp3, temp4;
	int pos  = Values.index(pos1, pos2); // siz1 * pos2 + pos1;
	int siz1 = Values.size1();  
    math::interp::basic_interp_biparabolic(temp1,
                        Values, pos,
                        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp2,
                        Values, pos,
                        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp3,
                        Values, pos,
                        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp4,
                        Values,
                        pos,
                        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    math::interp::basic_interp_biparabolic(result,
                        temp1, temp2, temp3, temp4,
                        rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                        rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
}
/* Fills up the input magnitude by interpolating in TWO dimensions based on the
input magnitudes included in the 2D matrix Values with the lowest positions being
those identified by pos1 and pos2, and the ratios contained in rat1 and rat2 */

void math::interp_biparabolic::interp2(double& result,
										const math::vec1& points2,
										const std::vector<math::f_table1V*>& tables,
										const int& pos2,
										const math::ratio& rat2,
										const double& input1,
										const math::hermite2v& her) const
{
	// Not implemented
	throw std::runtime_error("Function not implemented.");
}
/**< Fills up the input magnitude by interpolating in TWO dimensions based on the
input magnitudes included in the 2D matrix Values with the lowest positions being
those identified by pos1 and pos2, and the ratios contained in rat1 and rat2.
Contains the input magnitudes, position, and ratio only for the second dimension, 
plus a series of unidimensional tables.*/
void math::interp_biparabolic::interp3(double& result,
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
	const math::ratio_cubic& rat3_cub = static_cast<const math::ratio_cubic&>(rat3);
	const math::ratio_cubic& rat2_cub = static_cast<const math::ratio_cubic&>(rat2);
	const math::ratio_cubic& rat1_cub = static_cast<const math::ratio_cubic&>(rat1);
	double temp11, temp12, temp13, temp14, temp21, temp22, temp23, temp24;
    double temp31, temp32, temp33, temp34, temp41, temp42, temp43, temp44;
	int pos  = VValues.index(pos1, pos2, pos3); // _siz12 * pos3 + _siz1 * pos2 + pos1
	int siz1 = VValues.size1();  
    math::interp::basic_interp_biparabolic(temp11,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp12,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp13,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp14,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);

    pos = VValues.index(pos1, pos2, pos3+1);
    math::interp::basic_interp_biparabolic(temp21,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp22,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp23,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp24,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);

    pos = VValues.index(pos1, pos2, pos3+2);
    math::interp::basic_interp_biparabolic(temp31,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp32,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp33,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp34,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);

    pos = VValues.index(pos1, pos2, pos3+3);
    math::interp::basic_interp_biparabolic(temp41,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp42,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp43,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp44,
                VValues, pos,
                rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
                rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);

    math::interp::basic_interp_biparabolic(temp11, temp11, temp12, temp13, temp14,
                        rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                        rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp21, temp21, temp22, temp23, temp24,
                        rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                        rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp31, temp31, temp32, temp33, temp34,
                        rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                        rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp41, temp41, temp42, temp43, temp44,
                        rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                        rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);

    math::interp::basic_interp_biparabolic(result, temp11, temp21, temp31, temp41,
                        rat3_cub._ratio12, rat3_cub._ratio23, rat3_cub._ratio34,
                        rat3_cub._ratio13, rat3_cub._ratio24, rat3_cub._ratio14);
}
void math::interp_biparabolic::interp3(double& result,
												const math::vec1& points3,
												const std::vector<math::f_table2V*>& tables,
												const int& pos3,
												const math::ratio& rat3,
												const double& input2,
												const double& input1,
												const math::hermite3v&) const {
	const math::ratio_cubic& rat3_cub = static_cast<const math::ratio_cubic&>(rat3);
	double temp1, temp2, temp3, temp4;
    temp1 = tables[pos3]->value(input2, input1);
    temp2 = tables[pos3+1]->value(input2, input1);
    temp3 = tables[pos3+2]->value(input2, input1);
    temp4 = tables[pos3+3]->value(input2, input1);
    math::interp::basic_interp_biparabolic(result, temp1, temp2, temp3, temp4,
                        rat3_cub._ratio12, rat3_cub._ratio23, rat3_cub._ratio34,
                        rat3_cub._ratio13, rat3_cub._ratio24, rat3_cub._ratio14);
}
void math::interp_biparabolic::interp3(double& result,
											 const math::vec1& points3,
											 const std::vector<math::f_tabular2V*>& tables,
											 const int& pos3,
											 const math::ratio& rat3,
											 const double& input2,
											 const double& input1,
											 const math::hermite3v& her) const
{
	// Not implemented
	throw std::runtime_error("Function not implemented.");
}
/**< Fills up the input magnitude by interpolating in THREE dimensions based on the
input magnitudes included in the 3D matrix VValues with the lowest positions being
those identified by pos1, pos2, and pos3, and the ratios contained in rat1, rat2,
and rat3. The second version contains the input magnitudes, position, and ratio only
for the third dimension, plus a series of bidimensional tables. The third version 
contains the input magnitudes, position, and ratio only for the third dimension, 
plus a series of bidimensional tabular objects.*/

void math::interp_biparabolic::interp4(double& result,
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
	const math::ratio_cubic& rat4_cub = static_cast<const math::ratio_cubic&>(rat4);
	const math::ratio_cubic& rat3_cub = static_cast<const math::ratio_cubic&>(rat3);
	const math::ratio_cubic& rat2_cub = static_cast<const math::ratio_cubic&>(rat2);
	const math::ratio_cubic& rat1_cub = static_cast<const math::ratio_cubic&>(rat1);

    double temp111, temp112, temp113, temp114, temp121, temp122, temp123, temp124;
    double temp131, temp132, temp133, temp134, temp141, temp142, temp143, temp144;
    double temp211, temp212, temp213, temp214, temp221, temp222, temp223, temp224;
    double temp231, temp232, temp233, temp234, temp241, temp242, temp243, temp244;
    double temp311, temp312, temp313, temp314, temp321, temp322, temp323, temp324;
    double temp331, temp332, temp333, temp334, temp341, temp342, temp343, temp344;
    double temp411, temp412, temp413, temp414, temp421, temp422, temp423, temp424;
    double temp431, temp432, temp433, temp434, temp441, temp442, temp443, temp444;

    int pos  = VVValues.index(pos1,pos2,pos3,pos4); // siz123*pos4 + siz12*pos3 + siz1*pos2 + pos1
	int siz1 = VVValues.size1();  

    math::interp::basic_interp_biparabolic(temp111,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp112,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp113,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp114,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4);
    math::interp::basic_interp_biparabolic(temp121,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp122,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp123,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp124,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+2, pos4);
    math::interp::basic_interp_biparabolic(temp131,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp132,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp133,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp134,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+3, pos4);
    math::interp::basic_interp_biparabolic(temp141,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp142,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp143,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp144,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp111, temp111, temp112, temp113, temp114,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp121, temp121, temp122, temp123, temp124,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp131, temp131, temp132, temp133, temp134,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp141, temp141, temp142, temp143, temp144,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp111, temp111, temp121, temp131, temp141,
                                                     rat3_cub._ratio12, rat3_cub._ratio23, rat3_cub._ratio34,
                                                     rat3_cub._ratio13, rat3_cub._ratio24, rat3_cub._ratio14);

    pos = VVValues.index(pos1, pos2, pos3, pos4+1);
    math::interp::basic_interp_biparabolic(temp211,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp212,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp213,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp214,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4+1);
    math::interp::basic_interp_biparabolic(temp221,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp222,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp223,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp224,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+2, pos4+1);
    math::interp::basic_interp_biparabolic(temp231,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp232,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp233,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp234,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+3, pos4+1);
    math::interp::basic_interp_biparabolic(temp241,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp242,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp243,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp244,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp211,temp211, temp212, temp213, temp214,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp221, temp221, temp222, temp223, temp224,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp231, temp231, temp232, temp233, temp234,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp241, temp241, temp242, temp243, temp244,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp211, temp211, temp221, temp231, temp241,
                                                     rat3_cub._ratio12, rat3_cub._ratio23, rat3_cub._ratio34,
                                                     rat3_cub._ratio13, rat3_cub._ratio24, rat3_cub._ratio14);

    pos = VVValues.index(pos1, pos2, pos3, pos4+2);
    math::interp::basic_interp_biparabolic(temp311,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp312,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp313,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp314,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4+2);
    math::interp::basic_interp_biparabolic(temp321,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp322,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp323,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp324,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+2, pos4+2);
    math::interp::basic_interp_biparabolic(temp331,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp332,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp333,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp334,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+3, pos4+2);
    math::interp::basic_interp_biparabolic(temp341,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp342,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp343,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp344,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp311, temp311, temp312, temp313, temp314,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp321, temp321, temp322, temp323, temp324,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp331, temp331, temp332, temp333, temp334,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp341, temp341, temp342, temp343, temp344,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp311, temp311, temp321, temp331, temp341,
                                                     rat3_cub._ratio12, rat3_cub._ratio23, rat3_cub._ratio34,
                                                     rat3_cub._ratio13, rat3_cub._ratio24, rat3_cub._ratio14);

    pos = VVValues.index(pos1, pos2, pos3, pos4+3);
    math::interp::basic_interp_biparabolic(temp411,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp412,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp413,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp414,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+1, pos4+3);
    math::interp::basic_interp_biparabolic(temp421,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp422,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp423,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp424,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+2, pos4+3);
    math::interp::basic_interp_biparabolic(temp431,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp432,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp433,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp434,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos = VVValues.index(pos1, pos2, pos3+3, pos4+3);
    math::interp::basic_interp_biparabolic(temp441,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp442,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp443,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    pos += siz1;
    math::interp::basic_interp_biparabolic(temp444,
        VVValues, pos,
        rat1_cub._ratio12, rat1_cub._ratio23, rat1_cub._ratio34,
        rat1_cub._ratio13, rat1_cub._ratio24, rat1_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp411, temp411, temp412, temp413, temp414,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp421, temp421, temp422, temp423, temp424,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp431, temp431, temp432, temp433, temp434,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp441, temp441, temp442, temp443, temp444,
                                                     rat2_cub._ratio12, rat2_cub._ratio23, rat2_cub._ratio34,
                                                     rat2_cub._ratio13, rat2_cub._ratio24, rat2_cub._ratio14);
    math::interp::basic_interp_biparabolic(temp411, temp411, temp421, temp431, temp441,
                                                     rat3_cub._ratio12, rat3_cub._ratio23, rat3_cub._ratio34,
                                                     rat3_cub._ratio13, rat3_cub._ratio24, rat3_cub._ratio14);

    math::interp::basic_interp_biparabolic(result, temp111, temp211, temp311, temp411,
                                                     rat4_cub._ratio12, rat4_cub._ratio23, rat4_cub._ratio34,
                                                     rat4_cub._ratio13, rat4_cub._ratio24, rat4_cub._ratio14);
}
/* fills up the input magnitude by interpolating in four dimensions based on the
input magnitudes included in the 4d matrix VVValues with the lowest positions being
those identified by pos1, pos2, pos3, and pos4, and the ratios contained in rat1, rat2,
rat3 and rat4 */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////






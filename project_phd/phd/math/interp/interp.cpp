#include "interp.h"

#include "interp_lagrange_first.h" 
#include "interp_lagrange_second.h" 
#include "interp_lagrange_third.h" 
#include "interp_biparabolic.h" 
#include "interp_hermite.h"
#include "interp_spline.h"


// CLASS INTERP
// ============
// ============

math::interp* math::interp::get_interp(math::logic::INTERP_MODE interp_mode) {
	switch (interp_mode) {
		case math::logic::lagrange_first_precompute:
		case math::logic::lagrange_first:
			return new math::interp_lagrange_first();
			break;
		case math::logic::lagrange_second:
			return new math::interp_lagrange_second();
			break;
		case math::logic::lagrange_third:
			return new math::interp_lagrange_third();
			break;
		case math::logic::biparabolic:
			return new math::interp_biparabolic();
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			return new math::interp_hermite(interp_mode);
			break;
		case math::logic::spline:
			return new math::interp_spline();
			break;
		default:
			throw std::runtime_error("Incorrect interpolation method.");
	}				
}
/* return pointer to interpolation method that corresponds to input enumeration.
In the case of splines it needs to be followed by the complete_spline method */

math::interp* math::interp::get_interp(math::logic::INTERP_MODE interp_mode,
												   const math::vec1& points1,
												   const math::vec1& values) {
	switch (interp_mode) {
		case math::logic::lagrange_first_precompute:
		case math::logic::lagrange_first:
			return new math::interp_lagrange_first();
			break;
		case math::logic::lagrange_second:
			return new math::interp_lagrange_second();
			break;
		case math::logic::lagrange_third:
			return new math::interp_lagrange_third();
			break;
		case math::logic::biparabolic:
			return new math::interp_biparabolic();
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			return new math::interp_hermite(interp_mode);
			break;
		case math::logic::spline:
			return new math::interp_spline(points1, values, 0);
			break;
		default:
			throw std::runtime_error("Incorrect interpolation method.");
	}			
}
/* return pointer to interpolation method that corresponds to input enumeration.
In the case of splines it also needs two same size vectors of magnitudes, one
with the inputs, another with the outputs */

math::interp* math::interp::get_interp(math::logic::INTERP_MODE interp_mode,
												   const math::vec1& points2,
												   const math::vec1& points1,
												   const math::vec2& Values) {
	switch (interp_mode) {
		case math::logic::lagrange_first_precompute:
		case math::logic::lagrange_first:
			return new math::interp_lagrange_first();
			break;
		case math::logic::lagrange_second:
			return new math::interp_lagrange_second();
			break;
		case math::logic::lagrange_third:
			return new math::interp_lagrange_third();
			break;
		case math::logic::biparabolic:
			return new math::interp_biparabolic();
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			return new math::interp_hermite(interp_mode);
			break;
		case math::logic::spline:
			return new math::interp_spline(points2, points1, Values);
			break;
		default:
			throw std::runtime_error("Incorrect interpolation method.");
	}													   													   
}
/* return pointer to interpolation method that corresponds to input enumeration.
In the case of splines it also needs two vectors of magnitudes with the inputs and
one with the outputs */

const std::string math::interp::describe_interpolation(math::logic::INTERP_MODE index) {
	switch(index) {
		case math::logic::lagrange_first_precompute:	return ("lagrange_first_precompute");	break;
		case math::logic::lagrange_first:				return ("lagrange_first");				break;
		case math::logic::lagrange_second:			return ("lagrange_second");				break;
		case math::logic::lagrange_third:				return ("lagrange_third");				break;
		case math::logic::biparabolic:				return ("biparabolic");					break;
		case math::logic::hermite_first:				return ("hermite_first");				break;
		case math::logic::hermite_second:				return ("hermite_second");				break;
		case math::logic::spline:						return ("spline");						break;
		default:						
			throw std::runtime_error("Incorrect interpolation method.");
			return("");
			break; 
	}	
}
/* returns a string containing the name of the interpolation mode based on the
INTERP_MODE enumeration */

const math::logic::INTERP_MODE math::interp::reverse_describe_interpolation(std::string& st) {
	if		(st == "lagrange_first_precompute") {return math::logic::lagrange_first_precompute;}
	else if	(st == "lagrange_first"           ) {return math::logic::lagrange_first;}
	else if	(st == "lagrange_second"          ) {return math::logic::lagrange_second;}
	else if	(st == "lagrange_third"           ) {return math::logic::lagrange_third;}
	else if (st == "biparabolic"              ) {return math::logic::biparabolic;}
	else if (st == "hermite_first"            ) {return math::logic::hermite_first;}
	else if (st == "hermite_second"           ) {return math::logic::hermite_second;}
	else if	(st == "spline"                   ) {return math::logic::spline;}
	else {
		throw std::runtime_error("Incorrect interpolation method.");
		return math::logic::lagrange_first;
	}	
}
/* returns the name of the interpolation mode in the INTERP_MODE
enumeration based on a string describing it */

/* ===== ===== ===== Generic Static Methods ===== ===== ===== */
/* ========================================================== */
void math::interp::basic_interp_lagrange_first(double& result,
											 const double& op1,
											 const double& op2,
											 const double& ratio) {
	result = op1 - (op1 - op2) * ratio;
}
void math::interp::basic_interp_lagrange_first(double& result,
											const math::vec& Values,
											const int& Vb,
											const double& ratio) {
	result = Values[Vb] - (Values[Vb] - Values[Vb+1]) * ratio;
}
/* fills up the input magnitude result with the linear interpolation between
the input magnitudes op1 and op2, based on the input ratio (interpolation if 
ratio between 0 and 1, extrapolation otherwise). ratio = 0 returns op1 while
ratio = 1 returns op2. Unpredictable results if the three input magnitudes do 
not belong to the same derivate class. Instead of four magnitudes, it is also
possible to provide a multidimensional vector of magnitudes plus a position, 
taking two consecutive magnitudes starting at that position. */

void math::interp::basic_interp_lagrange_second(double& result,
												const double& op1,
												const double& op2,
												const double& op3,
												const double& ratio12,
												const double& ratio23,
												const double& ratio13) {										
	double op12, op23;
	basic_interp_lagrange_first(op12, op1, op2, ratio12);
	basic_interp_lagrange_first(op23, op2, op3, ratio23);
	basic_interp_lagrange_first(result, op12, op23, ratio13);
}
void math::interp::basic_interp_lagrange_second(double& result,
													  const math::vec& VValues,
													  const int& Vb,
													  const double& ratio12,
													  const double& ratio23,
													  const double& ratio13) {
	double op12, op23;
	basic_interp_lagrange_first(op12, VValues, Vb,   ratio12);
	basic_interp_lagrange_first(op23, VValues, Vb+1, ratio23);
	basic_interp_lagrange_first(result, op12, op23, ratio13);
}
/* fills up the input magnitude result with the quadratic interpolation between
the input magnitudes op1, op2, and op3, based on the input ratios (interpolation if 
ratio between 0 and 1, extrapolation otherwise) between each two of them. Unpredictable
results if the four input magnitudes do not belong to the same derivate class.
Instead of four magnitudes, it is also possible to provide a multidimensional vector
of magnitudes plus a position, taking three consecutive magnitudes
starting at that position. */

void math::interp::basic_interp_lagrange_third(double& result,
											const double& op1,
											const double& op2,
											const double& op3,
											const double& op4,
											const double& ratio12,
											const double& ratio23,
											const double& ratio34,
											const double& ratio13,
											const double& ratio24,
											const double& ratio14) {
	double op12, op23, op34;
	math::interp::basic_interp_lagrange_first(op12, op1, op2, ratio12);
	math::interp::basic_interp_lagrange_first(op23, op2, op3, ratio23);
	math::interp::basic_interp_lagrange_first(op34, op3, op4, ratio34);
	math::interp::basic_interp_lagrange_first(op12, op12, op23, ratio13);	// op12 is reused as op13
	math::interp::basic_interp_lagrange_first(op23, op23, op34, ratio24);	// op23 is resused as op24
	math::interp::basic_interp_lagrange_first(result, op12, op23, ratio14);
}
void math::interp::basic_interp_lagrange_third(double& result,
											 const math::vec& VVValues,
											 const int& Vb,
											 const double& ratio12,
											 const double& ratio23,
											 const double& ratio34,
											 const double& ratio13,
											 const double& ratio24,
											 const double& ratio14) {
    double op12, op23, op34;
	math::interp::basic_interp_lagrange_first(op12, VVValues, Vb,   ratio12);
	math::interp::basic_interp_lagrange_first(op23, VVValues, Vb+1, ratio23);
	math::interp::basic_interp_lagrange_first(op34, VVValues, Vb+2, ratio34);
	math::interp::basic_interp_lagrange_first(op12, op12, op23, ratio13);	// op12 is reused as op13
	math::interp::basic_interp_lagrange_first(op23, op23, op34, ratio24);	// op23 is resused as op24
	math::interp::basic_interp_lagrange_first(result, op12, op23, ratio14);
}
/* fills up the input magnitude result with the cubic interpolation between
the input magnitudes op1, op2, op3, and op4, based on the input ratios (interpolation if 
ratio between 0 and 1, extrapolation otherwise) between each two of them. Unpredictable
results if the five input magnitudes do not belong to the same mag derivate class.
Instead of four magnitudes, it is also possible to provide a multidimensional vector
of magnitudes plus a position, taking four consecutive magnitudes
starting at that position. */

void math::interp::basic_interp_lagrange_third_diff(double& diff_first,
										double& diff_last,
										const math::vec1& inp,
										const math::vec& out,
										const unsigned short& b) {
	unsigned short n = inp.size1() - 1;
	unsigned short e = b + n;
	// b indicates the 1st position within the out vector (corresponds to inp[0])
	
	diff_first = out[b] / (inp[0] - inp[1]) +
				 out[b] / (inp[0] - inp[2]) +
				 out[b] / (inp[0] - inp[3]) +
				 out[b+1] * (inp[0] - inp[2]) * (inp[0] - inp[3]) / (inp[1] - inp[0]) / (inp[1] - inp[2]) / (inp[1] - inp[3]) +
				 out[b+2] * (inp[0] - inp[1]) * (inp[0] - inp[3]) / (inp[2] - inp[0]) / (inp[2] - inp[1]) / (inp[2] - inp[3]) +
				 out[b+3] * (inp[0] - inp[1]) * (inp[0] - inp[2]) / (inp[3] - inp[0]) / (inp[3] - inp[1]) / (inp[3] - inp[2]);
	diff_last  = out[e] / (inp[n] - inp[n-3]) +
				 out[e] / (inp[n] - inp[n-2]) +
				 out[e] / (inp[n] - inp[n-1]) +
				 out[e-3] * (inp[n] - inp[n-2]) * (inp[n] - inp[n-1]) / (inp[n-3] - inp[n-2]) / (inp[n-3] - inp[n-1]) / (inp[n-3] - inp[n]) +
				 out[e-2] * (inp[n] - inp[n-3]) * (inp[n] - inp[n-1]) / (inp[n-2] - inp[n-3]) / (inp[n-2] - inp[n-1]) / (inp[n-2] - inp[n]) +
				 out[e-1] * (inp[n] - inp[n-3]) * (inp[n] - inp[n-2]) / (inp[n-1] - inp[n-3]) / (inp[n-1] - inp[n-2]) / (inp[n-1] - inp[n]);
}
/* Given an input vector inp, an output vector out of equal or greater size, and
a position within the output vector that corresponds to the first position in the
input, the function fills up the differential at the first and final position of
the input assumming a cubic interpolation. Required for the computation
of splines. */

void math::interp::basic_interp_biparabolic(double& result,
											const double& op1,
											const double& op2,
											const double& op3,
											const double& op4,
											const double& ratio12,
											const double& ratio23,
											const double& ratio34,
											const double& ratio13,
											const double& ratio24,
											const double& ratio14) {
	if (ratio23 < 0.) {
		basic_interp_lagrange_second(result, op1, op2, op3, ratio12, ratio23, ratio13);
	}
	else if (ratio23 > 1.) {
		basic_interp_lagrange_second(result, op2, op3, op4, ratio23, ratio34, ratio24);
	}
	else {
		double op123, op234;
		basic_interp_lagrange_second(op123, op1, op2, op3, ratio12, ratio23, ratio13);
		basic_interp_lagrange_second(op234, op2, op3, op4, ratio23, ratio34, ratio24);
		basic_interp_lagrange_first(result, op123, op234, ratio23);
	}
}
void math::interp::basic_interp_biparabolic(double& result,
											 const math::vec& VVValues,
											 const int& Vb,
											 const double& ratio12,
											 const double& ratio23,
											 const double& ratio34,
											 const double& ratio13,
											 const double& ratio24,
											 const double& ratio14) {
	if (ratio23 < 0.) {
		basic_interp_lagrange_second(result, VVValues, Vb, ratio12, ratio23, ratio13);
	}
	else if (ratio23 > 1.) {
		basic_interp_lagrange_second(result, VVValues, Vb+1, ratio23, ratio34, ratio24);
	}
	else {
		double op123, op234;
		basic_interp_lagrange_second(op123, VVValues, Vb,   ratio12, ratio23, ratio13);
		basic_interp_lagrange_second(op234, VVValues, Vb+1, ratio23, ratio34, ratio24);
		basic_interp_lagrange_first(result, op123, op234, ratio23);
	}
}
/* fills up the input magnitude result with the cubic interpolation between
the input magnitudes op1, op2, op3, and op4, based on the input ratios (interpolation if 
ratio between 0 and 1, extrapolation otherwise) between each two of them. Unpredictable
results if the five input magnitudes do not belong to the same mag derivate class.
Instead of four magnitudes, it is also possible to provide a multidimensional vector
of magnitudes plus a position, taking four consecutive magnitudes
starting at that position. */

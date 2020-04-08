#include "interp_spline.h"

// CLASS INTERP_SPLINE
// ===================
// ===================

const math::logic::INTERP_MODE math::interp_spline::_interp_mode = math::logic::spline;
/* interpolation mode */

const int math::interp_spline::_min_points = 4;
/* minimum number of points for interpolation */

const int math::interp_spline::_points_bracket = 2;
/* number of points to compute the ratios */

math::interp_spline::interp_spline()
: _diffdiff(0), _dist(0), _spline(0), _points2(0) {
}
/* empty constructor (must be followed by complete_spline method) */

math::interp_spline::interp_spline(const math::vec1& points1,
										 const math::vec& values,
										 const unsigned short& values_begin)
: _diffdiff(0), _dist(0), _spline(0), _points2(0) {
	this->complete_spline(points1, values, values_begin);
}											 
/* constructor based on an input vector, and output vector of equal or
greater size, and a position within the output vector that corresponds to 
the first position in the input. Employed for one dimensional 
interpolation. */

math::interp_spline::interp_spline(const math::vec1& points2,
										 const math::vec1& points1,
										 const math::vec2& Values)
: _diffdiff(0), _dist(0), _spline(0), _points2(0) {
	this->complete_spline(points2, points1, Values);
}									 
/* constructor based on two input vectors, the first or size n and the second
of size m, plues an output matrix of size nxm. Employed for two dimensional 
interpolation. */

void math::interp_spline::complete_spline(const math::vec1& points1,
											  const math::vec& values,
											  const unsigned short& b) {	
	// b indicates the 1st position within the out vector (corresponds to inp[0])
	_diffdiff = new std::vector<double>(points1.size1(), 0.);

	double yp1, ypn;
	math::interp::basic_interp_lagrange_third_diff(yp1, ypn, points1, values, b);
											 
	int n = points1.size1();
	std::vector<double> u(n - 1, 0.);		
	
	(*_diffdiff)[0] = -0.5;
	u[0]  = (3.0 / (points1.diff(1,0))) * ((values.diff(b+1,b)) /	(points1.diff(1,0)) - yp1);

	double sig = 0., p = 0.;
	for (int i = 1; i < (n-1); i++) {
		sig = (points1.diff(i, i-1)) / (points1.diff(i+1, i-1));
		p = sig * (*_diffdiff)[i-1] + 2.0;
		(*_diffdiff)[i] = (sig - 1.0) / p;
		u[i] = (values.diff(b+i+1, b+i)) / (points1.diff(i+1, i)) - 
			   (values.diff(b+i, b+i-1)) / (points1.diff(i, i-1));
		u[i] = (6.0 * u[i] / (points1.diff(i+1, i-1)) - sig * u[i-1]) / p;
	}
	double qn = 0., un = 0.;

	qn = 0.5;
	un = (3.0 / (points1.diff(n-1, n-2))) * (ypn - (values.diff(b+n-1, b+n-2)) /
		 (points1.diff(n-1, n-2)));
	
	(*_diffdiff)[n-1] = (un - qn * u[n-2]) / (qn * (*_diffdiff)[n-2] + 1.0);
	for (int k = n-2; k >= 0; k--) {
		(*_diffdiff)[k] = (*_diffdiff)[k] * (*_diffdiff)[k+1] + u[k];
	}		
}
/* constructor based on an input vector, and output vector of equal or
greater size, and a position within the output vector that corresponds
to the first position in the input. Employed for one dimensional 
interpolation. DOES NOT LIBERATE MEMORY, SO USE ONLY USE AFTER
CONSTRUCTOR. */

void math::interp_spline::complete_spline(const math::vec1& points2,
												const math::vec1& points1,
												const math::vec2& Values) {
	_points2 = &points2;	
	_spline = new std::vector<math::interp_spline*>(points1.size1());
	for (int i = 0; i != _spline->size(); ++i) {
		//(*_spline)[i] = new math::interp_spline(points2, Values(i));
		(*_spline)[i] = new math::interp_spline(points2, Values, Values.size1() * i);
		(*_spline)[i]->set_diffs(points2);
	}
}
/* complements the emtpy constructor based on two input vectors, the first of
size n and the second of size m, plues an output matrix of size nxm. Employed for
two dimensional interpolation. DOES NOT LIBERATE MEMORY, SO USE ONLY USE AFTER
CONSTRUCTOR. */

math::interp_spline::~interp_spline() {
	delete _diffdiff;
	delete _dist;	
	if (_spline != 0) {
		for (int i = 0; i != _spline->size(); ++i) {
			delete (*_spline)[i];
		}
	}
	delete _spline;
}
/* destructor */

void math::interp_spline::set_diffs(const double points1_diff) {
	delete _dist;
	_dist = new std::vector<double>(_diffdiff->size(), points1_diff);	
}
/* sets the _diff attribute when the input vector is equispaced, so all members
of the _diff vector are equal to each other. Employed for one dimensional 
interpolation. */

void math::interp_spline::set_diffs(const math::vec1& points1) {
	delete _dist;
	_dist = new std::vector<double>(_diffdiff->size() - 1, 0.);
	for (int i = 0; i != _dist->size(); ++i) {
		(*_dist)[i] = points1.diff(i+1, i);
	}
}
/* sets the _diff attribute when the input vector is NOT equispaced, so all
members of the _diff vector differ from each other. Employed for one dimensional 
interpolation.*/

/* ===== ===== ===== Ratio Methods ===== ===== ===== */
/* ================================================= */
math::ratio* math::interp_spline::compute_ratio(const double& input,
															const math::vec1& vec,
															const int& pos) const {
	math::ratio_spline* result = math::ratio_mgr::from_pool_spline();
	result->set_ratio12((input - vec[pos]) / (vec[pos+1] - vec[pos]));
	return result;
}
/* Given an input magnitude, a vector of magnitudes, and the result of the
"find_index" method, it returns a pointer to the appropriate ratio class with
the ratios between the different vector members involved in the interpolation. */

double math::interp_spline::compute_Dratio(math::ratio& rat,
												 const double& input,
												 const math::vec1& vec,
												 const int& pos) const {
	math::ratio_spline& rat_spl = static_cast<math::ratio_spline&>(rat);
	rat_spl.set_ratio12(rat_spl._ratio12 + 1e-4);
	return 1e-4 * (vec[pos+1] - vec[pos]);
}
/* Given an input magnitude, a vector of magnitudes, and the position provided by
the "find_index" method, it modifies the input ratio object so it can be employed
to compute the differential together with that provided by the "compute_ratio" method.
It computes the values corresponding to the input magnitude plus one ten-thousandth 
(1e-4) of the difference between positions "pos+1" and "pos" of the input vector. 
Returns this thousandth of difference by its later use computing the differential. */

math::ratio* math::interp_spline::copy_ratio(math::ratio* p) const {
	return math::ratio_mgr::copy(static_cast<const math::ratio_spline&>(*p));
}
/* returns pointer to ratio equal to input */

void math::interp_spline::to_pool(math::ratio* p) const {
	math::ratio_mgr::to_pool_spline(static_cast<math::ratio_spline*>(p));
}
/* returns the interpolation ratio pointer to the storage so it can be employed again */

/* ===== ===== ===== Interpolation Methods ===== ===== ===== */
/* ========================================================= */
void math::interp_spline::interp1(double& result,
										const math::vec1&,
										const math::vec1& values,
										const int& pos1,
										const math::ratio& rat1,
										const math::hermite1v&) const {
	double b = static_cast<const math::ratio_spline&>(rat1)._ratio12;
	double a = 1 - b;
	result = a * values[pos1] + b * values[pos1+1] +
		   ((a*a*a - a) * (*_diffdiff)[pos1] + (b*b*b - b) * (*_diffdiff)[pos1+1]) * ((*_dist)[pos1] * (*_dist)[pos1]) / 6.0;
}
void math::interp_spline::interp1(double& result,
										const math::vec& values,
										const unsigned short& pos1,
										const double& b) const {
	double a = 1 - b;
	result = a * values[pos1] + b * values[pos1+1] +
		   ((a*a*a - a) * (*_diffdiff)[pos1] + (b*b*b - b) * (*_diffdiff)[pos1+1]) * ((*_dist)[pos1] * (*_dist)[pos1]) / 6.0;
}
/* Fills up the input magnitude by interpolating in ONE dimension based on the
input magnitudes included in the 1D matrix values with the lowest position being
that identified by pos1, and the ratios contained in rat1 */

void math::interp_spline::interp2(double& result,
										const math::vec1&,
										const math::vec1&,
										const math::vec2& Values,
										const int& pos2,
										const int& pos1,
										const math::ratio& rat2,
										const math::ratio& rat1,
										const math::hermite2v&) const {
	math::vec1* temp = Values.sample_vector();
    double xx = 0.;
    // evaluate splines in one direction
    for (int i = 0; i != _spline->size(); ++i) {
        //(*_spline)[i]->interp1(*xx, Values(i), pos1, static_cast<const math::ratio_linear&>(rat1)._ratio12);
        (*_spline)[i]->interp1(xx, Values, Values.size1() * i + pos1, static_cast<const math::ratio_linear&>(rat1)._ratio12);
        temp->set(i, xx);
    }
    // construct spline with the results of the previous evaluation
    math::interp_spline Ospl(*_points2, *temp, 0);
    Ospl.set_diffs(*_points2);
    math::hermite1v Oherm_dummy;
    // evaluate resulting spline in other direction
    Ospl.interp1(result, *_points2, *temp, pos2, rat2, Oherm_dummy);
	delete temp;
}

void math::interp_spline::interp2(double& result,
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

void math::interp_spline::interp3(double& result,
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
    throw std::runtime_error("Function not implemented.");
}
void math::interp_spline::interp3(double& result,
										const math::vec1& points3,
										const std::vector<math::f_table2V*>& tables,
										const int& pos3,
										const math::ratio& rat3,
										const double& input2,
										const double& input1,
										const math::hermite3v&) const {
    throw std::runtime_error("Function not implemented.");
}

void math::interp_spline::interp3(double& result,
										const math::vec1& points3,
										const std::vector<math::f_tabular2V*>& tables,
										const int& pos3,
										const math::ratio& rat3,
										const double& input2,
										const double& input1,
										const math::hermite3v& her) const {
    throw std::runtime_error("Function not implemented.");
}

/* Fills up the input magnitude by interpolating in THREE dimensions based on the
input magnitudes included in the 3D matrix VValues with the lowest positions being
those identified by pos1, pos2, and pos3, and the ratios contained in rat1, rat2,
and rat3. The second version contains the input magnitudes, position, and ratio only
for the third dimension, plus a series of bidimensional tables. The third version 
contains the input magnitudes, position, and ratio only for the third dimension, 
plus a series of bidimensional tabular objects.  */

void math::interp_spline::interp4(double& result,
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
    throw std::runtime_error("Function not implemented.");
}
/* Fills up the input magnitude by interpolating in FOUR dimensions based on the
input magnitudes included in the 4D matrix VVValues with the lowest positions being
those identified by pos1, pos2, pos3, and pos4, and the ratios contained in rat1, rat2,
rat3 and rat4 */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////






#include "f_table2V.h"

// CLASS F_TABLE2V
// ===============
// ===============

const math::logic::PRED_NAME math::f_table2V::_name = math::logic::f_table2V;
/* predicate name */

const std::string math::f_table2V::_st_name = "f_table2V";
/* predicate name string */

math::f_table2V::f_table2V(vec1* points2,
						  vec1* points1,
						  vec2* Values,
						  math::logic::INTERP_MODE interp_mode)
: _points2(points2), _points1(points1), _Values(Values), _del_flag(true),
_interp_mode(interp_mode), _interp(math::interp::get_interp(interp_mode)),
_equi2(true), _equi1(true), _functor_diff(0),
_herm(0), _slopes_d2(0), _slopes_d1(0), _finder2(0), _finder1(0),
_checker2(new math::range_checker_inactive()),
_checker1(new math::range_checker_inactive()) {
	initialize();
}
/* constructor based on pointer to size n vector points2, pointer
to a size m vector points1, and pointer to a size n vector of size m
vectors Values - deleted by destructor. */

math::f_table2V::f_table2V(vec1& points2,
						  vec1& points1,
						  vec2& Values,
						  math::logic::INTERP_MODE interp_mode)
: _points2(&points2), _points1(&points1), _Values(&Values), _del_flag(false),
_interp_mode(interp_mode), _interp(math::interp::get_interp(interp_mode)),
_equi2(true), _equi1(true), _functor_diff(0),
_herm(0), _slopes_d2(0), _slopes_d1(0), _finder2(0), _finder1(0),
_checker2(new math::range_checker_inactive()),
_checker1(new math::range_checker_inactive()) {
	initialize();
}
/* constructor based on reference to size n vector points2, reference
to a size m vector points1, and reference to a size n vector of size m
vectors Values - not deleted by destructor. */

void math::f_table2V::initialize() {
	// check size compatibility between _points2, _points1, and _Values
	int size2 = _points2->size1();
	int size1 = _points1->size1();
	if ((size2 != _Values->size2()) || (size1 != _Values->size1())) {
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	// points vectors have the required number of points
	int n = _interp->get_min_points();
	if (size2 < n) {
		std::string st_name = "a";
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	if (size1 < n) {
		std::string st_name = "a";
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	// difference between first two points of _points2 and _points1
	_points2_diff = _points2->get(1) - _points2->get(0);
	if (_points2_diff <= 0.) { // they need to be in increasing order
		std::string st_name = "a";
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	_points1_diff = _points1->get(1) - _points1->get(0);
	if (_points1_diff <= 0.) { // they need to be in increasing order
		std::string st_name = "a";
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	// difference between consecutive _points2
	double temp = 0.;
	for (unsigned short i = 1; i != size2 - 1; ++i) {
		temp = _points2->get(i+1) - _points2->get(i);
		if (temp <= 0.) {  // they need to be in increasing order
			std::string st_name = "a";
			destroy();
            throw std::runtime_error("Incorrect size.");
		}
		if (_equi2 == true) { // so far equispaced
			if (fabs((temp-_points2_diff)/temp) > constant::DIFF()) {
				// not equiespaced
				_equi2 = false;
				_points2_diff = 0.;
			}
		}
		// else: so far not equispaced and will remain that way
	}
	// difference between consecutive _points1
	for (unsigned short i = 1; i != size1 - 1; ++i) {
		temp = _points1->get(i+1) - _points1->get(i);
		if (temp <= 0.) { // they need to be in increasing order
			std::string st_name = "a";
			destroy();
            throw std::runtime_error("Incorrect size.");
		}
		if (_equi1 == true) { // so far equispaced
			if (fabs((temp-_points1_diff)/temp) > constant::DIFF()) {
				// not equiespaced
				_equi1 = false;
				_points1_diff = 0.;
			}
		}
		// else: so far not equispaced and will remain that way
	}
	if (_equi2 == true) {_finder2 = new math::pos_finder_equispaced(*_points2, *_interp, _points2_diff);}
	else                {_finder2 = new math::pos_finder_binary(*_points2, *_interp);}
	if (_equi1 == true) {_finder1 = new math::pos_finder_equispaced(*_points1, *_interp, _points1_diff);}
	else                {_finder1 = new math::pos_finder_binary(*_points1, *_interp);}

	// fill up interpolation method (dummy except for splines)
	_interp->complete_spline(*_points2, *_points1, *_Values);

	switch (_interp_mode) {
		case math::logic::lagrange_first_precompute:
			fill_up_slopes_lagrange_first_precompute();
			_herm = new hermite2v();
			_functor_diff = new math::table2V_diff_prec(*this);
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			fill_up_slopes_hermite();
			_herm = new hermite2v(*_points2, *_points1, *_Values, 0, *_slopes_d2, 0, *_slopes_d1, 0, _interp_mode);
			_functor_diff = new math::table2V_diff_real(*this);
			break;
		default:
			_slopes_d2 = 0;
			_slopes_d1 = 0;
			_herm = new hermite2v();
			_functor_diff = new math::table2V_diff_real(*this);
			break;
	}
}
/**< initialization for constructors */

math::f_table2V::f_table2V(const f_table2V& other)
: _del_flag(other._del_flag),
_points2_diff(other._points2_diff), _equi2(other._equi2),
_points1_diff(other._points1_diff), _equi1(other._equi1),
_interp_mode(other._interp_mode),
_herm(0), _slopes_d2(0), _slopes_d1(0),
_interp(math::interp::get_interp(_interp_mode, *other._points2, *other._points1, *other._Values)),
_checker2(other._checker2->clone()), _checker1(other._checker1->clone()),
_finder2(other._finder2->clone()), _finder1(other._finder1->clone()) {

	if (other._herm != 0) _herm = new hermite2v(*other._herm);

	if (other._slopes_d2 != 0) _slopes_d2 = other._slopes_d2->clone();
	if (other._slopes_d1 != 0) _slopes_d1 = other._slopes_d1->clone();

	if (_del_flag == true) {
		_points2	= other._points2->clone();
		_points1	= other._points1->clone();
		_Values		= other._Values->clone();
	}
	else {
		_points2	= other._points2;
		_points1	= other._points1;
		_Values		= other._Values;
	}

	switch (_interp_mode) {
		case math::logic::lagrange_first_precompute:
			_functor_diff = new math::table2V_diff_prec(*this);
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			_functor_diff = new math::table2V_diff_real(*this);
			break;
		default:
			_functor_diff = new math::table2V_diff_real(*this);
			break;
	}
}
/* copy constructor */

void math::f_table2V::destroy() {
	if (_del_flag == true) {
		delete _points2;
		delete _points1;
		delete _Values;
	}
	delete _interp;
	delete _functor_diff;
	delete _slopes_d2;
	delete _slopes_d1;
	delete _herm;
	delete _checker2;
	delete _checker1;
	delete _finder2;
	delete _finder1;
}
/* destructor */

math::f_table2V* math::f_table2V::clone() const {
	return new f_table2V(*this);
}
/* cloner */

bool math::f_table2V::operator==(const pred2v& op2) const {
	return (op2.get_name() == math::logic::f_table2V) ?
		(*this == static_cast<const f_table2V&>(op2)) : false;
}
bool math::f_table2V::operator==(const f_table2V& op2) const {
	return ((*_points2 == *op2._points2) &&
		    (*_points1 == *op2._points1) &&
			(*_Values == *op2._Values) &&
			(_interp_mode == op2._interp_mode));
}
/* overloaded operator == (equal) */

int math::f_table2V::compute_pos2(const double& input2) const {
	return _interp->find_index(_finder2->search(*_points2, input2, _points2_diff),
		_points2->size1());
}
/* Returns first position within _points2 vector that shall be employed when
interpolating to obtain the result corresponding to the input magnitude. */

int math::f_table2V::compute_pos1(const double& input1) const {
	return _interp->find_index(_finder1->search(*_points1, input1, _points1_diff),
		_points1->size1());
}
/* Returns first position within _points1 vector that shall be employed when
interpolating to obtain the result corresponding to the input magnitude. */

math::ratio* math::f_table2V::compute_ratio2(const double& input2,
												  const int& pos2) const {
	return _interp->compute_ratio(input2, *_points2, pos2);
}
/* Returns ratio of input2 with respect to the two points2 identified by pos2 and pos2+1. */

math::ratio* math::f_table2V::compute_ratio1(const double& input1,
												  const int& pos1) const {
	return _interp->compute_ratio(input1, *_points1, pos1);
}
/* Returns ratio of input1 with respect to the two points1 identified by pos1 and pos1+1. */

double math::f_table2V::compute_value(const int& pos2,
                                const int& pos1,
                                const math::ratio& ratio2,
                                const math::ratio& ratio1) const {
	double res;
   _interp->interp2(res, *_points2, *_points1, *_Values, pos2, pos1, ratio2, ratio1, *_herm);
    return res;
}
/* Fills up result magnitude by interpolating based on the input positions
and ratios */

double math::f_table2V::compute_diff(const int& pos2,
								  const int& pos1,
								  const double& input2,
								  const double& input1,
								  const double& input2_dt,
								  const double& input1_dt) const {
	double res;
	_functor_diff->compute_diff(res, pos2, pos1, input2, input1, input2_dt, input1_dt);
    return res;
}
/* Fills up result differential based on positions and ratios */

double math::f_table2V::value(const double& input2,
                             const double& input1) const {
	// verify inputs are within range
	_checker2->check_range(*_points2, input2);
	_checker1->check_range(*_points1, input1);

	// pos2 provides the upper vector position
	// pos1 provides the upper position within each vector
	int pos2 = compute_pos2(input2);
	int pos1 = compute_pos1(input1);
	math::ratio* ratio2 = compute_ratio2(input2, pos2);
	math::ratio* ratio1 = compute_ratio1(input1, pos1);
    double res;
	_interp->interp2(res, *_points2, *_points1, *_Values, pos2, pos1, *ratio2, *ratio1, *_herm);
	_interp->to_pool(ratio2);
	_interp->to_pool(ratio1);
    return res;
}
/* evaluates the function at the reference magnitudes input2 and input1,
and	writes the result at the reference magnitude result. Only the magnitude
value is inserted into result, the units are assummed to be OK. */

double math::f_table2V::d_dt(const double& input2,
                       const double& input1,
                       const double& input2_dt,
                       const double& input1_dt) const {
	////////////////////////////////////////////////////////////////////
	// The computation of the two positions, which are quite expensive,
	// in theory have already been done before in the value method, and
	// should not be repeated here. However, this can not be avoided.
	////////////////////////////////////////////////////////////////////
	// verify inputs are within range
	_checker2->check_range(*_points2, input2);
	_checker1->check_range(*_points1, input1);
	// pos2 provides the upper vector position
	// pos1 provides the upper position within each vector
	int pos2 = compute_pos2(input2);
	int pos1 = compute_pos1(input1);
    double res;
	_functor_diff->compute_diff(res, pos2, pos1, input2, input1, input2_dt, input1_dt);
    return res;
}
/* evaluates the function differential with time at the reference
magnitudes input2 and input1 and their differentials with time input2_dt
and input1_dt, and writes the result at the reference magnitude result. */

double math::f_table2V::d_d2(const double& input2,
							const double& input1) const {
	////////////////////////////////////////////////////////////////////
	// The computation of the two positions, which are quite expensive,
	// in theory have already been done before in the value method, and
	// should not be repeated here. However, this can not be avoided.
	////////////////////////////////////////////////////////////////////

	// pos2 provides the lower vector position
	// pos1 provides the lower position within each vector
	int pos2 = compute_pos2(input2);
	int pos1 = compute_pos1(input1);
	math::ratio_linear* ratio1 = static_cast<math::ratio_linear*>(compute_ratio1(input1, pos1));
	double temp;
    math::interp::basic_interp_lagrange_first(temp,
											_slopes_d2->get(pos2, pos1),
											_slopes_d2->get(pos2, pos1+1),
											ratio1->_ratio12);
	_interp->to_pool(ratio1);
	return temp;
}
/* computes partial variation of Values with respect to 1st independent
magnitude, based on the values of the two independent magnitudes. Crashes
if _interp_mode != math::logic::lagrange_first_precompute. */

double math::f_table2V::d_d1(const double& input2,
							const double& input1) const {
	////////////////////////////////////////////////////////////////////
	// The computation of the two positions, which are quite expensive,
	// in theory have already been done before in the value method, and
	// should not be repeated here. However, this can not be avoided.
	////////////////////////////////////////////////////////////////////

	// pos2 provides the lower vector position
	// pos1 provides the lower position within each vector
	int pos2 = compute_pos2(input2);
	int pos1 = compute_pos1(input1);
	math::ratio_linear* ratio2 = static_cast<math::ratio_linear*>(compute_ratio2(input2, pos2));
	double temp = 0.;
	math::interp::basic_interp_lagrange_first(temp,
											_slopes_d1->get(pos1, pos2),
											_slopes_d1->get(pos1, pos2+1),
											ratio2->_ratio12);
	_interp->to_pool(ratio2);
	return temp;
}
/* computes partial variation of Values with respect to 2nd independent
magnitude, based on the values of the two independent magnitudes. Crashes
if _interp_mode != math::logic::lagrange_first_precompute. */

void math::f_table2V::fill_up_slopes_lagrange_first_precompute() {
	// differentials with respect to first independent magnitude
	// has size (m) x (n)
	_slopes_d2 = new vec2(_points2->size1(), _points1->size1());
	for (unsigned short j = 0; j != _slopes_d2->size2(); ++j) {
		fill_up_slopes_aux2_lagrange_first_precompute(*_slopes_d2, j, _points2->size1());
	}
	// differentials with respect to second independent magnitude
	// has size (n) x (m)
	_slopes_d1 = new vec2(_points1->size1(), _points2->size1());
	for (unsigned short j = 0; j != _slopes_d1->size2(); ++j) {
		fill_up_slopes_aux1_lagrange_first_precompute(*_slopes_d1, j, _points1->size1());
	}
}
/* fills up the _slopes_d2 and _slopes_d1 attributes based on _points2,
_points1, and _Values */

void math::f_table2V::fill_up_slopes_aux2_lagrange_first_precompute
				(math::vec2& slopes_d2,
				 unsigned short pos,
				 unsigned short siz) {
	for (unsigned short i = 0; i != siz - 1; ++i) {
		slopes_d2.set(i, pos, (_Values->get(pos, i+1) - _Values->get(pos, i)) /
			_finder2->diff(i+1,i));
	}
	slopes_d2.set(siz - 1, pos, 0.);
}
/* fills up row "pos" of the _slopes_d2 matrix of differentials with respect to the
first independent magnitude with a vector of size "siz" */

void math::f_table2V::fill_up_slopes_aux1_lagrange_first_precompute
				(math::vec2& slopes_d1,
				 unsigned short pos,
				 unsigned short siz) {
	for (unsigned short i = 0; i != siz - 1; ++i) {
		slopes_d1.set(i, pos, (_Values->get(i+1, pos) - _Values->get(i,pos)) /
				_finder1->diff(i+1, i));
	}
	slopes_d1.set(siz-1, pos, 0.);
}
/* fills up row "pos" of the _slopes_d1 matrix of differentials with respect to the
second independent magnitude with a vector of size "siz" */

void math::f_table2V::fill_up_slopes_hermite() {
	// differentials with respect to first independent magnitude
	// has size (m) x (n)
	_slopes_d2 = new vec2(_points2->size1(), _points1->size1());
	for (unsigned short j = 0; j != _slopes_d2->size2(); ++j) {
		fill_up_slopes_aux2_hermite(*_slopes_d2, j, _points2->size1());
	}
	// differentials with respect to second independent magnitude
	// has size (n) x (m)
	_slopes_d1 = new vec2(_points1->size1(), _points2->size1());
	for (unsigned short j = 0; j != _slopes_d1->size2(); ++j) {
		fill_up_slopes_aux1_hermite(*_slopes_d1, j, _points1->size1());
	}
}
/* fills up the _slopes_d2 and _slopes_d1 attributes based on _points2,
_points1, and _Values */

void math::f_table2V::fill_up_slopes_aux2_hermite(math::vec2& slopes_d2,
												unsigned short pos,
												unsigned short n) {
	switch (_interp_mode) {
		case math::logic::hermite_first: {
			vec1 temp(n - 1);
			// compute temporary slopes as in Lagrange first order for each interval
			for (unsigned short i = 0; i < n - 1; ++i) {
				temp[i] = (_Values->get(pos, i+1) - _Values->get(pos, i)) / _finder2->diff(i+1, i);
			}
			// compute slopes by average of previous slopes
			slopes_d2.set(0, pos, temp[0]);
			for (unsigned short i = 1; i < n - 1; ++i) {
				slopes_d2.set(i, pos, 0.5 * (temp[i-1] + temp[i]));
			}
			slopes_d2.set(n-1, pos, temp.back());
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			slopes_d2.set(0, pos,
				 _Values->get(pos, 1) / _finder2->diff(2, 0, 1, 0, 2, 1) -
				 _Values->get(pos, 2) / _finder2->diff(1, 0, 2, 0, 2, 1) -
				 _Values->get(pos, 0) / _finder2->diff(2, 0) -
				 _Values->get(pos, 0) / _finder2->diff(1, 0));
			for (unsigned short i = 1; i < n-1; ++i) {
				slopes_d2.set(i, pos,
				_Values->get(pos, i+1) / _finder2->diff(i,   i-1, i+1, i-1, i+1, i) -
				_Values->get(pos, i-1) / _finder2->diff(i+1, i,   i+1, i-1, i,   i-1) +
				_Values->get(pos, i)   / _finder2->diff(i,   i-1) -
				_Values->get(pos, i)   / _finder2->diff(i+1, i));
			}
			slopes_d2.set(n-1, pos,
				_Values->get(pos, n-3) / _finder2->diff(n-1, n-2, n-2, n-3, n-1, n-3) -
				_Values->get(pos, n-2) / _finder2->diff(n-1, n-3, n-2, n-3, n-1, n-2) +
				_Values->get(pos, n-1) / _finder2->diff(n-1, n-3) +
				_Values->get(pos, n-1) / _finder2->diff(n-1, n-2));
			break; }
		default:
            throw std::runtime_error("Incorrect interpolation method.");
			break;
	}
}
/* fills up row "pos" of the _slopes_d2 matrix of differentials with respect to the
first independent magnitude with a vector of size "n" */

void math::f_table2V::fill_up_slopes_aux1_hermite(math::vec2& slopes_d1,
												unsigned short pos,
												unsigned short n) {
	switch (_interp_mode) {
		case math::logic::hermite_first: {
			vec1 temp(n - 1);
			// compute temporary slopes as in Lagrange first order for each interval
			for (unsigned short i = 0; i < n - 1; ++i) {
				temp[i] = (_Values->get(i+1, pos) - _Values->get(i, pos)) / _finder1->diff(i+1, i);
			}
			// compute slopes by average of previous slopes
			slopes_d1.set(0, pos, temp[0]);
			for (unsigned short i = 1; i < n - 1; ++i) {
				slopes_d1.set(i, pos, 0.5 * (temp[i-1] + temp[i]));
			}
			slopes_d1.set(n-1, pos, temp.back());
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			slopes_d1.set(0, pos,
				 _Values->get(1, pos) / _finder1->diff(2, 0, 1, 0, 2, 1) -
				 _Values->get(2, pos) / _finder1->diff(1, 0, 2, 0, 2, 1) -
				 _Values->get(0, pos) / _finder1->diff(2, 0) -
				 _Values->get(0, pos) / _finder1->diff(1, 0));
			for (unsigned short i = 1; i < n-1; ++i) {
				slopes_d1.set(i, pos,
					_Values->get(i+1, pos) / _finder1->diff(i,   i-1, i+1, i-1, i+1, i) -
					_Values->get(i-1, pos) / _finder1->diff(i+1, i,   i,   i-1, i+1, i-1) +
					_Values->get(i, pos)   / _finder1->diff(i,   i-1) -
					_Values->get(i, pos)   / _finder1->diff(i+1, i));
			}
			slopes_d1.set(n-1, pos,
				_Values->get(n-3, pos) / _finder1->diff(n-1, n-2, n-2, n-3, n-1, n-3) -
				_Values->get(n-2, pos) / _finder1->diff(n-1, n-3, n-2, n-3, n-1, n-2) +
				_Values->get(n-1, pos) / _finder1->diff(n-1, n-3) +
				_Values->get(n-1, pos) / _finder1->diff(n-1, n-2));
			break; }
		default:
            throw std::runtime_error("Incorrect interpolation method.");
			break;
	}
}
/* fills up row "pos" of the _slopes_d1 matrix of differentials with respect to the
second independent magnitude with a vector of size "n" */

void math::f_table2V::activate_checker2() {
	delete _checker2;
	_checker2 = new math::range_checker_active();
}
void math::f_table2V::deactivate_checker2() {
	delete _checker2;
	_checker2 = new math::range_checker_inactive();
}
/* Activates or deactives the out of range verification for _points2, which
is inactive by default */

void math::f_table2V::activate_checker1() {
	delete _checker1;
	_checker1 = new math::range_checker_active();
}
void math::f_table2V::deactivate_checker1() {
	delete _checker1;
	_checker1 = new math::range_checker_inactive();
}
/* Activates or deactives the out of range verification for _points1, which
is inactive by default */


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE2V_DIFF_PREC
// =======================
// =======================

math::table2V_diff_prec::table2V_diff_prec(math::f_table2V& pred)
: _pred(&pred) {}
/* constructor based on two dimensional table */

math::table2V_diff_prec::~table2V_diff_prec() {
}
/* destructor */

void math::table2V_diff_prec::compute_diff(double& result,
											  const int& pos2,
											  const int& pos1,
											  const double& input2,
											  const double& input1,
											  const double& input2_dt,
											  const double& input1_dt) const {
	math::ratio_linear* ratio2 = static_cast<math::ratio_linear*>(_pred->compute_ratio2(input2, pos2));
	math::ratio_linear* ratio1 = static_cast<math::ratio_linear*>(_pred->compute_ratio1(input1, pos1));
    // Differential with respect to 2nd independent magnitude
    double temp = 0.;
    math::interp::basic_interp_lagrange_first(temp,
                                        _pred->_slopes_d1->get(pos1, pos2),
                                        _pred->_slopes_d1->get(pos1, pos2+1),
                                        ratio2->_ratio12);
    result = temp * input1_dt;
    // Differential with respect to 1st independent magnitude
    math::interp::basic_interp_lagrange_first(temp,
                                        _pred->_slopes_d2->get(pos2, pos1),
                                        _pred->_slopes_d2->get(pos2, pos1+1),
                                        ratio1->_ratio12);
    result = result + (temp * input2_dt);
	math::ratio_mgr::to_pool_linear(ratio2);
	math::ratio_mgr::to_pool_linear(ratio1);
}
/* fill up result differential based on positions, inputs, and its partial
differentials with time */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE2V_DIFF_REAL
// =======================
// =======================

math::table2V_diff_real::table2V_diff_real(math::f_table2V& pred)
: _pred(&pred) {}
/* constructor based on two dimensional table */

math::table2V_diff_real::~table2V_diff_real() {
}
/* destructor */

void math::table2V_diff_real::compute_diff(double& result,
											  const int& pos2,
											  const int& pos1,
											  const double& input2,
											  const double& input1,
											  const double& input2_dt,
											  const double& input1_dt) const {
	math::ratio* ratio2 = _pred->compute_ratio2(input2, pos2);
	math::ratio* ratio1 = _pred->compute_ratio1(input1, pos1);
	math::ratio* ratioX2 = _pred->_interp->copy_ratio(ratio2);
	math::ratio* ratioX1 = _pred->_interp->copy_ratio(ratio1);
	double Pmag1, PmagX;

    // compute result at point
    _pred->_interp->interp2(Pmag1, *_pred->_points2, *_pred->_points1, *_pred->_Values, pos2, pos1, *ratio2, *ratio1, *_pred->_herm);
    // compute result a short interval after point in direction 1
    double difX = _pred->_finder2->compute_Dratio(*ratioX2, input2, pos2);
    _pred->_interp->interp2(PmagX, *_pred->_points2, *_pred->_points1, *_pred->_Values, pos2, pos1, *ratioX2, *ratio1, *_pred->_herm);
    result = (PmagX - Pmag1) * _pred->_finder2->compute_final_diff(input2_dt, difX);
    // compute result a short interval after point in direction 2
    difX = _pred->_finder1->compute_Dratio(*ratioX1, input1, pos1);
    _pred->_interp->interp2(PmagX, *_pred->_points2, *_pred->_points1, *_pred->_Values, pos2, pos1, *ratio2, *ratioX1, *_pred->_herm);
    result = result + (PmagX - Pmag1) * _pred->_finder1->compute_final_diff(input1_dt, difX);
	// release memory
	_pred->_interp->to_pool(ratio2);
	_pred->_interp->to_pool(ratio1);
	_pred->_interp->to_pool(ratioX1);
	_pred->_interp->to_pool(ratioX2);
}
/* fill up result differential based on positions, inputs, and its partial
differentials with time */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




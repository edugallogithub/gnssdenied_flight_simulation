#include "f_table1V.h"

// CLASS F_MATH1V
// =================
// =================

const math::logic::PRED_NAME math::f_table1V::_name = math::logic::f_table1V;
/* predicate name */

const std::string math::f_table1V::_st_name = "f_table1V";
/* predicate name string */

math::f_table1V::f_table1V(vec1* points1,
                     vec1* values,
                     math::logic::INTERP_MODE interp_mode)
: _points1(points1), _values(values), _del_flag(true), 
_interp_mode(interp_mode), _interp(math::interp::get_interp(interp_mode)),
_equi1(true), _functor_diff(0), _herm(0), _slopes(0), _finder1(0),
_checker1(new math::range_checker_inactive()) {
	initialize();
}
/* constructor based on pointers - deleted by destructor. */

math::f_table1V::f_table1V(vec1& points1,
                     vec1& values,
                     math::logic::INTERP_MODE interp_mode)
: _points1(&points1), _values(&values), _del_flag(false),
_interp_mode(interp_mode), _interp(math::interp::get_interp(interp_mode)),
_equi1(true), _functor_diff(0), _herm(0), _slopes(0), _finder1(0),
_checker1(new math::range_checker_inactive()) {
	initialize();
}
/* constructor based on references - not deleted by destructor. */

void math::f_table1V::initialize() {
	//check size compatibility between _points and _values
	int size1 = _points1->size1();
	if (size1 != _values->size1()) {
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	// points vector has the required number of points
	int n = _interp->get_min_points();
	if (size1 < n) {
		std::string st_name = "a";
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	// difference between first two points of _points1
	_points1_diff = _points1->get(1) - _points1->get(0);
	if (_points1_diff <= 0.) { // they need to be in increasing order
		std::string st_name = "a";
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	// difference between consecutive _points1
	double temp = 0.;
	for (unsigned short i = 1; i != size1 - 1; ++i) {
		temp = _points1->get(i+1) -  _points1->get(i);
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
	if (_equi1 == true) {_finder1 = new math::pos_finder_equispaced(*_points1, *_interp, _points1_diff);}
	else                {_finder1 = new math::pos_finder_binary(*_points1, *_interp);}

	// fill up interpolation method (dummy except for splines)
	_interp->complete_spline(*_points1, *_values, 0);
	// complete interpolation method pointer (dummy except for splines)
	_interp->set_diffs(*_points1);

	switch (_interp_mode) {
		case math::logic::lagrange_first_precompute:
			fill_up_slopes_lagrange_first_precompute();
			_herm = new hermite1v();
			_functor_diff = new math::table1V_diff_prec(*this);
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			fill_up_slopes_hermite();
			_herm = new hermite1v(*_points1, *_values, 0, *_slopes, 0, _interp_mode);
			_functor_diff = new math::table1V_diff_real(*this);
			break;
		default:
			_slopes = 0;
			_herm = new hermite1v();
			_functor_diff = new math::table1V_diff_real(*this);
			break;
	}
}
/**< initialization for constructors */

math::f_table1V::f_table1V(const f_table1V& other)
: _del_flag(other._del_flag), _points1_diff(other._points1_diff), _equi1(other._equi1),
_interp_mode(other._interp_mode), _herm(0), _slopes(0),
_interp(math::interp::get_interp(_interp_mode, *other._points1, *other._values)),
_checker1(other._checker1->clone()), _finder1(other._finder1->clone()) {
	
	if (other._herm != 0) _herm = new hermite1v(*other._herm);
	
	if (other._slopes != 0) _slopes = other._slopes->clone();
	
	if (_del_flag == true) {
		_points1 = other._points1->clone();
		_values = other._values->clone();
	}
	else {
		_points1 = other._points1;
		_values = other._values;
	}
	
	switch (_interp_mode) {
		case math::logic::lagrange_first_precompute:
			_functor_diff = new math::table1V_diff_prec(*this);
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			_functor_diff = new math::table1V_diff_real(*this);
			break; 
		default: 
			_functor_diff = new math::table1V_diff_real(*this);
			break; 
	}
	// complete _interp pointer (dummy except for splines)
	_interp->set_diffs(*_points1);
}
/* copy constructor */

void math::f_table1V::destroy() {
	if (_del_flag == true) {
		delete _points1;
		delete _values;
	}
	delete _interp;
	delete _functor_diff;
	delete _herm;
	delete _slopes;
	delete _checker1;
	delete _finder1;
}
/* destructor */

math::f_table1V* math::f_table1V::clone() const {
	return new f_table1V(*this);
}
/* cloner */

bool math::f_table1V::operator==(const pred1v& op2) const {
	return (op2.get_name() == math::logic::f_table1V) ? (*this == static_cast<const f_table1V&>(op2)) : false;
}
bool math::f_table1V::operator==(const f_table1V& op2) const {
	return ((*_points1 == *op2._points1) && (*_values == *op2._values) && (_interp_mode == op2._interp_mode));
}
/* overloaded operator == (equal) */

int math::f_table1V::compute_pos1(const double& input1) const {
	return _interp->find_index(_finder1->search(*_points1, input1, _points1_diff), _points1->size1());
}
/* Returns first position within _points1 vector that shall be employed when
interpolating to obtain the result corresponding to the input magnitude. */

math::ratio* math::f_table1V::compute_ratio1(const double& input1, const int& pos1) const {
	return _interp->compute_ratio(input1, *_points1, pos1);
}
/* Returns ratio of input1 with respect to the two points identified by pos1 and pos1+1.*/

double math::f_table1V::compute_value(const int& pos1, const math::ratio& ratio1) const {
	double res;
	_interp->interp1(res, *_points1, *_values, pos1, ratio1, *_herm);
    return res;
}
/* Fills up result magnitude by interpolating based on the input position and 
ratio */

double math::f_table1V::compute_diff(const int& pos1, const double& input1, const double& input1_dt) const {
	double res;
	_functor_diff->compute_diff(res, pos1, input1, input1_dt);
    return res;
}
/* Fills up result differential basedon position and ratio */

double math::f_table1V::value(const double& input1) const {
	// verify inputs are within range
	_checker1->check_range(*_points1, input1);
	int pos1 = compute_pos1(input1);
	math::ratio* ratio1 = compute_ratio1(input1, pos1);
    double res;
	_interp->interp1(res, *_points1, *_values, pos1, *ratio1, *_herm);
	_interp->to_pool(ratio1);
    return res;
}
/* evaluates the function at the reference magnitude input, and writes the
result at the reference magnitude result. Only the magnitude value is
inserted into result, the units are assummed to be OK. */

double math::f_table1V::d_dt(const double& input1, const double& input1_dt) const {
	////////////////////////////////////////////////////////////////////
	// The computation of the position, which is quite expensive, in theory
	// has already been done before in the value method, and should not be
	// repeated here. However, this can not be avoided.
	////////////////////////////////////////////////////////////////////
	// verify inputs are within range
    _checker1->check_range(*_points1, input1);
	int pos1 = compute_pos1(input1);
    double res;
    _functor_diff->compute_diff(res, pos1, input1, input1_dt);
    return res;
}
/* evaluates the function differential with time at the reference
magnitude input and its differential with time input_dt, and writes the
result at the reference magnitude result. */

void math::f_table1V::fill_up_slopes_lagrange_first_precompute() {
	_slopes = new math::vec1(_points1->size1());
	for (unsigned short i = 0; i != _slopes->size1() - 1; ++i) {
		_slopes->set(i, _values->diff(i+1, i) / _finder1->diff(i+1,i)); 
	}
	_slopes->back() = 0.; // last value is meaningless
	// size is n for compatibility with other interpolations
}
void math::f_table1V::fill_up_slopes_hermite() {
	switch (_interp_mode) {
		case math::logic::hermite_first: {
			// compute temporary slopes as in Lagrange first order for each interval 
			std::vector<double> temp(_points1->size1() - 1); 
			for (unsigned short i = 0; i != temp.size() - 1; ++i) {
				temp[i] = _values->diff(i+1, i) / _finder1->diff(i+1,i);
			}
			// compute slopes by average of previous slopes
			_slopes = new math::vec1(_points1->size1());
			_slopes->front() = temp[0];
			for (unsigned short i = 1; i < _slopes->size1() - 1; ++i) {
				_slopes->set(i, 0.5 * (temp[i-1] + temp[i]));
			}
 			_slopes->back() = temp.back();
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			unsigned short n = _points1->size1();
			_slopes = new math::vec1(n);
			_slopes->front() =
				 _values->get(1) / _finder1->diff(2, 0, 1, 0, 2, 1) -
			     _values->get(2) / _finder1->diff(1, 0, 2, 0, 2, 1) -
			     _values->get(0) / _finder1->diff(2, 0) - 
				 _values->get(0) / _finder1->diff(1, 0);
			for (unsigned short i = 1; i < n-1; ++i) {
				_slopes->set(i, 
				_values->get(i+1) / _finder1->diff(i,   i-1, i+1, i-1, i+1, i) - 
			    _values->get(i-1) / _finder1->diff(i+1, i,   i,   i-1, i+1, i-1) + 			   
				_values->get(i)   / _finder1->diff(i,   i-1) - 
			    _values->get(i)   / _finder1->diff(i+1, i));
			}
			_slopes->back() =
			    _values->get(n-3) / _finder1->diff(n-1, n-2, n-2, n-3, n-1, n-3) -
				_values->get(n-2) / _finder1->diff(n-1, n-3, n-2, n-3, n-1, n-2) +
			    _values->get(n-1) / _finder1->diff(n-1, n-3) +
			    _values->get(n-1) / _finder1->diff(n-1, n-2);
			break; }
		default:
			throw std::runtime_error("Incorrect interpolation method.");
	}
}
/* fills up the _slopes attribute based on _points and _values */

void math::f_table1V::activate_checker1() {
	delete _checker1;
	_checker1 = new math::range_checker_active();
}
void math::f_table1V::deactivate_checker1() {
	delete _checker1;
	_checker1 = new math::range_checker_inactive();
}
/* Activates or deactives the out of range verification for _points1, which
is inactive by default */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS MATH1V_DIFF_PREC
// =======================
// =======================
	
math::table1V_diff_prec::table1V_diff_prec(math::f_table1V& pred)
: _pred(&pred) {}
/* constructor based on one dimensional table */

math::table1V_diff_prec::~table1V_diff_prec() {
}
/* destructor */

void math::table1V_diff_prec::compute_diff(double& result,
											  const int& pos1,
											  const double& input1,
											  const double& input1_dt) const {
	result = _pred->_slopes->get(pos1) * input1_dt;
}
/* fill up result differential based on position, input, and its partial
differential with time */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE1V_DIFF_REAL
// =======================
// =======================

math::table1V_diff_real::table1V_diff_real(math::f_table1V& pred)
: _pred(&pred) {}
/* constructor based on one dimensional table */

math::table1V_diff_real::~table1V_diff_real() {
}
/* destructor */

void math::table1V_diff_real::compute_diff(double& result,
											  const int& pos1,
											  const double& input1,
											  const double& input1_dt) const {
	math::ratio* ratio1 = _pred->compute_ratio1(input1, pos1);
	math::ratio* ratioX1 = _pred->_interp->copy_ratio(ratio1);
	double Pmag1, PmagX;
    // compute result at point
    _pred->_interp->interp1(Pmag1, *_pred->_points1, *_pred->_values, pos1, *ratio1, *_pred->_herm);
    // compute result a short interval after point in direction 1
    double difX = _pred->_finder1->compute_Dratio(*ratioX1, input1, pos1);
    _pred->_interp->interp1(PmagX, *_pred->_points1, *_pred->_values, pos1, *ratioX1, *_pred->_herm);
    result = (PmagX - Pmag1) * _pred->_finder1->compute_final_diff(input1_dt, difX);
	// release memory
	_pred->_interp->to_pool(ratio1);
	_pred->_interp->to_pool(ratioX1);
}
/* fill up result differential based on position, input, and its partial
differential with time */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




#include "f_table3V.h"

// CLASS F_TABLE3V
// ===============
// ===============

const math::logic::PRED_NAME math::f_table3V::_name = math::logic::f_table3V;
/* predicate name */

const std::string math::f_table3V::_st_name = "f_table3V";
/* predicate name string */

math::f_table3V::f_table3V(vec1* points3,
						  vec1* points2,
						  vec1* points1,
						  vec3* VValues,
						  math::logic::INTERP_MODE interp_mode)
: _points3(points3), _points2(points2), _points1(points1), _VValues(VValues),
_del_flag(true), _interp_mode(interp_mode),
_interp(math::interp::get_interp(interp_mode)),
_equi3(true), _equi2(true), _equi1(true), _functor_diff(0),
_slopes_d3(0), _slopes_d2(0), _slopes_d1(0),  _herm(0),
_finder3(0), _finder2(0), _finder1(0),
_checker3(new math::range_checker_inactive()),
_checker2(new math::range_checker_active()),
_checker1(new math::range_checker_active()) {
	initialize();
}
/* constructor based on pointer to size l vector points3, pointer	to a
size m vector points2, pointer to a size n vector points1, and pointer to
a size l vector of size m vectors of size n vectors VValues - deleted by
destructor. */

math::f_table3V::f_table3V(vec1& points3,
						  vec1& points2,
						  vec1& points1,
						  vec3& VValues,
						  math::logic::INTERP_MODE interp_mode)
: _points3(&points3), _points2(&points2), _points1(&points1), _VValues(&VValues),
_del_flag(false), _interp_mode(interp_mode),
_interp(math::interp::get_interp(interp_mode)),
_equi3(true), _equi2(true), _equi1(true), _functor_diff(0),
_slopes_d3(0), _slopes_d2(0), _slopes_d1(0),  _herm(0),
_finder3(0), _finder2(0), _finder1(0),
_checker3(new math::range_checker_inactive()),
_checker2(new math::range_checker_active()),
_checker1(new math::range_checker_active()) {
	initialize();
}
/* constructor based on reference to size l vector points3, reference	to a
size m vector points2, reference to a size n vector points1, and reference to
a size l vector of size m vectors of size n vectors VValues - not deleted
by destructor. If the interpolation mode is "linear_precompute", the
differentials are precomputed but the constructor is slower. */

void math::f_table3V::initialize() {
	// check size compatibility between _points3, _points2, _points1 and _VValues
	int size3 = _points3->size1();
	int size2 = _points2->size1();
	int size1 = _points1->size1();
	if ((size3!=_VValues->size3()) || (size2!=_VValues->size2()) || (size1!=_VValues->size1())) {
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
	// points vectors have at least minimum number of points
	int n = _interp->get_min_points();
	if (size3 < n) {
        std::string st_name = "a";
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
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
	// difference between first two points of _points3,_points2 and _points1
	_points3_diff = _points3->get(1) - _points3->get(0);
	if (_points3_diff <= 0.) { // they need to be in increasing order
		std::string st_name = "a";
		destroy();
        throw std::runtime_error("Incorrect size.");
	}
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
	// difference between consecutive _points3
	double temp = 0.;
	for (unsigned short i = 1; i != size3 - 1; ++i) {
		temp = _points3->get(i+1) - _points3->get(i);
		if (temp <= 0.) { // they need to be in increasing order
			std::string st_name = "a";
			destroy();
            throw std::runtime_error("Incorrect size.");
		}
		if (_equi3 == true) { // so far equispaced
			if (fabs((temp-_points3_diff)/temp) > constant::DIFF()) {
				// not equiespaced
				_equi3 = false;
				_points3_diff = 0.;
			}
		}
		// else: so far not equispaced and will remain that way
	}
	// difference between consecutive _points2
	for (unsigned short i = 1; i != size2 - 1; ++i) {
		temp = _points2->get(i+1) - _points2->get(i);
		if (temp <= 0.) { // they need to be in increasing order
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
	if (_equi3 == true) {_finder3 = new math::pos_finder_equispaced(*_points3, *_interp, _points3_diff);}
	else                {_finder3 = new math::pos_finder_binary(*_points3, *_interp);}
	if (_equi2 == true) {_finder2 = new math::pos_finder_equispaced(*_points2, *_interp, _points2_diff);}
	else                {_finder2 = new math::pos_finder_binary(*_points2, *_interp);}
	if (_equi1 == true) {_finder1 = new math::pos_finder_equispaced(*_points1, *_interp, _points1_diff);}
	else                {_finder1 = new math::pos_finder_binary(*_points1, *_interp);}

	switch (_interp_mode) {
		case math::logic::lagrange_first_precompute:
			fill_up_slopes_lagrange_first_precompute();
			_herm = new hermite3v();
			_functor_diff = new math::table3V_diff_prec(*this);
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			fill_up_slopes_hermite();
			_herm = new hermite3v(*_points3, *_points2, *_points1, *_VValues, *_slopes_d3, *_slopes_d2, *_slopes_d1, _interp_mode);
			_functor_diff = new math::table3V_diff_real(*this);
			break;
		case math::logic::spline: // splines not allowed in 3 dimensions
			destroy();
            throw std::runtime_error("Incorrect interpolation method.");
			break;
		default:
			_slopes_d3 = 0;
			_slopes_d2 = 0;
			_slopes_d1 = 0;
			_herm = new hermite3v();
			_functor_diff = new math::table3V_diff_real(*this);
			break;
	}
}
/**< initialization for constructors */

math::f_table3V::f_table3V(const f_table3V& other)
: _del_flag(other._del_flag),
_points3_diff(other._points3_diff), _equi3(other._equi3),
_points2_diff(other._points2_diff), _equi2(other._equi2),
_points1_diff(other._points1_diff), _equi1(other._equi1),
_interp_mode(other._interp_mode), _herm(0),
_slopes_d3(0), _slopes_d2(0), _slopes_d1(0),
_interp(math::interp::get_interp(_interp_mode)),
_checker3(other._checker3->clone()),
_checker2(other._checker2->clone()),
_checker1(other._checker1->clone()),
_finder3(other._finder3->clone()),
_finder2(other._finder2->clone()),
_finder1(other._finder1->clone()) {

	if (other._herm != 0) _herm = new hermite3v(*other._herm);

	if (other._slopes_d3 != 0) _slopes_d3 = other._slopes_d3->clone();
	if (other._slopes_d2 != 0) _slopes_d2 = other._slopes_d2->clone();
	if (other._slopes_d1 != 0) _slopes_d1 = other._slopes_d1->clone();

	if (_del_flag == true) {
		_points3 = other._points3->clone();
		_points2 = other._points2->clone();
		_points1 = other._points1->clone();
		_VValues = other._VValues->clone();
	}
	else {
		_points3 = other._points3;
		_points2 = other._points2;
		_points1 = other._points1;
		_VValues = other._VValues;
	}

	switch (_interp_mode) {
		case math::logic::lagrange_first_precompute:
			_functor_diff = new math::table3V_diff_prec(*this);
			break;
		case math::logic::hermite_first:
		case math::logic::hermite_second:
			_functor_diff = new math::table3V_diff_real(*this);
			break;
		default:
			_functor_diff = new math::table3V_diff_real(*this);
			break;
	}
}
/* copy constructor */

void math::f_table3V::destroy() {
	if (_del_flag == true) {
		delete _points3;
		delete _points2;
		delete _points1;
		delete _VValues;
	}
	delete _interp;
	delete _functor_diff;
	delete _slopes_d3;
	delete _slopes_d2;
	delete _slopes_d1;
	delete _herm;
	delete _checker3;
	delete _checker2;
	delete _checker1;
	delete _finder3;
	delete _finder2;
	delete _finder1;
}
/* destructor */

math::f_table3V* math::f_table3V::clone() const {
	return new f_table3V(*this);
}
/* cloner */

bool math::f_table3V::operator==(const pred3v& op2) const {
	return (op2.get_name() == math::logic::f_table3V) ?
		(*this == static_cast<const f_table3V&>(op2)) : false;
}
bool math::f_table3V::operator==(const f_table3V& op2) const {
	return ((*_points3 == *op2._points3) &&
		    (*_points2 == *op2._points2) && (*_points1 == *op2._points1) &&
			(*_VValues == *op2._VValues) && (_interp_mode == op2._interp_mode));
}
/* overloaded operator == (equal) */

int math::f_table3V::compute_pos3(const double& input3) const {
	return _interp->find_index(_finder3->search(*_points3, input3, _points3_diff),
								_points3->size1());
}
/* Returns first position within _points3 vector that shall be employed when
interpolating to obtain the result corresponding to the input magnitude. */

int math::f_table3V::compute_pos2(const double& input2) const {
	return _interp->find_index(_finder2->search(*_points2, input2, _points2_diff),
								_points2->size1());
}
/* Returns first position within _points2 vector that shall be employed when
interpolating to obtain the result corresponding to the input magnitude. */

int math::f_table3V::compute_pos1(const double& input1) const {
	return _interp->find_index(_finder1->search(*_points1, input1, _points1_diff),
								_points1->size1());
}
/* Returns first position within _points1 vector that shall be employed when
interpolating to obtain the result corresponding to the input magnitude. */

math::ratio* math::f_table3V::compute_ratio3(const double& input3,
												  const int& pos3) const {
	return _interp->compute_ratio(input3, *_points3, pos3);
}
/* Returns ratio of input3 with respect to the two points3 identified by pos3 and pos3+1. */

math::ratio* math::f_table3V::compute_ratio2(const double& input2,
												  const int& pos2) const {
	return _interp->compute_ratio(input2, *_points2, pos2);
}
/* Returns ratio of input2 with respect to the two points2 identified by pos2 and pos2+1. */

math::ratio* math::f_table3V::compute_ratio1(const double& input1,
												  const int& pos1) const {
	return _interp->compute_ratio(input1, *_points1, pos1);
}
/* Returns ratio of input1 with respect to the two points1 identified by pos1 and pos1+1. */

double math::f_table3V::compute_value(const int& pos3,
                                const int& pos2,
                                const int& pos1,
                                const math::ratio& ratio3,
                                const math::ratio& ratio2,
                                const math::ratio& ratio1) const {
	double res;
	_interp->interp3(res, *_points3, *_points2, *_points1, *_VValues, pos3, pos2, pos1, ratio3, ratio2, ratio1, *_herm);
    return res;
}
/* Fills up result magnitude by interpolating based on the input positions
and ratios */

double math::f_table3V::compute_diff(const int& pos3,
								  const int& pos2,
								  const int& pos1,
								  const double& input3,
								  const double& input2,
								  const double& input1,
								  const double& input3_dt,
								  const double& input2_dt,
								  const double& input1_dt) const {
	double res;
	_functor_diff->compute_diff(res, pos3, pos2, pos1,
						input3, input2, input1, input3_dt, input2_dt, input1_dt);
    return res;
}
/* Fills up result differential based on positions and ratios */

double math::f_table3V::value(const double& input3,
						   const double& input2,
						   const double& input1) const {
	// verify inputs are within range
	_checker3->check_range(*_points3, input3);
	_checker2->check_range(*_points2, input2);
	_checker1->check_range(*_points1, input1);
	// pos3 provides the upper vector position
	// pos2 provides the upper position within each vector
	// pos1 provides the upper position within each vector (within each matrix)
	int pos3 = compute_pos3(input3);
	int pos2 = compute_pos2(input2);
	int pos1 = compute_pos1(input1);
	math::ratio* ratio3 = compute_ratio3(input3, pos3);
	math::ratio* ratio2 = compute_ratio2(input2, pos2);
	math::ratio* ratio1 = compute_ratio1(input1, pos1);
    double res;
	_interp->interp3(res, *_points3, *_points2, *_points1, *_VValues, pos3, pos2, pos1, *ratio3, *ratio2, *ratio1, *_herm);
	_interp->to_pool(ratio3);
	_interp->to_pool(ratio2);
	_interp->to_pool(ratio1);
    return res;
}
/* evaluates the function at the reference magnitudes input3, input2, and
input1, and writes the result at the reference magnitude result. Only the
magnitude value is inserted into result, the units are assummed to be OK. */

double math::f_table3V::d_dt(const double& input3, const double& input2, const double& input1,
                       const double& input3_dt, const double& input2_dt, const double& input1_dt) const {
	////////////////////////////////////////////////////////////////////
	// The computation of the three positions, which are quite expensive,
	// in theory have already been done before in the value method, and
	// should not be repeated here. However, there is no way to avoid it.
	////////////////////////////////////////////////////////////////////
	// verify inputs are within range
	_checker3->check_range(*_points3, input3);
	_checker2->check_range(*_points2, input2);
	_checker1->check_range(*_points1, input1);

	// pos3 provides the upper vector position
	// pos2 provides the upper position within each vector
	// pos1 provides the upper position within each vector (within each matrix)
	int pos3 = compute_pos3(input3);
	int pos2 = compute_pos2(input2);
	int pos1 = compute_pos1(input1);
    double res;
	_functor_diff->compute_diff(res, pos3, pos2, pos1,
					input3, input2, input1, input3_dt, input2_dt, input1_dt);
    return res;
}
/* evaluates the function differential with time at the reference
magnitudes input3, input2, and input1 and their differentials with time
input3_dt, input2_dt, and input1_dt, and writes the result at the reference
magnitude result. */

void math::f_table3V::fill_up_slopes_lagrange_first_precompute() {
	// differentials with respect to first independent magnitude
	// has size (l) x (m) x (n)
	_slopes_d3 = new vec3(_points1->size1(), _points2->size1(), _points3->size1());
	for (unsigned short j = 0; j != _slopes_d3->size3()-1; ++j) { // last one is meaningless and remains 0
		fill_up_slopes_aux3_lagrange_first_precompute(*_slopes_d3, j, _points2->size1(), _points1->size1());
	}
	//coefficientv2* pdiffs2_dummy = new coefficientv2(_points2->size1());
	for (unsigned short i = 0; i != _points2->size1(); ++i) {
		//coefficientv1* pdiffs3_dummy = new coefficientv1(_points1->size1());
		for (unsigned short j = 0; j != _points1->size1(); ++j) {
			_slopes_d3->set(j, i, _slopes_d3->size3()-1, 0.);
		}
	}

	//_slopes_d3 = new vec3(_points3->size1());
	//for (unsigned short j = 0; j != _slopes_d3->size()-1; ++j) { // last one is meaningless and remains 0
	//	fill_up_slopes_aux3_lagrange_first_precompute(*_slopes_d3, j, _points2->size(), _points1->size());
	//}
	//coefficientv2* pdiffs2_dummy = new coefficientv2(_points2->size1());
	//for (unsigned short i = 0; i != pdiffs2_dummy->size(); ++i) {
	//	coefficientv1* pdiffs3_dummy = new coefficientv1(_points1->size1());
	//	for (unsigned short j = 0; j != pdiffs3_dummy->size(); ++j) {
	//		(*pdiffs3_dummy)[j].set_value(0.);
	//	}
	//	pdiffs2_dummy->set(i, pdiffs3_dummy);
	//}
	//_slopes_d3->set(_slopes_d3->size()-1, pdiffs2_dummy);

	// differentials with respect to second independent magnitude
	// has size (l) x (m) x (n)
	_slopes_d2 = new vec3(_points1->size1(), _points2->size1(), _points3->size1());
	for (unsigned short j = 0; j != _slopes_d2->size3(); ++j) {
		fill_up_slopes_aux2_lagrange_first_precompute(*_slopes_d2, j, _points2->size1(), _points1->size1());
	}
	// differentials with respect to third independent magnitude
	// has size (l) x (m) x (n)
	_slopes_d1 = new vec3(_points1->size1(), _points2->size1(), _points3->size1());
	for (unsigned short j = 0; j != _slopes_d1->size3(); ++j) {
		fill_up_slopes_aux1_lagrange_first_precompute(*_slopes_d1, j, _points2->size1(), _points1->size1());
	}
}
/* fills up the _slopes_d3, _slopes_d2, and _slopes_d1 attributes based on
_points3, _points2, _points1, and _VValues */

void math::f_table3V::fill_up_slopes_aux3_lagrange_first_precompute
				(math::vec3& slopes_d3,
				 unsigned short pos,
				 unsigned short siz2,
				 unsigned short siz3) {
    for (unsigned short i = 0; i != siz2; ++i) {
        for (unsigned short j = 0; j != siz3; ++j) {
            slopes_d3.set(j, i, pos, (_VValues->get(j, i, pos+1) - _VValues->get(j, i, pos)) / _finder3->diff(pos+1, pos));
        }
    }
}
/* fills up order "pos" of the _slopes_d3 3Dmatrix of differentials with respect to the
first independent magnitude with a matrix of size "siz2" by "siz3" */

void math::f_table3V::fill_up_slopes_aux2_lagrange_first_precompute
						(math::vec3& slopes_d2,
						 unsigned short pos,
						 unsigned short siz2,
						 unsigned short siz3) {
    for (unsigned short i = 0; i != siz2-1; ++i) {
        for (unsigned short j = 0; j != siz3; ++j) {
            slopes_d2.set(j, i, pos, (_VValues->get(j, i+1, pos) - _VValues->get(j, i, pos)) / _finder2->diff(i+1, i));
        }
    }
    for (unsigned short j = 0; j != siz3; ++j) {
        slopes_d2.set(j, siz2-1, pos, 0.);
    }
}
/* fills up order "pos" of the _slopes_d2 3Dmatrix of differentials with respect
to the second independent magnitude with a matrix of size "siz2" by "siz3" */

void math::f_table3V::fill_up_slopes_aux1_lagrange_first_precompute
						(math::vec3& slopes_d1,
						 unsigned short pos,
						 unsigned short siz2,
						 unsigned short siz3) {
    for (unsigned short i = 0; i != siz2; ++i) {
        for (unsigned short j = 0; j != siz3-1; ++j) {
            slopes_d1.set(j, i, pos, (_VValues->get(j+1, i, pos) - _VValues->get(j, i, pos)) / _finder1->diff(j+1, j));
        }
        slopes_d1.set(siz3-1, i, pos, 0.); // last value is meaningless
    }
}
/* fills up order "pos" of the _slopes_d1 3Dmatrix of differentials with respect
to the third independent magnitude with a matrix of size "siz2" by "siz3" */

void math::f_table3V::fill_up_slopes_hermite() {
	// differentials with respect to each of the three independent magnitudes
	// have size (l) x (m) x (n)
	_slopes_d3 = new vec3(_points1->size1(), _points2->size1(), _points3->size1());
	_slopes_d2 = new vec3(_points1->size1(), _points2->size1(), _points3->size1());
	_slopes_d1 = new vec3(_points1->size1(), _points2->size1(), _points3->size1());
	for (unsigned short i = 0; i != _points2->size1(); ++i) {
		for (unsigned short j = 0; j != _points1->size1(); ++j) {
			fill_up_slopes_aux3_hermite(*_slopes_d3, i, j);
		}
	}
	for (unsigned short i = 0; i != _points3->size1(); ++i) {
		for (unsigned short j = 0; j != _points1->size1(); ++j) {
			fill_up_slopes_aux2_hermite(*_slopes_d2, i, j);
		}
	}
	for (unsigned short i = 0; i != _points3->size1(); ++i) {
		for (unsigned short j = 0; j != _points2->size1(); ++j) {
			fill_up_slopes_aux1_hermite(*_slopes_d1, i, j);
		}
	}
}
/* fills up the _slopes_d3 and _slopes_d2 attributes based on _points3,
_points2, and _Values */

void math::f_table3V::fill_up_slopes_aux3_hermite(math::vec3& slopes_d3,
												 unsigned short index2,
												 unsigned short index1) {
	switch (_interp_mode) {
		case math::logic::hermite_first: {
			// compute temporary slopes as in Lagrange first order for each interval
			vec1 temp(_points3->size1()-1);
			for (unsigned short k = 0; k != _points3->size1()-1; ++k) {
				temp[k] = (_VValues->get(index1, index2, k+1) - _VValues->get(index1, index2, k))
									/ _finder3->diff(k+1, k);
			}
			// compute slopes by average of previous slopes
			slopes_d3.set(index1, index2, 0, temp[0]);
			for (unsigned short k = 1; k != _points3->size1()-1; ++k) {
				slopes_d3.set(index1, index2, k, 0.5 * (temp[k-1] + temp[k]));
			}
			slopes_d3.set(index1, index2, _points3->size1()-1, temp.back());;
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			unsigned short n = _points3->size1();
			slopes_d3.set(index1, index2, 0,
				 _VValues->get(index1, index2, 1) / _finder3->diff(2, 0, 1, 0, 2, 1) -
				 _VValues->get(index1, index2, 2) / _finder3->diff(1, 0, 2, 0, 2, 1) -
				 _VValues->get(index1, index2, 0) / _finder3->diff(2, 0) -
				 _VValues->get(index1, index2, 0) / _finder3->diff(1, 0));
			for (unsigned short i = 1; i < n-1; ++i) {
				slopes_d3.set(index1, index2, i,
				_VValues->get(index1, index2, i+1) / _finder3->diff(i,   i-1, i+1, i-1, i+1, i) -
				_VValues->get(index1, index2, i-1) / _finder3->diff(i+1, i,   i,   i-1, i+1, i-1) +
				_VValues->get(index1, index2, i)   / _finder3->diff(i,   i-1) -
				_VValues->get(index1, index2, i)   / _finder3->diff(i+1, i));
			}
			slopes_d3.set(index1, index2, n-1,
				_VValues->get(index1, index2, n-3) / _finder3->diff(n-1, n-2, n-2, n-3, n-1, n-3) -
				_VValues->get(index1, index2, n-2) / _finder3->diff(n-1, n-3, n-2, n-3, n-1, n-2) +
				_VValues->get(index1, index2, n-1) / _finder3->diff(n-1, n-3) +
				_VValues->get(index1, index2, n-1) / _finder3->diff(n-1, n-2));
			break; }
		default:
			throw std::runtime_error("Incorrect interpolation method.");
	}
}
/* fills up order "pos" of the _slopes_d3 3Dmatrix of differentials with respect to the
first independent magnitude with a matrix of size "siz2" by "siz3" */

void math::f_table3V::fill_up_slopes_aux2_hermite(math::vec3& slopes_d2,
												 unsigned short index3,
												 unsigned short index1) {
	switch (_interp_mode) {
		case math::logic::hermite_first: {
			// compute temporary slopes as in Lagrange first order for each interval
			vec1 temp(_points2->size1()-1);
			for (unsigned short k = 0; k != temp.size1(); ++k) {
				temp[k] = (_VValues->get(index1, k+1, index3) - _VValues->get(index1, k, index3))
										/ _finder2->diff(k+1, k);
			}
			// compute slopes by average of previous slopes
			slopes_d2.set(index1, 0, index3, temp[0]);
			for (unsigned short k = 1; k != temp.size1(); ++k) {
				slopes_d2.set(index1, k, index3, 0.5 * (temp[k-1] + temp[k]));
			}
			slopes_d2.set(index1, _points2->size1()-1, index3, temp.back());
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			unsigned short n = _points2->size1();
			slopes_d2.set(index1, 0, index3,
				 _VValues->get(index1, 1, index3) / _finder2->diff(2, 0, 1, 0, 2,1) -
				 _VValues->get(index1, 2, index3) / _finder2->diff(1, 0, 2, 0, 2,1) -
				 _VValues->get(index1, 0, index3) / _finder2->diff(2, 0) -
				 _VValues->get(index1, 0, index3) / _finder2->diff(1, 0));
			for (unsigned short i = 1; i < n-1; ++i) {
				slopes_d2.set(index1, i, index3,
				_VValues->get(index1, i+1, index3) / _finder2->diff(i,   i-1, i+1, i-1, i+1, i) -
				_VValues->get(index1, i-1, index3) / _finder2->diff(i+1, i,   i,   i-1, i+1, i-1) +
				_VValues->get(index1, i, index3)   / _finder2->diff(i,   i-1) -
				_VValues->get(index1, i, index3)   / _finder2->diff(i+1, i));
			}
			slopes_d2.set(index1, n-1, index3,
				_VValues->get(index1, n-3, index3) / _finder2->diff(n-1, n-2, n-2, n-3, n-1, n-3) -
				_VValues->get(index1, n-2, index3) / _finder2->diff(n-1, n-3, n-2, n-3, n-1, n-2) +
				_VValues->get(index1, n-1, index3) / _finder2->diff(n-1, n-3) +
				_VValues->get(index1, n-1, index3) / _finder2->diff(n-1, n-2));
			break; }
		default:
			throw std::runtime_error("Incorrect interpolation method.");
	}
}
/* fills up order "pos" of the _slopes_d2 3Dmatrix of differentials with respect
to the second independent magnitude with a matrix of size "siz2" by "siz3" */

void math::f_table3V::fill_up_slopes_aux1_hermite(math::vec3& slopes_d1,
												 unsigned short index3,
												 unsigned short index2) {
	switch (_interp_mode) {
		case math::logic::hermite_first: {
			// compute temporary slopes as in Lagrange first order for each interval
			vec1 temp(_points1->size1()-1);
			for (unsigned short k = 0; k != temp.size1(); ++k) {
				temp[k] = (_VValues->get(k+1, index2, index3) - _VValues->get(k, index2, index3))
										/ _finder1->diff(k+1, k);
			}
			// compute slopes by average of previous slopes
			slopes_d1. set(0, index2, index3, temp[0]);
			for (unsigned short k = 1; k != temp.size1(); ++k) {
				slopes_d1.set(k, index2, index3, 0.5 * (temp[k-1] + temp[k]));
			}
			slopes_d1.set(_points1->size1()-1, index2, index3, temp.back());
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			unsigned short n = _points1->size1();
			slopes_d1.set(0, index2, index3,
				 _VValues->get(1, index2, index3) / _finder1->diff(2, 0, 1, 0, 2, 1) -
				 _VValues->get(2, index2, index3) / _finder1->diff(1, 0, 2, 0, 2, 1) -
				 _VValues->get(0, index2, index3) / _finder1->diff(2, 0) -
				 _VValues->get(0, index2, index3) / _finder1->diff(1, 0));
			for (unsigned short i = 1; i < n-1; ++i) {
				slopes_d1.set(i, index2, index3,
				_VValues->get(i+1, index2, index3) / _finder1->diff(i,   i-1, i+1, i-1, i+1, i) -
				_VValues->get(i-1, index2, index3) / _finder1->diff(i+1, i,   i, i-1,   i+1, i-1) +
				_VValues->get(i, index2, index3)   / _finder1->diff(i,   i-1) -
				_VValues->get(i, index2, index3)   / _finder1->diff(i+1, i));
			}
			slopes_d1.set(n-1, index2, index3,
				_VValues->get(n-3, index2, index3) / _finder1->diff(n-1, n-2, n-2, n-3, n-1, n-3) -
				_VValues->get(n-2, index2, index3) / _finder1->diff(n-1, n-3, n-2, n-3, n-1, n-2) +
				_VValues->get(n-1, index2, index3) / _finder1->diff(n-1, n-3) +
				_VValues->get(n-1, index2, index3) / _finder1->diff(n-1, n-2));
			break; }
		default:
			throw std::runtime_error("Incorrect interpolation method.");
	}
}
/* fills up order "pos" of the _slopes_d1 3Dmatrix of differentials with respect
to the third independent magnitude with a matrix of size "siz2" by "siz3" */

void math::f_table3V::activate_checker3() {
	delete _checker3;
	_checker3 = new math::range_checker_active();
}
void math::f_table3V::deactivate_checker3() {
	delete _checker3;
	_checker3 = new math::range_checker_inactive();
}
/* Activates or deactives the out of range verification for _points3, which
is inactive by default */

void math::f_table3V::activate_checker2() {
	delete _checker2;
	_checker2 = new math::range_checker_active();
}
void math::f_table3V::deactivate_checker2() {
	delete _checker2;
	_checker2 = new math::range_checker_inactive();
}
/* Activates or deactives the out of range verification for _points2, which
is active by default */

void math::f_table3V::activate_checker1() {
	delete _checker1;
	_checker1 = new math::range_checker_active();
}
void math::f_table3V::deactivate_checker1() {
	delete _checker1;
	_checker1 = new math::range_checker_inactive();
}
/* Activates or deactives the out of range verification for _points1, which
is active by default */


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE3V_DIFF_PREC
// =======================
// =======================

math::table3V_diff_prec::table3V_diff_prec(math::f_table3V& pred)
: _pred(&pred) {}
/* constructor based on three dimensional table */

math::table3V_diff_prec::~table3V_diff_prec() {
}
/* destructor */

void math::table3V_diff_prec::compute_diff(double& result,
											  const int& pos3,
											  const int& pos2,
											  const int& pos1,
											  const double& input3,
											  const double& input2,
											  const double& input1,
											  const double& input3_dt,
											  const double& input2_dt,
											  const double& input1_dt) const {
	math::ratio_linear* ratio3 = static_cast<math::ratio_linear*>(_pred->compute_ratio3(input3, pos3));
	math::ratio_linear* ratio2 = static_cast<math::ratio_linear*>(_pred->compute_ratio2(input2, pos2));
	math::ratio_linear* ratio1 = static_cast<math::ratio_linear*>(_pred->compute_ratio1(input1, pos1));
    // Differential with respect to 1st independent magnitude
    double temp, temp1, temp2;
    math::interp::basic_interp_lagrange_first(temp1, _pred->_slopes_d3->get(pos1, pos2, pos3),
                                                   _pred->_slopes_d3->get(pos1+1, pos2, pos3), ratio1->_ratio12);
    math::interp::basic_interp_lagrange_first(temp2, _pred->_slopes_d3->get(pos1, pos2+1, pos3),
                                                   _pred->_slopes_d3->get(pos1+1, pos2+1, pos3), ratio1->_ratio12);
    math::interp::basic_interp_lagrange_first(temp, temp1, temp2, ratio2->_ratio12);
    result = temp * input3_dt;
    // Differential with respect to 2nd independent magnitude
    math::interp::basic_interp_lagrange_first(temp1, _pred->_slopes_d2->get(pos1, pos2, pos3),
                                                   _pred->_slopes_d2->get(pos1+1, pos2, pos3), ratio1->_ratio12);
    math::interp::basic_interp_lagrange_first(temp2, _pred->_slopes_d2->get(pos1, pos2, pos3+1),
                                                   _pred->_slopes_d2->get(pos1+1, pos2, pos3+1), ratio1->_ratio12);
    math::interp::basic_interp_lagrange_first(temp, temp1, temp2, ratio3->_ratio12);
    result = result + temp * input2_dt;
    // Differential with respect to 3rd independent magnitude
    math::interp::basic_interp_lagrange_first(temp1, _pred->_slopes_d1->get(pos1, pos2, pos3),
                                                   _pred->_slopes_d1->get(pos1, pos2+1, pos3), ratio2->_ratio12);
    math::interp::basic_interp_lagrange_first(temp2, _pred->_slopes_d1->get(pos1, pos2, pos3+1),
                                                   _pred->_slopes_d1->get(pos1, pos2+1, pos3+1), ratio2->_ratio12);
    math::interp::basic_interp_lagrange_first(temp, temp1, temp2, ratio3->_ratio12);
    result = result + temp * input1_dt;
	math::ratio_mgr::to_pool_linear(ratio3);
	math::ratio_mgr::to_pool_linear(ratio2);
	math::ratio_mgr::to_pool_linear(ratio1);
}
/* fill up result differential based on positions, inputs, and its partial
differentials with time */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS TABLE3V_DIFF_REAL
// =======================
// =======================

math::table3V_diff_real::table3V_diff_real(math::f_table3V& pred)
: _pred(&pred) {}
/* constructor based on three dimensional table */

math::table3V_diff_real::~table3V_diff_real() {
}
/* destructor */

void math::table3V_diff_real::compute_diff(double& result,
											  const int& pos3,
											  const int& pos2,
											  const int& pos1,
											  const double& input3,
											  const double& input2,
											  const double& input1,
											  const double& input3_dt,
											  const double& input2_dt,
											  const double& input1_dt) const {
	math::ratio* ratio3 = _pred->compute_ratio3(input3, pos3);
	math::ratio* ratio2 = _pred->compute_ratio2(input2, pos2);
	math::ratio* ratio1 = _pred->compute_ratio1(input1, pos1);
	math::ratio* ratioX3 = _pred->_interp->copy_ratio(ratio3);
	math::ratio* ratioX2 = _pred->_interp->copy_ratio(ratio2);
	math::ratio* ratioX1 = _pred->_interp->copy_ratio(ratio1);
	double Pmag1, PmagX;
    // compute result at point
    _pred->_interp->interp3(Pmag1, *_pred->_points3, *_pred->_points2, *_pred->_points1, *_pred->_VValues, pos3, pos2, pos1, *ratio3, *ratio2, *ratio1, *_pred->_herm);
    // compute result a short interval after point in direction 1
    double difX = _pred->_finder3->compute_Dratio(*ratioX3, input3, pos3);
    _pred->_interp->interp3(PmagX, *_pred->_points3, *_pred->_points2, *_pred->_points1, *_pred->_VValues, pos3, pos2, pos1, *ratioX3, *ratio2, *ratio1, *_pred->_herm);
    result = (PmagX - Pmag1) * _pred->_finder3->compute_final_diff(input3_dt, difX);
    // compute result a short interval after point in direction 2
    difX = _pred->_finder2->compute_Dratio(*ratioX2, input2, pos2);
    _pred->_interp->interp3(PmagX, *_pred->_points3, *_pred->_points2, *_pred->_points1, *_pred->_VValues, pos3, pos2, pos1, *ratio3, *ratioX2, *ratio1, *_pred->_herm);
    result = result + (PmagX - Pmag1) * _pred->_finder2->compute_final_diff(input2_dt, difX);
    // compute result a short interval after point in direction 3
    difX = _pred->_finder1->compute_Dratio(*ratioX1, input1, pos1);
    _pred->_interp->interp3(PmagX, *_pred->_points3, *_pred->_points2, *_pred->_points1, *_pred->_VValues, pos3, pos2, pos1, *ratio3, *ratio2, *ratioX1, *_pred->_herm);
    result = result + (PmagX - Pmag1) * _pred->_finder1->compute_final_diff(input1_dt, difX);
	// release memory
	_pred->_interp->to_pool(ratio3);
	_pred->_interp->to_pool(ratio2);
	_pred->_interp->to_pool(ratio1);
	_pred->_interp->to_pool(ratioX3);
	_pred->_interp->to_pool(ratioX2);
	_pred->_interp->to_pool(ratioX1);
}
/* fill up result differential based on positions, inputs, and its partial
differentials with time */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




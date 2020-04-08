#include "vec.h"

// CLASS VEC
// =========
// =========

math::vec::vec(int siz)
: _vec(siz) { 	
}
/* constructor based on vector size */

math::vec::vec(const vec& other)
: _vec(other._vec.size()) {
	for (int i = 0; i != other._vec.size(); ++i) {
		if (other._vec[i] != 0) {
			_vec[i] = other._vec[i];
		}
		else _vec[i] = 0;
	}	
}
/* copy constructor */

math::vec::~vec() {
}
/* destructor */

bool math::vec::operator==(const vec& op2) const {
	if (_vec.size() != op2._vec.size())	{return false;}
	for (int i = 0; i != _vec.size(); ++i) {
		if (_vec[i] != op2._vec[i]) {return false;}
	}
	return true;
}
/* overloaded operator == (equal) */

std::vector<double>::const_iterator math::vec::begin() const {
	return _vec.begin();
}
/* returns constant iterator to the first magnitude pointer */

std::vector<double>::const_iterator math::vec::end() const {
	return _vec.end();
}
/* returns constant iterator to the last magnitude pointer */

const double& math::vec::front() const {
    return _vec.front();
}
double& math::vec::front() {
	return _vec.front();
}
/* returns reference to first element */

const double& math::vec::back() const {
    return _vec.back();
}
double& math::vec::back() {
	return _vec.back();
}
/* returns reference to last element */

double math::vec::diff (int siz1, int siz2) const {
	return (_vec[siz1] -  _vec[siz2]);
}
/* obtains the shortest numeric difference between the magnitudes 
positioned in pos1 and pos2. */

const double& math::vec::operator[](int index1) const {
    return _vec[index1];
}
double& math::vec::operator[](int index1) {
    return _vec[index1];
}
/* get a magnitude reference from the position provided by index1 */









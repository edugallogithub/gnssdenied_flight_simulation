#include "vec3.h"

// CLASS VEC3
// ==========
// ==========

math::vec3::vec3() : vec(0), _siz1(0), _siz2(0), _siz3(0), _siz12(0) {
}
/* empty constructor creates vector of size 0 */

math::vec3::vec3(unsigned int siz1, unsigned int siz2, unsigned int siz3)
: vec(siz1 * siz2 * siz3),
_siz1(siz1), _siz2(siz2), _siz3(siz3), _siz12(siz1 * siz2) {
    for (int i = 0; i != siz1 * siz2 * siz3; ++i) {
        _vec[i] = 0.;
    }
}
/* constructor based on matrix dimensions */

math::vec3::vec3(const std::string &st) : vec(0), _siz1(0), _siz2(0), _siz3(0), _siz12(0) {
    std::ifstream mystream(st.c_str());
    create(mystream, st);
    mystream.close();
}
/* constructor based on a string identifying a text file containing the
required info. The first number contains the size of each vector, the second
the number of vectors in each bidimensional matrix, and the third the
number of bidimensional matrixes, followed by its contents in standard units. */

math::vec3::vec3(std::ifstream &mystream) : vec(0), _siz1(0), _siz2(0), _siz3(0), _siz12(0) {
    create(mystream, "");
}
/* constructor based on an open stream containing the required info.
The first number contains the size of each vector, the second
the number of vectors in each bidimensional matrix, and the third the
number of bidimensional matrixes, followed by its contents in standard units. */

math::vec3::vec3(const vec3& other)
: vec(other), _siz1(other._siz1), _siz2(other._siz2), _siz3(other._siz3),
_siz12(other._siz12) {
}
/* copy constructor */

math::vec3& math::vec3::operator=(const vec3& op2) {
	_siz1 = op2._siz1;
	_siz2 = op2._siz2;
	_siz3 = op2._siz3;
	_siz12 = op2._siz12;
	int siz = _siz12 * _siz3;
	_vec	= std::vector<double>(siz);
	for (unsigned int i = 0; i != siz ; ++i) {
		_vec[i]	= op2._vec[i];
	}
	return *this;
}
/* overloaded operator = (assignment) */

bool math::vec3::operator==(const vec3& op2) const {
	return (static_cast<const vec&>(*this) == static_cast<const vec&>(op2));
}
/* overloaded operator == (equal) */

math::vec3::~vec3() {
}
/* destructor */

void math::vec3::set(unsigned int index1, unsigned int index2, unsigned int index3, double value) {
    _vec[_siz12 * index3 + _siz1 * index2 + index1] = value;
}
/* change the value in standard units in the position provided by index1, index2, and index3 */

/* ===== ===== Private Methods ===== ===== */

void math::vec3::create(std::ifstream &mystream, const std::string &st) {
    // do not replace by unsigned short as it does not work if text file has decimals for
    // the sizes, as in 3.0 instead of 3.
    double siz1 = 0, siz2 = 0, siz3 = 0;
    mystream >> siz1 >> siz2 >> siz3;
    _siz1 = siz1;
    _siz2 = siz2;
    _siz3 = siz3;
    _siz12 = _siz1 * _siz2;
    int siz = _siz12 * _siz3;
    _vec = std::vector<double>(siz);
    double temp = 0.;
    for (int i = 0; i != siz; ++i) {
        mystream >> temp;
        _vec[i] = temp;
    }
}
/* initialization based on an open stream containing the required info.
The first number contains the size of each vector, the second the number of
vectors in each bidimensional matrix, and the third the	number of
bidimensional matrixes, followed by its contents in standard units.
Name of the text file containing the info is optional*/






#include "vec2.h"

// CLASS VEC2
// ==========
// ==========

math::vec2::vec2() : vec(0), _siz1(0), _siz2(0) {
}
/* empty constructor creates vector of size 0 */

math::vec2::vec2(unsigned int siz1, unsigned int siz2)
: vec(siz1 * siz2), _siz1(siz1), _siz2(siz2) {
    for (int i = 0; i != siz1*siz2; ++i) {
        _vec[i] = 0.;
    }
}
/* constructor based on matrix dimensions */

math::vec2::vec2(const std::string& st) : vec(0), _siz1(0), _siz2(0) {
    std::ifstream mystream(st.c_str());
    create(mystream, st);
    mystream.close();
}
/* constructor based on a string identifying a text file containing the
required info. The first number contains the size of each vector and the
second the number of vectors, followed by its contents in standard units. */

math::vec2::vec2(std::ifstream& mystream) : vec(0), _siz1(0), _siz2(0) {
    create(mystream, "");
}
/* constructor based on an open stream containing the required info.
The first number contains the size of each vector and the second the
number of vectors, followed by its contents in standard units.
Name of the text file containing the info is optional*/

math::vec2::vec2(unsigned int siz1, unsigned int siz2, std::ifstream &mystream)
: vec(0), _siz1(siz1), _siz2(siz2) {
    create_bis(mystream);
}
/* constructor based on matrix sizes and input stream containing data */

math::vec2::vec2(const vec2& other)
: vec(other), _siz1(other._siz1), _siz2(other._siz2) {
}
/* copy constructor */

math::vec2& math::vec2::operator=(const vec2& op2) {
	_siz1 = op2._siz1;
	_siz2 = op2._siz2;
	int siz = _siz1 * _siz2;
	_vec	= std::vector<double>(siz);
	for (unsigned int i = 0; i != siz ; ++i) {
		_vec[i]	= op2._vec[i];
	}
	return *this;
}
/* overloaded operator = (assignment) */

bool math::vec2::operator==(const vec2& op2) const {
	return (static_cast<const vec&>(*this) == static_cast<const vec&>(op2));
}
/* overloaded operator == (equal) */

math::vec2::~vec2() {
}
/* destructor */

void math::vec2::set(unsigned int index1, unsigned int index2, double value) {
    _vec[_siz1 * index2 + index1] = value;
}
/* change the value in standard units in the position provided by
index1 and index2 */

/* ===== ===== Private Methods ===== ===== */

void math::vec2::create(std::ifstream& mystream, const std::string& st) {
    // do not replace by unsigned short as it does not work if text file has decimals for
    // the sizes, as in 3.0 instead of 3.
    double siz1 = 0, siz2 = 0;
    mystream >> siz1 >> siz2;
    _siz1 = siz1;
    _siz2 = siz2;
    int siz = _siz1 * _siz2;
    _vec = std::vector<double>(siz);
    double temp = 0.;
    for (int i=0; i!=siz; ++i) {
        mystream >> temp;
        _vec[i] = temp;
    }
}
/* initialization based on an open stream containing the required info.
The first number contains the size of each vector and the second the
number of vectors, followed by its contents in standard units.
Name of the text file containing the info is optional*/

void math::vec2::create_bis(std::ifstream& mystream) {
    int siz = _siz1 * _siz2;
    _vec = std::vector<double>(siz);
    double temp = 0.;
    for (int i=0; i!=siz; ++i) {
        mystream >> temp;
        _vec[i] = temp;
    }
}
/* initialization based on an open stream containing the contents
* in standard units (without the sizes first, as above). */





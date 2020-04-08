#include "vec1.h"

// CLASS MAGV1 (unidimensional vector of mag pointers)
// ===================================================
// ===================================================

math::vec1::vec1() : vec(0), _siz1(0) {
}
/* empty constructor creates vector of size 0 */

math::vec1::vec1(unsigned int siz1) : vec(siz1), _siz1(siz1) {
    for (unsigned int i = 0; i != _siz1; ++i) {
        _vec[i] = 0.;
    }
}
/* constructor based on vector size */

math::vec1::vec1(const std::string& st) : vec(0), _siz1(0) {
    std::ifstream mystream(st.c_str());
    create(mystream, st);
    mystream.close();
}
/* constructor based on a string identifying a text file containing the
required info. The first number contains the size of the vector, followed
by its contents in standard units. */

math::vec1::vec1(std::ifstream& mystream) : vec(0), _siz1(0) {
    create(mystream, "");
}
/* constructor based on an open stream containing the required info.
The first number contains the size of the vector, followed by its contents
in standard units. Name of the text file containing the info is optional*/

math::vec1::vec1(const vec1& other)
: vec(other), _siz1(other._siz1) {
}
/* copy constructor */

math::vec1& math::vec1::operator=(const vec1& op2) {
	_siz1 = op2._siz1;
	_vec	= std::vector<double>(_siz1);
	for (unsigned int i = 0; i != _siz1 ; ++i) {
		_vec[i]	= op2._vec[i];
	}
	return *this;
}
/* overloaded operator = (assignment) */

math::vec1::~vec1() {}
/* destructor */

bool math::vec1::operator==(const vec1& op2) const {
	return (static_cast<const vec&>(*this) == static_cast<const vec&>(op2));
}
/* overloaded operator == (equal) */

math::vec1* math::vec1::create(unsigned int siz) {
    return new vec1(siz);
}
/* returns default altitude vector pointer (for magnitude factory) */

void math::vec1::set(unsigned int index, double value) {
    _vec[index] = value;
}
/* change the value in [m] in the position provided by siz */

/* ===== ===== Private Methods ===== ===== */
void math::vec1::create(std::ifstream& mystream, const std::string& st) {
    // do not replace by unsigned short as it does not work if text file has decimals for
    // the sizes, as in 3.0 instead of 3.
    double siz = 0;
    mystream >> siz;
    _siz1 = siz;
    _vec = std::vector<double>(siz);
    double temp = 0.;
    for (unsigned int i = 0; i != siz; ++i) {
        mystream >> temp;
        _vec[i] = temp;
    }
}
/* initialization based on an open stream containing the required info.
The first number contains the size of the vector, followed by its contents
in standard units. Name of the text file containing the info is optional*/




#ifndef MATH_VEC1
#define MATH_VEC1

#include "../math.h"
#include "vec.h"

/*
This file contains the virtual class vec1, intended to contain unidimensional
vectors.
*/

namespace math {

// CLASS VEC1 (unidimensional vector)
// ==================================
// ==================================

class MATH_API vec1: public vec {
protected:
	int _siz1;
	/**< size in 1st dimension */

    void create(std::ifstream& mystream, const std::string& st);
    /**< initialization based on an open stream containing the required info.
    The first number contains the size of the vector, followed by its contents
    in standard units. Name of the text file containing the info is optional*/
public:
    vec1();
    /**< empty constructor */
    vec1(unsigned int siz1);
    /**< constructor based on vector size */
    vec1(const std::string& st);
    /**< constructor based on a string identifying a text file containing the
    required info. The first number contains the size of the vector, followed
    by its contents in standard units. */
    vec1(std::ifstream& mystream);
    /**< constructor based on an open stream containing the required info.
    The first number contains the size of the vector, followed by its contents
    in standard units. Name of the text file containing the info is optional*/
    vec1(const vec1&);
	/**< copy constructor */
	vec1& operator=(const vec1& op2);
	/**< overloaded operator = (assignment) */
	~vec1();
	/**< destructor */
	bool operator==(const vec1& op2) const;
	/**< overloaded operator == (equal) */
	bool operator!=(const vec1& op2) const	{return !(*this == op2);}
	/**< overloaded operator != (not equal) */
	vec1* clone() const {return new vec1(*this);};
	/**< cloner */
    static vec1* create(unsigned int siz);
    /**< returns default F vector pointer (for magnitude factory) */

	void set(unsigned int index, double value);
	/**< change the value in [m] in the position provided by siz */
	const int size1() const {return _siz1;}
	/**< returns vector size */
    const double& operator[](unsigned int index1) const {return _vec[index1];}
    double& operator[](unsigned int index1) {return _vec[index1];}
	/**< get a magnitude reference from the position provided by index1 */
    const double& get(unsigned int index1) const {return _vec[index1];}
    double& get(unsigned int index1) {return _vec[index1];}
	/**< get a magnitude reference from the position provided by index1 */
}; // closes class vec1

} // closes namespace math

inline std::ostream& operator<<(std::ostream& stream,
								const math::vec1& Ovec1) {
	stream << "size " << Ovec1.size1() << " vector of " << " magnitudes";;
	return stream;
}
/**< overloaded operator << (inserter): no need for friend of class vec1 as it
does not use private members of mag. */

#endif


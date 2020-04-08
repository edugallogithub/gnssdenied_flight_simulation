#ifndef MATH_VEC2
#define MATH_VEC2

#include "../math.h"
#include "vec1.h"

/*
This file contains the virtual class vec2, intended to contain bidimensional
vectors.
*/

namespace math {

// CLASS VEC2 (bidimensional vector)
// =================================
// =================================

class MATH_API vec2: public vec {
protected:
	int _siz1;
	/**< size in 1st dimension */
	int _siz2;
	/**< size in 2nd dimension */

    void create(std::ifstream& mystream, const std::string& st);
    /**< initialization based on an open stream containing the required info.
    The first number contains the size of each vector and the second the
    number of vectors, followed by its contents in standard units.
    Name of the text file containing the info is optional*/
    void create_bis(std::ifstream& mystream);
    /**< initialization based on an open stream containing the contents
     * in standard units (without the sizes first, as above). */
public:
    vec2();
    /**< empty constructor */
    vec2(unsigned int siz1, unsigned intsiz2);
    /**< constructor based on matrix dimensions */
    vec2(const vec2&);
    /**< copy constructor */
    vec2(const std::string& st);
    /**< constructor based on a string identifying a text file containing the
    required info. The first number contains the size of each vector and the
    second the number of vectors, followed by its contents in standard units. */
    vec2(std::ifstream& mystream);
    /**< constructor based on an open stream containing the required info.
    The first number contains the size of each vector and the second the
    number of vectors, followed by its contents in standard units.
    Name of the text file containing the info is optional*/
    vec2(unsigned int siz1, unsigned int siz2, std::ifstream &mystream);
    /**< constructor based on matrix sizes and input stream containing data */
	vec2& operator=(const vec2& op2);
	/**< overloaded operator = (assignment) */
	~vec2();
	/**< destructor */
	bool operator==(const vec2& op2) const;
	/**< overloaded operator == (equal) */
	bool operator!=(const vec2& op2) const {return !(*this == op2);}
	/**< overloaded operator != (not equal) */
	vec2* clone() const {return new vec2(*this);}
	/**< cloner */
	math::vec1* sample_vector() const {return new math::vec1(this->size());}
	/**< returns pointer of magnitude vector of proper type and size n */

	void set(unsigned int index1, unsigned int index2, double value);
	/**< change the value in standard units in the position provided by
	index1 and index2 */
	const int size1() const {return _siz1;}
	const int size2() const {return _siz2;}
	/**< returns size in different dimensions */
	int index(unsigned int index1, unsigned int index2) const
	{return _siz1 * index2 + index1;}
	/**< returns position within vec of combination of two inputs */
    const double& get(unsigned int index1, unsigned int index2) const {return _vec[_siz1 * index2 + index1];}
    double& get(unsigned int index1, unsigned int index2) {return _vec[_siz1 * index2 + index1];}
	/**< get a magnitude reference from the positions provided by index1 and index2 */
}; // closes class vec2

} // closes namespace math

#endif


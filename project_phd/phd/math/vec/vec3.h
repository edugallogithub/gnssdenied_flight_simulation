#ifndef MATH_VEC3
#define MATH_VEC3

#include "../math.h"
#include "vec2.h"

/*
This file contains the virtual class vec3, intended to contain tridimensional
vectors.
*/

namespace math {

// CLASS VEC3 (tridimensional vector)
// ==================================
// ==================================

class MATH_API vec3: public vec {
protected:
	int _siz1;
	/**< size in 1st dimension */
	int _siz2;
	/**< size in 2nd dimension */
	int _siz3;
	/**< size in 3rd dimension */
	int _siz12;
	/**< product of _siz1 * _siz2 to accelerate internal computations */

    void create(std::ifstream& mystream, const std::string& st);
    /**< initialization based on an open stream containing the required info.
    The first number contains the size of each vector, the second the number of
    vectors in each bidimensional matrix, and the third the	number of
    bidimensional matrixes, followed by its contents in standard units.
    Name of the text file containing the info is optional*/
public:
    vec3();
    /**< empty constructor */
    vec3(unsigned int siz1, unsigned int siz2, unsigned int siz3);
    /**< constructor based on matrix dimensions */
    vec3(const vec3&);
    /**< copy constructor */
    vec3(const std::string& st);
    /**< constructor based on a string identifying a text file containing the
    required info. The first number contains the size of each vector, the second
    the number of vectors in each bidimensional matrix, and the third the
    number of bidimensional matrixes, followed by its contents in standard units. */
    vec3(std::ifstream& mystream);
    /**< constructor based on an open stream containing the required info.
    The first number contains the size of each vector, the second
    the number of vectors in each bidimensional matrix, and the third the
    number of bidimensional matrixes, followed by its contents in standard units. */
	vec3& operator=(const vec3& op2);
	/**< overloaded operator = (assignment) */
	~vec3();
	/**< destructor */
	bool operator==(const vec3& op2) const;
	/**< overloaded operator == (equal) */
	bool operator!=(const vec3& op2) const {return !(*this == op2);}
	/**< overloaded operator != (not equal) */
	vec3* clone() const {return new vec3(*this);}
	/**< cloner */
	void set(unsigned int index1, unsigned int index2, unsigned int index3, double value);
    /**< change the value in standard units in the position provided by
	index1, index2, and index3 */
	const int size1() const {return _siz1;}
	const int size2() const {return _siz2;}
	const int size3() const {return _siz3;}
	/**< returns size in different dimensions */
	int index(unsigned int index1, unsigned int index2, unsigned int index3) const
	{return _siz12 * index3 + _siz1 * index2 +  index1;}
	/**< returns position within vec of combination of two inputs */
    const double& get(unsigned int index1, unsigned int index2, unsigned int index3) const {return _vec[_siz12 * index3 + _siz1 * index2 + index1];}
    double& get(unsigned int index1, unsigned int index2, unsigned int index3) {return _vec[_siz12 * index3 + _siz1 * index2 + index1];}
	/**< get a magnitude reference from the positions provided by
	index1, index2, and index3 */
}; // closes class vec3

} // closes namespace math


#endif




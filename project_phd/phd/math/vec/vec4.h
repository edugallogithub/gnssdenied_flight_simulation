#ifndef MATH_VEC4
#define MATH_VEC4

#include "../math.h"
#include "vec3.h"

/*
This file contains the virtual class vec4, intended to contain quatridimensional
vectors.
*/

namespace math {

// CLASS VEC4 (quatridimensional vector)
// ====================================
// ====================================

class MATH_API vec4: public vec {
protected:
	int _siz1;
	/**< size in 1st dimension */
	int _siz2;
	/**< size in 2nd dimension */
	int _siz3;
	/**< size in 3rd dimension */
	int _siz4;
	/**< size in 4th dimension */

	int _siz123;
	/**< product of _siz1 * _siz2 * _siz3 to accelerate internal computations */
	int _siz12;
	/**< product of _siz1 * _siz2 to accelerate internal computations */

    void create(std::ifstream& mystream, const std::string& st);
    /**< initialization based on an open stream containing the required info.
    The first number contains the size of each vector, the second the number of
    vectors in each bidimensional matrix, the third the number of bidimensional
    matrixes, and the fourth the last dimension, followed by its contents in
    standard units. Name of the text file containing the info is optional*/
public:
    vec4();
    /**< empty constructor */
    vec4(unsigned int siz1, unsigned int siz2, unsigned int siz3, unsigned int siz4);
    /**< constructor based on matrix dimensions */
    vec4(const std::string& st);
    /**< constructor based on a string identifying a text file containing the
    required info. The first number contains the size of each vector, the second
    the number of vectors in each bidimensional matrix, the third the
    number of bidimensional matrixes, and the fourth the last dimension,
    followed by its contents in standard units. */
    vec4(std::ifstream& mystream);
    /**< constructor based on an open stream containing the required info.
    The first number contains the size of each vector, the second the number
    of vectors in each bidimensional matrix, the third the number of
    bidimensional matrixes, and the fourth the last dimension, followed by
    its contents in standard units. */
    vec4(const vec4&);
    /**< copy constructor */

	vec4& operator=(const vec4& op2);
	/**< overloaded operator = (assignment) */
	~vec4();
	/**< destructor */
	bool operator==(const vec4& op2) const;
	/**< overloaded operator == (equal) */
	bool operator!=(const vec4& op2) const {return !(*this == op2);}
	/**< overloaded operator != (not equal) */
	vec4* clone() const {return new vec4(*this);}
	/**< cloner */

	void set(unsigned int index1, unsigned int index2, unsigned int index3, unsigned int index4, double value);
	/**< change the value in standard units in the position provided by index1, index2, index3, and index4 */
	const int size1() const {return _siz1;}
	const int size2() const {return _siz2;}
	const int size3() const {return _siz3;}
	const int size4() const {return _siz4;}
	/**< returns size in different dimensions */
	int index(unsigned int index1, unsigned int index2, unsigned int index3, unsigned int index4) const
	{return _siz123 * index4 + _siz12 * index3 + _siz1 * index2 +  index1;}
	/**< returns position within vec of combination of two inputs */
    const double& get(unsigned int index1, unsigned int index2, unsigned int index3, unsigned int index4) const
    {return _vec[_siz123 * index4 + _siz12 * index3 + _siz1 * index2 + index1];}
    double& get(unsigned int index1, unsigned int index2, unsigned int index3, unsigned int index4)
    {return _vec[_siz123 * index4 + _siz12 * index3 + _siz1 * index2 + index1];}
	/**< get a magnitude reference from the positions provided by index1, index2, index3, and index4 */
}; // closes class vec4

} // closes namespace math

#endif




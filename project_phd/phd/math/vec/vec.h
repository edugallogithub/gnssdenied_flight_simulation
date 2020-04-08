#ifndef MATH_VEC
#define MATH_VEC

#include "../math.h"
#include "algorithm.h"
#include "../interp/ratio.h"
#include <algorithm>
#include <vector>

/*
This file contains the virtual class vec, intended to contain multidimensional vectors.
*/

namespace math {

// CLASS VEC
// =========
// =========

class MATH_API vec {
protected:
	std::vector<double> _vec;
	/**< vector of pointer to magnitude objects */

    vec();
    /**< empty constructor not implemented*/
    vec(int siz);
    /**< constructor based on vector size */
	vec(const vec&);
	/**< copy constructor */
	vec& operator=(const vec& op2);
	/**< overloaded operator = (assignment) not implemented */
public:
	virtual ~vec();
	/**< destructor */
	virtual bool operator==(const vec& op2) const;
	/**< overloaded operator == (equal) */
	virtual bool operator!=(const vec& op2) const {return !(*this == op2);}
	/**< overloaded operator != (not equal) */

    virtual const int size() const {return _vec.size();}
    /**< returns vector size */

    virtual std::vector<double>::const_iterator begin() const;
	/**< returns constant iterator to the first magnitude pointer */
	virtual std::vector<double>::const_iterator end() const;
	/**< returns constant iterator to the last magnitude pointer */

    virtual const double& front() const;
    virtual double& front();
	/**< returns reference to first element */
    virtual const double& back() const;
    virtual double& back();
	/**< returns reference to last element */

    const double& operator[](int index1) const;
    double& operator[](int index1);
	/**< get a magnitude reference from the position provided by index1 */

	virtual double diff (int siz1, int siz2) const;
	/**< obtains the shortest numeric difference between the magnitudes
	positioned in pos1 and pos2. */
}; // closes class vec

} // closes namespace math

#endif


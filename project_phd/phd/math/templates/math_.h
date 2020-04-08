#ifndef MATH_MATH
#define MATH_MATH

#include "../math.h"
#include <sstream>
#include <iomanip>

/*
This file contains several template functions:
- str transforms the input into a string.
- sign returns the second input with the sign of the first.
- swap replaces the input with each other.
*/

namespace math {

// COMMON OPERATIONS
// =================
// =================

template<typename T> 
inline std::string str(const T& atvalue) {
	std::stringstream ss;
	std::string s;
	ss.precision(33);
	ss << atvalue;
	return ss.str();
}
/**< transforms the input into a string */

template<> 
inline std::string str<double> (const double& v) {
	std::ostringstream oss;
	oss << std::fixed << std::setprecision(33);
	oss << v;
	return oss.str();
}
/**< particularization of str<T> for doubles */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

template<class T>
inline const T max(const T& a, const T& b) {
	return b > a ? (b) : (a);
}
/**< returns maximum of a and b */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

template<class T>
inline const T min(const T& a, const T& b) {
	return b < a ? (b) : (a);
}
/**< returns maximum of a and b */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

template<class T>
inline const T sign(const T& a, const T& b) {
	return (b >= 0) ?
		(a >= 0 ? a : - a) :
		(a >= 0 ? -a : a);
}
/**< returns a with the sign of b */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

template<class T>
inline void swap(T& a, T& b) {
	T temp = a;
	a = b;
	b = temp;
}
/**< swaps a with b */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

template<class T>
inline void shift2(T& a, T& b, const T& c) {
	a = b;
	b = c;
}
/**< a obtains the initial value of b, b that of c, and c that of d */

template<class T>
inline void shift3(T& a, T& b, T& c, const T& d) {
	a = b;
	b = c;
	c = d;
}
/**< a obtains the initial value of b, b that of c, and c that of d */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// ===== IMAGE PYRAMID SCALES =====
// ================================
/**< return the scale corresponding to a given image pyramid level --> 1 - 2 - 4 - 8 - 16 */
inline int pyramid_scale(int pyramid_level) {return (1 << pyramid_level);}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


} // closes namespace math

#endif

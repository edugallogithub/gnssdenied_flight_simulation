#ifndef MATH_HERMITE_POW_H
#define MATH_HERMITE_POW_H

#include "../math.h"
#include "../vec/vec4.h"
#include "../vec/vec2.h"


/*
This file contains the classes containing the Hermite coefficients for
various interpolation types.
The hermite_pow class contains the coefficients for a single interval
in a single direction, hence, four (4) coefficients.
The hermite_pow2 class contains the coefficients for a single interval
in a second direction, hence, sixteen (16) coefficients.
The hermite_pow3 class contains the coefficients for a single interval
in a third direction, hence, sixtyfour (64) coefficients.
The hermite_pow4 class contains the coefficients for a single interval
in a fourth direction, hence, two hundred and fiftysix (256) coefficients.

*/

namespace math {

// CLASS HERMITE_POW
// =================
// =================

class MATH_API hermite_pow {
private:
	std::vector<double> _coeffs;
	/**< vector of size 4 with the Hermite coefficients */

	hermite_pow(double, 
				double, 
				double, 
				double);
	/**< private constructor based on the four coefficients, only 
	employed by the arithmetical operators */
public:
	hermite_pow(const double& point0,
				const double& point1,
				const double& value0,
				const double& value1,
				const double& slope0,
				const double& slope1);
	/**< constructor based on the two consecutive input values, their
	corresponding outpus, and the slopes at both points */
	hermite_pow();
	/**< empty constructor that sets the four coefficients to 0 */

	hermite_pow(const hermite_pow&);
	/**< copy constructor */
	hermite_pow& operator=(const hermite_pow& op2);
	/**< overloaded operator = (assignment) */
	~hermite_pow();
	/**< destructor */

	const double& operator[](unsigned short index) const {return _coeffs[index];}
	/**< overloaded operator [] to read */

	const hermite_pow operator+(const hermite_pow& op2) const;
	const hermite_pow operator-(const hermite_pow& op2) const;
	const hermite_pow operator*(const hermite_pow& op2) const;
	const hermite_pow operator/(const hermite_pow& op2) const;
	const hermite_pow operator*(const double& op2) const;
	const hermite_pow operator/(const double& op2) const;
	/**< arithmetical operators */
}; // closes class hermite_pow

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE_POW2
// ==================
// ==================

class MATH_API hermite_pow2 {
private:
	std::vector<hermite_pow> _coeffs;
	/**< vector of size 4 of size 4 Hermite coefficients (total 16 coefficients) */

	hermite_pow2(const hermite_pow&, 
				 const hermite_pow&, 
				 const hermite_pow&,
				 const hermite_pow&);
	/**< private constructor based on the four coefficients, only 
	employed by the arithmetical operators */
public:
	hermite_pow2(const double& point0,
				 const double& point1,
				 const math::hermite_pow& herm0,
				 const math::hermite_pow& herm1,
				 const math::hermite_pow& slope0,
				 const math::hermite_pow& slope1);
	/**< constructor based on the two consecutive input values, their corres
	ponding outputs (Hermite coefficients), and the slopes at both
	points (also Hermite coefficients) */
	hermite_pow2();
	/**< empty constructor that sets the 16 coefficients to 0 */
	hermite_pow2(const hermite_pow2&);
	/**< copy constructor */
	hermite_pow2& operator=(const hermite_pow2& op2);
	/**< overloaded operator = (assignment) */
	~hermite_pow2();
	/**< destructor */

	const hermite_pow& operator[](unsigned short index) const {return _coeffs[index];}
	/**< overloaded operator [] to read */

	const hermite_pow2 operator+(const hermite_pow2& op2) const;
	const hermite_pow2 operator-(const hermite_pow2& op2) const;
	const hermite_pow2 operator*(const hermite_pow2& op2) const;
	const hermite_pow2 operator/(const hermite_pow2& op2) const;
	const hermite_pow2 operator*(const double& op2) const;
	const hermite_pow2 operator/(const double& op2) const;
	/**< arithmetical operators */
}; // closes class hermite_pow2

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE_POW3
// ==================
// ==================

class MATH_API hermite_pow3 {
private:
	std::vector<hermite_pow2> _coeffs;
	/**< vector of size 4 vectors of size 4 of size 4 Hermite coefficients (total 64 coefficients) */

	hermite_pow3(const hermite_pow2&,
				 const hermite_pow2&,
				 const hermite_pow2&,
				 const hermite_pow2&);
	/**< private constructor based on the four coefficients, only 
	employed by the arithmetical operators */
public:
	hermite_pow3(const double& point0,
				 const double& point1,
				 const math::hermite_pow2& herm0,
				 const math::hermite_pow2& herm1,
				 const math::hermite_pow2& slope0,
				 const math::hermite_pow2& slope1);
	/**< constructor based on the two consecutive input values, their corres
	ponding outputs (Hermite coefficients), and the slopes at both
	points (also Hermite coefficients) */
	hermite_pow3();
	/**< empty constructor that sets the 64 coefficients to 0 */
	hermite_pow3(const hermite_pow3&);
	/**< copy constructor */
	hermite_pow3& operator=(const hermite_pow3& op2);
	/**< overloaded operator = (assignment) */
	~hermite_pow3();
	/**< destructor */

	const hermite_pow2& operator[](unsigned short index) const {return _coeffs[index];}
	/**< overloaded operator [] to read */

	const hermite_pow3 operator+(const hermite_pow3& op2) const;
	const hermite_pow3 operator-(const hermite_pow3& op2) const;
	const hermite_pow3 operator*(const hermite_pow3& op2) const;
	const hermite_pow3 operator/(const hermite_pow3& op2) const;
	const hermite_pow3 operator*(const double& op2) const;
	const hermite_pow3 operator/(const double& op2) const;
	/**< arithmetical operators */
}; // closes class hermite_pow3

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE_POW4
// ==================
// ==================

class MATH_API hermite_pow4 {
private:
	std::vector<hermite_pow3> _coeffs;
	/**< vector of size 4 vectors of size 4 vectors of size 4 Hermite coefficients
	(total 256 coefficients) */

	hermite_pow4(const hermite_pow3&,
				 const hermite_pow3&,
				 const hermite_pow3&,
				 const hermite_pow3&);
	/**< private constructor based on the four coefficients, only 
	employed by the arithmetical operators */
public:
	hermite_pow4(const double& point0,
				 const double& point1,
				 const math::hermite_pow3& herm0,
				 const math::hermite_pow3& herm1,
				 const math::hermite_pow3& slope0,
				 const math::hermite_pow3& slope1);
	/**< constructor based on the two consecutive input values, their corres
	ponding outputs (Hermite coefficients), and the slopes at both
	points (also Hermite coefficients) */
	hermite_pow4();
	/**< empty constructor that sets the 256 coefficients to 0 */
	hermite_pow4(const hermite_pow4&);
	/**< copy constructor */
	hermite_pow4& operator=(const hermite_pow4& op2);
	/**< overloaded operator = (assignment) */
	~hermite_pow4();
	/**< destructor */

	const hermite_pow3& operator[](unsigned short index) const {return _coeffs[index];}
	/**< overloaded operator [] to read */

	const hermite_pow4 operator+(const hermite_pow4& op2) const;
	const hermite_pow4 operator-(const hermite_pow4& op2) const;
	const hermite_pow4 operator*(const hermite_pow4& op2) const;
	const hermite_pow4 operator/(const hermite_pow4& op2) const;
	const hermite_pow4 operator*(const double& op2) const;
	const hermite_pow4 operator/(const double& op2) const;
	/**< arithmetical operators */
}; // closes class hermite_pow4

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

} // closes namespace math

#endif









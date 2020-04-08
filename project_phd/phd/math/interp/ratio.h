#ifndef MATH_RATIO_H
#define MATH_RATIO_H

#include "../math.h"

/*
This file contains the ratio base class and its derivate classes for linear,
quadratic, cubic, and spline interpolation. They are just containers indicating
the relative position between the input magnitude and two of the inputs on which
the interpolation is based. 

Given a vector of input magnitudes Omag1, Omag2, Omag3, Omag4, and another input
Omag where the interpolation is to be performed, the ratio respons to the following
formula:
			   Omag.diff(Omagx)
	ratioxy = ------------------
               Omag2.diff(Omag1)			

Different interpolation schemes require a different number of ratios, and these 
derives in the different derivate classes of this file. The ratios are filled up
by the interpolation classes.

LEFT:
N/A

SLOW FUNCTIONS (require loops):
N/A

ACCURACY:
No function presents any degradation in accuracy.

MODE OF USE:
The ratio classes are not intended to be directly employed by the user, but managed
by the interpolation (interp) derivated classes.
*/

namespace math {

// CLASS RATIO
// ===========
// ===========

class MATH_API ratio {
protected:
	ratio(){}
	/**< empty constructor */
public:
	virtual void reset() {}
	/**< required by the ratio_mgr to return object to initial state, although
	in this case it is a dummy to save time. The ratio_mgr retursn object with
	their previous state. */

}; // closes class ratio

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_LINEAR
// ==================
// ==================

class MATH_API ratio_linear: public ratio {
public:
	ratio_linear();
	/**< empty constructor */
	ratio_linear(const ratio_linear&);
	/**< copy constructor */
	ratio_linear& operator=(const ratio_linear& op2);
	/**< overloaded operator = (assignment) */
	~ratio_linear(){}
	/**< destructor */

	double _ratio12;
	/**< ratio (relative position between the 1st and 2nd points) to be used for
	the	interpolation. 0 means first point, 1 means second. Outside that
	range means extrapolation */
	void set_ratio12(const double& ratio12) {_ratio12 = ratio12;}
	/**< sets the ratio attribute value */
}; // closes class ratio_linear

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_QUADRATIC
// =====================
// =====================

class MATH_API ratio_quadratic: public ratio {
public:
	ratio_quadratic();
	/**< empty constructor */
	ratio_quadratic(const ratio_quadratic&);
	/**< copy constructor */
	ratio_quadratic& operator=(const ratio_quadratic& op2);
	/**< overloaded operator = (assignment) */
	~ratio_quadratic(){}
	/**< destructor */

	double _ratio12;
	/**< ratio (relative position between the 1st and 2nd points) to be used for
	the	interpolation. 0 means first point, 1 means second. Outside that
	range means extrapolation */
	double _ratio23;
	/**< ratio (relative position between the 2nd and 3rd points) to be used for
	the	interpolation. */
	double _ratio13;
	/**< ratio (relative position between the 1st and 3rd points) to be used for
	the	interpolation. */

	void set_ratio12(const double& ratio12) {_ratio12 = ratio12;}
	void set_ratio23(const double& ratio23) {_ratio23 = ratio23;}
	void set_ratio13(const double& ratio13) {_ratio13 = ratio13;}
	/**< sets the ratio attribute value */
}; // closes class ratio_quadratic

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_CUBIC
// =================
// =================

class MATH_API ratio_cubic: public ratio {
public:
	ratio_cubic();
	/**< empty constructor */
	ratio_cubic(const ratio_cubic&);
	/**< copy constructor */
	ratio_cubic& operator=(const ratio_cubic& op2);
	/**< overloaded operator = (assignment) */
	~ratio_cubic(){}
	/**< destructor */

	double _ratio12;
	/**< ratio (relative position between the 1st and 2nd points) to be used for
	the	interpolation. 0 means first point, 1 means second. Outside that
	range means extrapolation */
	double _ratio23;
	/**< ratio (relative position between the 2nd and 3rd points) to be used for
	the	interpolation. */
	double _ratio34;
	/**< ratio (relative position between the 3rd and 4th points) to be used for
	the	interpolation. */
	double _ratio13;
	/**< ratio (relative position between the 1st and 3rd points) to be used for
	the	interpolation. */
	double _ratio24;
	/**< ratio (relative position between the 2nd and 4th points) to be used for
	the	interpolation. */
	double _ratio14;
	/**< ratio (relative position between the 1st and 4th points) to be used for
	the	interpolation. */

	void set_ratio12(const double& ratio12) {_ratio12 = ratio12;}
	void set_ratio23(const double& ratio23) {_ratio23 = ratio23;}
	void set_ratio34(const double& ratio34) {_ratio34 = ratio34;}
	void set_ratio13(const double& ratio13) {_ratio13 = ratio13;}
	void set_ratio24(const double& ratio24) {_ratio24 = ratio24;}
	void set_ratio14(const double& ratio14) {_ratio14 = ratio14;}
	/**< sets the ratio attribute value */	
}; // closes class ratio_cubic

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_HERMITE
// ===================
// ===================

class MATH_API ratio_hermite: public ratio {
public:
	ratio_hermite();
	/**< empty constructor */
	ratio_hermite(const ratio_hermite&);
	/**< copy constructor */
	ratio_hermite& operator=(const ratio_hermite& op2);
	/**< overloaded operator = (assignment) */
	~ratio_hermite(){}
	/**< destructor */

	double _ratio12;
	/**< ratio (relative position between the 1st and 2nd points) to be used for
	the	interpolation. 0 means first point, 1 means second. Outside that
	range means extrapolation */
	void set_ratio12(const double& ratio12) {_ratio12 = ratio12;}
	/**< sets the ratio attribute value */
}; // closes class ratio_hermite

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS RATIO_SPLINE
// ==================
// ==================

class MATH_API ratio_spline: public ratio {
public:
	ratio_spline();
	/**< empty constructor */
	ratio_spline(const ratio_spline&);
	/**< copy constructor */
	ratio_spline& operator=(const ratio_spline& op2);
	/**< overloaded operator = (assignment) */
	~ratio_spline(){}
	/**< destructor */

	double _ratio12;
	/**< ratio (relative position between the 1st and 2nd points) to be used for
	the	interpolation. 0 means first point, 1 means second. Outside that
	range means extrapolation */
	void set_ratio12(const double& ratio12) {_ratio12 = ratio12;}
	/**< sets the ratio attribute value */
}; // closes class ratio_spline

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

} // closes namespace math

#endif









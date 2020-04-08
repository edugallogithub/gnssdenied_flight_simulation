#ifndef MATH_FUNC_H
#define MATH_FUNC_H

#include "../math.h"
#include "../templates/math_.h"
#include "../logic/constant.h"
#include <vector>
#include <cmath>
#include <algorithm>

/*
This file contains the func class (functional), which contains numeric methods. Most methods here are taken from the book "Numerical Recipes in C++
2nd edition" by Cambridge University Press. The individual methods state the page where the method is described in the book.
*/

namespace math {

// FUNCTIONALS
// ===========
// ===========

class MATH_API func {
public:
    /**< definition of function over which the different numeric methods work. Returns a double based on an input parameter (over which numeric methods
    generally try to find an optimum value) and a set of other inputs grouped in a vector */
	virtual double exec	(const double&, const std::vector<double>&) = 0;

	/**< ===== ===== Secant Method for Finding a Zero ===== ===== */
	/**< ======================================================== */
    /**< based on the derivate class exec method that has as input (x, par), a desired result res (usually 0), any two initial values x1 and x2, a tolerance tol,
    and a maximum number of iterations, returns a value of x such that fabs(exec(x, par) - res) < tol. TOLERANCE IN OUTPUT. Secant method employed. Page 361. */
	virtual double find_zero_secant(const std::vector<double>& par, const double& res, double x1, double x2,
									const double& tol, // = 1e -10,
									const unsigned short& iter = constant::NUM_ITER());


	/**< ===== ===== Brent Method for Finding a Zero ===== ===== */
	/**< ======================================================= */
    /**< based on the derivate class exec method that has as input (x, par), a desired result res (usually 0), any two initial values x1 and x2, a tolerance tol,
    and a maximum number of iterations,  returns a value of x such that "fabs(Deltax)<= [2 * EPS * fabs(x) + 0.5 * tol]". TOLERANCE IN INPUT. Van Wijngaarden -
     Dekker - Brent method employed. Page 365. */
	virtual double find_zero_brent(const std::vector<double>& par, const double& res, const double& x1, const double& x2,
								   const double& tol, // = 1e -10,
								   const unsigned short& iter = constant::NUM_ITER());
    /**< based on the derivate class exec method that has as input (x, par), a desired result res (usually 0), any two initial values x1 and x2, the rate at
    which the interval size grows, and a maximum number of iterations, searches for values of x1 and x2 between which (exec(x, par) - res) has a zero (change
    of sign), modifying x1 and x2. If exec(x1) is closer to res than exec(x2), x1 expands away from x2 by an amount fact(x2-x1), and viceversa, until x1 and x2
    bracket the solution res. METHOD SO FAR NOT EMPLOYED. */
	virtual void find_interval(const std::vector<double>& par, const double& res, double& x1, double& x2,
							   const double& fact = 1.6,
							   const unsigned short& iter = constant::NUM_ITER());
    /**< based on the derivate class exec method that has as input (x, par), a desired result res (usually 0), any two initial values x1 < x2,  a maximum valid
    value for x (func causes problems above that), the rate at which the interval size grows, and a maximum number of iterations, searches for values of x1
    and x2 between which (exec (x, par) - res) has a zero (change of sign), ensuring that x1 < x2 and that x1 is bigger than the input x1), modifying x1 and x2.
    Works by continously trying new intervals in which x1 equals the previous x2 and x2 grows by the product of fact by the difference between the previous
    (x2-x1). Should be executed inmediately before find_zero_brent. */
	virtual void find_interval_pos(const std::vector<double>& par, const double& res, double& x1, double& x2, const double& xmax,
								   const double& fact = 1.6, const unsigned short& iter = constant::NUM_ITER());
    /**< based on the derivate class exec method that has as input (x, par), a desired result res (usually 0), any two initial values x1 < x2, a minimum valid
    value for x (func causes problems below that), the rate at which the interval size grows, and a maximum number of iterations, searches for values of x1
    and x2 between which (exec(x, par) - res) has a zero (change of sign), ensuring that x1 < x2 and that x2 is smaller than the input x2), modifying x1 and x2.
    Works by continously trying new intervals in which x2 equals the previous x1 and x1 diminishes by the product of fact by the difference between the previous
    (x2-x1). Should be executed inmediately before find_zero_brent.  */
	virtual void find_pos_interval(const std::vector<double>& par, const double& res, double& x1, double& x2, const double& xmin,
								   const double& fact = 1.6, const unsigned short& iter = constant::NUM_ITER());

	/**< ===== ===== Golden Method for Finding a Minimum ===== ===== */
	/**< =========================================================== */
    /**< based on the derivate class exec method that has as input (x, par), any three initial values bracketing a minimum provided by the find_bracket_pos
     * method, in returns the value at which exec(x, par) has a minimum, using the bisection or golden search method. The bracket size is approximately
     * 2*tol (+-tol), and tol should not be set lower than the square root of the maching precision (default is for doubles). Page 406. */
	virtual double find_minimum_golden(const std::vector<double>& par, const double& x1, const double& x2, const double& x3,
									   const double& tol = 3e-8, const unsigned short& iter = constant::NUM_ITER());
    /**< based on the derivate class exec method that has as input (x, par), any two initial values (x3 input value is meaningless), a minimum value for x
     * (func causes problems below that), a maximum value for x (func causes problems above that), the rate at which the interval size grows, and a maximum
     * number	of iterations, searches for values of (x1, x2, x3) that bracket a minimum (such that exec(x2,par) < exec(x1,par) and exec(x2,par) < exec(x3,par)),
     * modifying x1 and x2 and filling up x3. Should be executed inmediately before find_minimum_golden. Page 404. */
	virtual void find_bracket(const std::vector<double>& par, double& x1, double& x2, double& x3, const double& xmin, const double& xmax,
							  const double& fact = 1.6, const unsigned short& iter = constant::NUM_ITER());

}; // closes class func
} // closes namespace math

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#endif

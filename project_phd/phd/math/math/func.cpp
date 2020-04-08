#include "func.h"

// FUNCTIONALS
// ===========
// ===========

/* ===== ===== Secant Method for Finding a Zero ===== ===== */
/* ======================================================== */
double math::func::find_zero_secant(const std::vector<double>& par,
								  const double& res, 
								  double x0,
								  double x1,
								  const double& tol,
								  const unsigned short& iter) {
	double f0, f1, f2, x2;
	try {f0	= exec(x0, par);}
	catch (...) {
		throw std::runtime_error("Function evaluation error.");
	}
	try {f1	= exec(x1, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}

	int c = 0;
	while (fabs(f1-res) > tol) {
		if (++c == iter) {
			throw std::runtime_error("Maximum number of iterations reached.");
		}
		if (f1 == f0) {throw std::runtime_error("Unable to find a zero.");}
		x2	= x0 - (f0 - res) * (x1 - x0) / (f1 - f0);
		try {f2	= exec(x2, par);}
        catch (...) {
            throw std::runtime_error("Function evaluation error.");
		}
		x0	= x1;
		f0	= f1;
		x1	= x2;
		f1	= f2;
	}
	return x1;
}
/* based on the derivate class exec method that has as input (x, par), a
desired result res (usually 0), any two initial values x1 and x2, a tolerance tol,
and a maximum number of iterations, returns a value of x such that
fabs(exec(x, par) - res) < tol. TOLERANCE IN OUTPUT. Secant method
employed. Page 361. */

/* ===== ===== Brent Method for Finding a Zero ===== ===== */
/* ======================================================= */

double math::func::find_zero_brent(const std::vector<double>& par,
								 const double& res, 
								 const double& x1,
								 const double& x2,
								 const double& tol,
								 const unsigned short& iter) {
	double EPS = math::constant::EPS();
	double a = x1, fa;
	try {fa = exec(a, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}
	double b = x2, fb;
	try {fb = exec(b, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}

	double c = x2, fc = fb;
	double d, e, min1, min2, p, q, r, s, tol1, xm;

	if (((fa - res) > 0.0 && (fb - res) > 0.0) ||
		((fa - res) < 0.0 && (fb - res) < 0.0)) {
        throw std::runtime_error("Problem with Brent bracket.");
	} // solution not in bracket

	for (unsigned short i = 0; i < iter; ++i) {
		if (((fb - res) > 0.0 && (fc - res) > 0.0) ||
			((fb - res) < 0.0 && (fc - res) < 0.0)) {
			// if b and c are on same side
			// rename a, b, c, and adjust bounding interval d
			c	= a;
			fc	= fa;
			e	= d	= b - a;
		}
		if (fabs(fc - res) < fabs(fb - res)) {
			// if c closer than b
			// turn b into a and c, c into b 
			a	= b;
			b	= c;
			c	= a;
			fa	= fb;
			fb	= fc;
			fc	= fa;
		}

		tol1	= 2.0 * EPS * fabs(b) + 0.5 * tol; // convergence check
		xm		= 0.5 * (c - b);
		if ((fabs(xm) <= tol1)	|| ((fb - res) == 0.0)) {return b;} 
		if ((fabs(e) >= tol1)	&& (fabs(fa - res) > fabs(fb - res))) {
			// attemp inverse quadratic interpolation
			s = (fb - res) / (fa - res);
			if (a == c) {
				p = 2.0 * xm * s;
				q = 1.0 - s;
			}
			else {
				q = (fa - res) / (fc - res);
				r = (fb - res) / (fc - res);
				p = s * (2.0 * xm * q * (q - r) - (b - a) * (r - 1.0));
				q = (q - 1.0) * (r - 1.0) * (s - 1.0);
			}
			if (p > 0.0) {q = -q;} // check whether in bounds
			p		= fabs(p);
			min1	= 3.0 * xm * q - fabs(tol1 * q);
			min2	= fabs(e * q);
			if (2.0 * p < (min1 < min2 ? min1 : min2)) {	// accept interpolation
				e = d;
				d = p / q;
			}
			else { // interpolation failed, use bisection
				d = xm;
				e = d;
			}
		}
		else { // bonds decreasing too slowly, use bisection
			d = xm;
			e = d;
		}
		// move last best guess to a
		a	= b;
		fa	= fb;
		if (fabs(d) > tol1)	{b += d;}
		else				{b += math::sign(tol1, xm);}
		try {fb = exec(b, par);}
        catch (...) {
            throw std::runtime_error("Function evaluation error.");
		}
	}
	// maximum number of iterations exceeded
    throw std::runtime_error("Maximum number of iterations reached.");
	return 0.0; // never gets here
}
/* based on the derivate class exec method that has as input (x, par), a
desired result res (usually 0), any two initial values x1 and x2, a tolerance tol,
and a maximum number of iterations,  returns a value of x such that "fabs(Deltax)
<= [2 * EPS * fabs(x) + 0.5 * tol]". TOLERANCE IN INPUT. Van Wijngaarden - Dekker
- Brent method employed. Page 365.*/

void math::func::find_interval(const std::vector<double>& par,
							 const double& res, 
							 double& x1,
							 double& x2,
							 const double& fact,
							 const unsigned short& iter) {
	if (x1 == x2) { // initial positions shall be different
        throw std::runtime_error("Wrong interval.");
	}
	double f1, f2;
	try {f1 = exec(x1, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}
	try {f2 = exec(x2, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}
	for (unsigned short i = 0; i != iter; ++i) {
		if (((f1 - res) * (f2 - res)) < 0.0) {return;}
		if (fabs(f1 - res) < fabs(f2 - res)) {
			try {f1	= exec(x1 += fact * (x1 - x2), par);}
            catch (...) {
                throw std::runtime_error("Function evaluation error.");
			}
		}
		else {
			try {f2	= exec(x2 += fact * (x2 - x1), par);}
            catch (...) {
                throw std::runtime_error("Function evaluation error.");
			}
		}
	}
	// maximum number of iterations exceeded
    throw std::runtime_error("Maximum number of iterations reached.");
}
/* based on the derivate class exec method that has as input (x, par), a
desired result res (usually 0), any two initial values x1 and x2, the rate at
which the interval size grows, and a maximum number of iterations, searches for
values of x1 and x2 between which (exec(x, par) - res) has a zero (change
of sign), modifying x1 and x2. If exec(x1) is closer to res than exec(x2), x1
expands away from x2 by an amount fact(x2-x1), and viceversa, until x1 and x2
bracket the solution res. METHOD SO FAR NOT EMPLOYED. */

void math::func::find_interval_pos(const std::vector<double>& par,
								 const double& res, 
								 double& x1,
								 double& x2,
								 const double& xmax,
								 const double& fact,
								 const unsigned short& iter) {
	if (x1 >= x2) { // initial positions shall be in order
        throw std::runtime_error("Wrong interval.");
	}
	if (x2 > xmax) { // x2 shall not be bigger than maximum
        throw std::runtime_error("Wrong interval.");
	} 

	double f1, f2, x, f;
	try {f1 = exec(x1, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}
	try {f2 = exec(x2, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}

	for (unsigned short i = 0; i != iter; ++i) {
		if (((f1 - res) * (f2 - res)) < 0.0) {return;}
		x	= x2 + fact * (x2 - x1);
		if (x >= xmax) {
			try {f = exec(xmax, par);}
            catch (...) {
                throw std::runtime_error("Function evaluation error.");
			}
			x1	= x2;
			f1	= f2;
			x2  = xmax;
			f2	= f;
			if (((f1 - res) * (f2 - res)) < 0.0) {return;} 
			else {
                throw std::runtime_error("Wrong interval.");
			}
		}
		try {f = exec(x, par);}
        catch (...) {
            throw std::runtime_error("Function evaluation error.");
		}
		x1	= x2;
		f1	= f2;
		x2	= x;
		f2	= f;
	}
	// maximum number of iterations exceeded
    throw std::runtime_error("Maximum number of iterations reached.");
}
/* based on the derivate class exec method that has as input (x, par), a
desired result res (usually 0), any two initial values x1 < x2,  a maximum valid
value for x (func causes problems above that), the rate at which the interval size
grows, and a maximum number of iterations, searches for values of x1 and x2 between
which (exec (x, par) - res) has a zero (change of sign), ensuring that x1 < x2
and that x1 is bigger than the input x1), modifying x1 and x2. Works by
continously trying new intervals in which x1 equals the previous x2 and x2 grows
by the product of fact by the difference between the previous (x2-x1). Should be
executed inmediately before find_zero_brent. */

void math::func::find_pos_interval(const std::vector<double>& par,
								 const double& res,
								 double& x1,
								 double& x2,
								 const double& xmin,
								 const double& fact,
								 const unsigned short& iter) {
	if (x1 >= x2) { // initial positions shall be in order
        throw std::runtime_error("Wrong interval.");
	}
	if (x1 < xmin) { // x1 shall not be smaller than minimum
        throw std::runtime_error("Wrong interval.");
	} 

	double f1, f2, x, f;
	try {f1 = exec(x1, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}
	try {f2 = exec(x2, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}

	for (unsigned short i = 0; i != iter; ++i) {
		if (((f1 - res) * (f2 - res)) < 0.0) {return;}
		x = x1 + fact * (x1 - x2); // x	= x2 + fact * (x2 - x1);
		if (x <= xmin) {
			try {f = exec(xmin, par);}
            catch (...) {
                throw std::runtime_error("Function evaluation error.");
			}
			x2	= x1;
			f2	= f1;
			x1  = xmin;
			f1	= f;
			if (((f1 - res) * (f2 - res)) < 0.0) {return;} 
			else {
                throw std::runtime_error("Wrong interval.");
			}
		}
		try {f = exec(x, par);}
        catch (...) {
            throw std::runtime_error("Function evaluation error.");
		}
		x2	= x1;
		f2	= f1;
		x1	= x;
		f1	= f;
	}
	// maximum number of iterations exceeded
    throw std::runtime_error("Maximum number of iterations reached.");
}
/* based on the derivate class exec method that has as input (x, par), a 
desired result res (usually 0), any two initial values x1 < x2, a minimum valid
value for x (func causes problems below that), the rate at which the interval size
grows, and a maximum number of iterations, searches for values of x1 and x2 between
which (exec(x, par) - res) has a zero (change of sign), ensuring that x1 < x2 and
that x2 is smaller than the input x2), modifying x1 and x2. Works by
continously trying new intervals in which x2 equals the previous x1 and x1 diminishes
by the product of fact by the difference between the previous (x2-x1). Should be
executed inmediately before find_zero_brent. */

/* ===== ===== Golden Method for Finding a Minimum ===== ===== */
/* =========================================================== */
double math::func::find_minimum_golden(const std::vector<double>& par,
									 const double& x1,
									 const double& x2,
									 const double& x3,
									 const double& tol, 
									 const unsigned short& iter) {
	const double R = 0.61803399; // see book
	const double C = 1.0 - R;
	double fB, fC, xA, xB, xC, xD, x, f;

	// continuously keep track of 4 points: xA, xB, xC, xD
	xA = x1;
	xD = x3;
	if (fabs(x3-x2) > fabs(x2-x1)) { // xA to xB is smaller segment
		xB = x2;
		xC = x2 + C * (x3-x2);
	}
	else {
		xB = x2 - C * (x2-x1);
		xC = x2;
	}
	try {fB = exec(xB, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}
	try {fC = exec(xC, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}

	for (unsigned short i = 0; i != iter; ++i) {
		if (fabs(xD-xA) < tol*(fabs(xB)+fabs(xC))) {
			if (fB < fC) {return xB;}
			else         {return xC;}
		}
		if (fC < fB) {
			x = R*xC + C*xD;
			try {f = exec(x, par);}
            catch (...) {
                throw std::runtime_error("Function evaluation error.");
			}
            math::shift3(xA, xB, xC, x);
            math::shift2(fB, fC, f);
		}
		else {
			x = R*xB + C*xA;
			try {f = exec(x, par);}
            catch (...) {
                throw std::runtime_error("Function evaluation error.");
			}
            math::shift3(xD, xC, xB, x);
            math::shift2(fC, fB, f);
		}
	}
	// maximum number of iterations exceeded
    throw std::runtime_error("Maximum number of iterations reached.");
}
/* based on the derivate class exec method that has as input (x, par), any three
initial values bracketing a minimum provided by the find_bracket_pos method, in returns
the value at which exec(x, par) has a minimum, using the bisection or golden search
method. The bracket size is approximately 2*tol (+-tol), and tol should not be set
lower than the square root of the maching precision (default is for doubles).
Page 406. */

void math::func::find_bracket(const std::vector<double>& par,
							double& x1,
							double& x2,
							double& x3,
							const double& xmin,
							const double& xmax,
							const double& fact,
							const unsigned short& iter) {
	if (x1 == x2) { // initial positions shall be different
        throw std::runtime_error("Wrong interval.");
	}
	double f1, f2, f3;
	try {f1 = exec(x1, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}
	try {f2 = exec(x2, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}

	if (f2 > f1) { // so algorithm moves always to the right 
        math::swap(x1, x2);
        math::swap(f1, f2);
	}

	x3 = x2 + fact * (x2 - x1);
	if (x3>= xmax) {x3 = xmax;}
	try {f3 = exec(x3, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}

	if (x3<= xmin) {x3 = xmin;}
	try {f3 = exec(x3, par);}
    catch (...) {
        throw std::runtime_error("Function evaluation error.");
	}

	double x, f;
	for (unsigned short i = 0; i != iter; ++i) {
		if (f2 < f3) {return;}
		x = x3 + fact * (x3 - x2);
		if (x >= xmax) {
			try {f = exec(xmax, par);}
            catch (...) {
                throw std::runtime_error("Function evaluation error.");
			}
            math::shift3(x1, x2, x3, xmax);
            math::shift3(f1, f2, f3, f);
			if (f2 < f3) {return;}
			else {
                throw std::runtime_error("Wrong interval.");
			}
		}
		if (x <= xmin) {
			try {f = exec(xmin, par);}
            catch (...) {
                throw std::runtime_error("Function evaluation error.");
			}
            math::shift3(x1, x2, x3, xmin);
            math::shift3(f1, f2, f3, f);
			if (f2 < f3) {return;}
			else {
                throw std::runtime_error("Wrong interval.");
			}
		}
		try {f = exec(x, par);}
        catch (...) {
            throw std::runtime_error("Function evaluation error.");
		}
        math::shift3(x1, x2, x3, x); // eliminate oldest point and continue
        math::shift3(f1, f2, f3, f);
	}
	// maximum number of iterations exceeded
    throw std::runtime_error("Maximum number of iterations reached.");
}
/* based on the derivate class exec method that has as input (x, par), any two
initial values (x3 input value is meaningless), a minimum value for x (func causes
problems below that), a maximum value for x (func causes problems above that), the
rate at which the interval size grows, and a maximum number	of iterations, searches
for values of (x1, x2, x3) that bracket a minimum (such that exec(x2,par) < 
exec(x1,par) and exec(x2,par) < exec(x3,par)), modifying x1 and x2 and filling up x3.
Should be executed inmediately before find_minimum_golden. Page 404. */





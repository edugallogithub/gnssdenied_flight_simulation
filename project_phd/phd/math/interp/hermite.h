#ifndef MATH_HERMITE1V_H
#define MATH_HERMITE1V_H

#include "../math.h"
#include "../vec/vec3.h"
#include "hermite_pow.h"

/*
This file contains the hermite1X classes (X 1 through 4), which are in 
charge of computing the Hermite coefficients (held in classes hermite_powX)
for interpolations in one, two, three, and four dimensions.

They are based on the input points, the output values, and the differentials
at those points. The Hermite cubic interpolation is containuous both on
the values and the differentials. Two different types of differentials are
suported: those computed as averages of Lagrange 1st order interpolation,
and those computed by Lagrange 2nd order interpolation.
*/

namespace math {

// CLASS HERMITE1V
// ===============
// ===============

class MATH_API hermite1v {
private:
	std::vector<math::hermite_pow*> _coeffs;
	/**< vector of size n-1 with the Hermite coefficients for each interval */
public:
	hermite1v(const math::vec1& points,
			  const math::vec& values, // this is a vec1
			  const unsigned short& values_begin,
			  const math::vec& slopes, // this is a coefficientv1
			  const unsigned short& slopes_begin,
			  math::logic::INTERP_MODE interp_mode);
	/**< constructor based on a vector of inputs, a vector of outputs,
	and a vector of slopes. The output vector may be of equal or greater size
	than the input, with the values_begin input identifying the position within
	the output vector that corresponds to the first position in the input. 
	The slopes vector may also be of equal or greater size than the input, with
	the slopes_begin input identifying the position within the slopes vector
	that corresponds to the first position in the input. Employed for one dimensional 
	interpolation. */
	hermite1v(const math::vec1& points);
	/**< constructor based on a vector of inputs. Creates _coeffs atribute
	of the proper size but filled with zeros */
	hermite1v();
	/**< empty constructor for when only a dummy is needed */
	hermite1v(const hermite1v&);
	/**< copy constructor */
	hermite1v& operator=(const hermite1v& op2);
	/**< overloaded operator = (assignment) */
	~hermite1v();
	/**< destructor */
    
    unsigned short size() const {return _coeffs.size();}
    /**< returns _coeffs size */
    
	const math::hermite_pow& back() {return *(_coeffs.back());}
	/**< returns reference to last component of _coeffs */

	math::hermite_pow& operator[](unsigned short index) {return *_coeffs[index];}
	const math::hermite_pow& operator[](unsigned short index) const {return *_coeffs[index];}
	/**< overloaded operator [] to write or read */
}; // closes class hermite1v

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE2V
// ===============
// ===============

class MATH_API hermite2v {
private:
	std::vector<std::vector<math::hermite_pow2*> > _coeffs;
	/**< vector of vector to Hermite coefficients (first in second input
	direction and second in first input direction) (size (n-1) x (m-1)) */
public:
	hermite2v(const math::vec1& points1,
			  const math::vec1& points2,
			  const math::vec& Values, // this is a vec2
			  const unsigned short& Values_begin,
			  const math::vec& slopes_d1, // this vec2 is not needed ////////////////////////////
			  const unsigned short& slopes_d1_begin, // not needed /////////////////////////////
			  const math::vec& slopes_d2, // this is a coefficientv2
			  const unsigned short& slopes_d2_begin,
			  math::logic::INTERP_MODE interp_mode);
	/**< constructor based on two vectors of inputs (sizes siz1 and siz2),
	a 2D matrix of outputs (siz1 x siz2), and two 2D matrixes of slopes
	(siz2 x siz1 and siz1 x siz2). As the matrixes are presented as vecs,
	there are also indexes that stablish the position that corresponds
	to the start of the two input vectors. The vecs are big enough
	to hold siz1 x siz2 magnitudes. */
	hermite2v(const math::vec1& points1,
			  const math::vec1& points2);
	/**< constructor based on a two vectors of inputs. Creates _coeffs atribute
	of the proper size but filled with zeros */
	hermite2v();
	/**< empty constructor for when only a dummy is needed */
	hermite2v(const hermite2v&);
	/**< copy constructor */
	hermite2v& operator=(const hermite2v& op2);
	/**< overloaded operator = (assignment) */
	~hermite2v();
	/**< destructor */

	unsigned short size() const {return _coeffs.size();}
	/**< returns _coeffs size */
	std::vector<math::hermite_pow2*>& operator[](unsigned short index) {return _coeffs[index];}
	const std::vector<math::hermite_pow2*>& operator[](unsigned short index) const {return _coeffs[index];}
	/**< overloaded operator [] to read */
}; // closes class hermite2v

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE3V
// ===============
// ===============

class MATH_API hermite3v {
private:
	std::vector<std::vector<std::vector<math::hermite_pow3*> > > _coeffs;
	/**< vector of vector of vector to Hermite coefficients (first in third input
	direction, then second, then first) (size (l-1) x (n-1) x (m-1)) */
public:
	hermite3v(const math::vec1& points1,
			  const math::vec1& points2,
			  const math::vec1& points3,
			  const math::vec3& VValues,
			  const math::vec3& slopes_d1, // not needed ////////////////////////////
			  const math::vec3& slopes_d2, // not needed ////////////////////////////
			  const math::vec3& slopes_d3,
			  math::logic::INTERP_MODE interp_mode);
	hermite3v(const math::vec1& points1,
			  const math::vec1& points2,
			  const math::vec1& points3,
			  const math::vec& VValues, // this is a vec3
			  const unsigned short& VValues_begin,
			  const math::vec& slopes_d1, // this vec3 is not needed ////////////////////////////
			  const unsigned short& slopes_d1_begin, // not needed /////////////////////////////
			  const math::vec& slopes_d2, // this vec3 is not needed ////////////////////////////
			  const unsigned short& slopes_d2_begin, // not needed /////////////////////////////
			  const math::vec& slopes_d3, // this is a coefficientv3
			  const unsigned short& slopes_d3_begin,
			  math::logic::INTERP_MODE interp_mode);
	/**< constructor based on three vectors of inputs (sizes l, m and n), a 3Dmatrix of
	outputs (l x m x n), and three matrixes of slopes (l x m x n). */
	/**< constructor based on three vectors of inputs (sizes siz1, siz2, and siz3),
	a 3D matrix of outputs (siz1 x siz2 * siz3), and three �D matrixes of slopes
	(siz2 x siz1 and siz1 x siz2 ssssssssssssssssssssssssss). As the matrixes are presented as vecs,
	there are also indexes that stablish the position that corresponds
	to the start of the two input vectors. The vecs are big enough
	to hold siz1 x siz2 magnitudes. */



	hermite3v(const math::vec1& points1,
			  const math::vec1& points2,
			  const math::vec1& points3);
	/**< constructor based on three vectors of inputs (sizes l, m and n). Creates _coeffs
	attribute of the proper size but filled with zeros */
	hermite3v();
	/**< empty constructor for when only a dummy is needed */
	hermite3v(const hermite3v&);
	/**< copy constructor */
	hermite3v& operator=(const hermite3v& op2);
	/**< overloaded operator = (assignment) */
	~hermite3v();
	/**< destructor */

	std::vector<std::vector<math::hermite_pow3*> >& operator[](unsigned short index) {return _coeffs[index];}
	const std::vector<std::vector<math::hermite_pow3*> >& operator[](unsigned short index) const {return _coeffs[index];}
	/**< overloaded operator [] to read */
}; // closes class hermite3v

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE4V
// ===============
// ===============

class MATH_API hermite4v {
private:
	std::vector<std::vector<std::vector<std::vector<math::hermite_pow4*> > > > _coeffs;
	/**< vector of vector of vector of vector to Hermite coefficients (first in third input
	direction, then second, then first) (size (l-1) x (n-1) x (m-1) x (q-1)) */
public:
	hermite4v(const math::vec1& points1,
			  const math::vec1& points2,
			  const math::vec1& points3,
			  const math::vec1& points4,
			  const math::vec4& VVValues,
			  const math::vec4& slopes_d1, // not needed ////////////////////////////
			  const math::vec4& slopes_d2, // not needed ////////////////////////////
			  const math::vec4& slopes_d3, // not needed ///////////////////////////
			  const math::vec4& slopes_d4,
			  math::logic::INTERP_MODE interp_mode);
	/**< constructor based on four vectors of inputs (sizes l, m, n and 1), a 4Dmatrix of
	outputs (l x m x n x q), and four matrixes of slopes (l x m x n x q). */
	hermite4v(const math::vec1& points1,
			  const math::vec1& points2,
			  const math::vec1& points3,
			  const math::vec1& points4);
	/**< constructor based on four vectors of inputs (sizes l, m, n and q). Creates _coeffs
	attribute of the proper size but filled with zeros */
	hermite4v();
	/**< empty constructor for when only a dummy is needed */
	hermite4v(const hermite4v&);
	/**< copy constructor */
	hermite4v& operator=(const hermite4v& op2);
	/**< overloaded operator = (assignment) */
	~hermite4v();
	/**< destructor */

	std::vector<std::vector<std::vector<math::hermite_pow4*> > >& operator[](unsigned short index) {return _coeffs[index];}
	const std::vector<std::vector<std::vector<math::hermite_pow4*> > >& operator[](unsigned short index) const {return _coeffs[index];}
	/**< overloaded operator [] to read */
}; // closes class hermite4v

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

} // closes namespace math

#endif









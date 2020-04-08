#ifndef MATH_POS_FINDER_H
#define MATH_POS_FINDER_H

#include "../math.h"
#include "../vec/vec1.h"
#include "../interp/interp.h"

/*
This file contains the functor in charge of determining the position of a 
given magnitude within a vector. It is used by table functions to determine
the position of the input with respect to the table vectors.

Two options are present: binary (for non equispaced vectors, executes binary
search), and equispaced (for equispaced vectors, much faster search).

It is also used as a functor for each input vector in a table to accelerate
the computation of distances and differentials.
*/

namespace math {

// CLASS POS_FINDER
// ================
// ================

class MATH_API pos_finder {
public:
	virtual pos_finder* clone() const = 0;
	/**< cloner */
	virtual int search(const math::vec1& vec,
					   const double& input,
					   const double& diff) const = 0;
	/**< Given an input vector of magnitudes, an input magnitude and the difference
	between consecutive members	of the input vector (only if applicable), it returns
	an integer locating the latest position in the vector that is smaller OR equal
	to the input magnitude. For a size "n" vector, the response range is [-1,n-1].
	If -1, it means input is less than first (0) vector magnitude. If "n-1" (vector
	size), it means input is higher or equal than last (n-1) vector magnitude. */
	virtual double diff(const int& pos1,
						const int& pos2) const = 0;
	/**< obtains shortest numeric difference between the magnitude located
	at pos1 and pos. */
	virtual double diff(const int& posA1, const int& posA2,
						const int& posB1, const int& posB2,
						const int& posC1, const int& posC2) const = 0;
	/**< returns the [product of diff(posB1, posB2) by diff(posC1, posC2)] divided
	by diff(posA1, posA2). Check out specific version for equispaced vectors,
	which does something slightly different. */
	virtual double compute_Dratio(math::ratio& ratX,
								  const double& input,
								  const int& pos) const = 0;
	/**< Given an input magnitude and the position provided by the "find_index" method, it
	modifies the input ratio object so it can be employed to compute the differential
	together with that provided by the "compute_ratio" method. It computes the values
	corresponding to the input magnitude plus one ten-thousandth (1e-4) of the
	difference between positions "pos+1" and "pos" of the input vector. Returns this
	thousandth of difference by its later use computing the differential. */
	virtual double compute_final_diff(const double& input_dt,
									  const double difX) const = 0;
	/**< returns the division of the input magnitude differential with time by
	the input differential */
}; // closes class pos_finder

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS POS_FINDER_BINARY
// =======================
// =======================

class MATH_API pos_finder_binary : public pos_finder {
private:
	const math::vec1& _Ovec1;
	/**< constant reference to vector of magnitudes */
	const math::interp& _Ointerp;
	/**< constant reference to interpolation method */
 public:
	pos_finder_binary(const math::vec1& Ovec1,
					  const math::interp& Ointerp);
	/**< constructor based on vector of input magnitudes and interpolation
	method. */
	pos_finder_binary(const pos_finder_binary&);
	/**< copy constructor */
	~pos_finder_binary() {}
	/**< destructor */
	pos_finder_binary* clone() const;
	/**< cloner */	 
	
	int search(const math::vec1& vec,
			   const double& input,
			   const double&) const 
	{return math::search_binary(input, vec);}
	/**< Given an input vector of magnitudes, an input magnitude and the difference
	between consecutive members	of the input vector (only if applicable), it returns
	an integer locating the latest position in the vector that is smaller OR equal
	to the input magnitude. For a size "n" vector, the response range is [-1,n-1].
	If -1, it means input is less than first (0) vector magnitude. If "n-1" (vector
	size), it means input is higher or equal than last (n-1) vector magnitude. */
	double diff(const int& pos1,
				const int& pos2) const;
	/**< obtains shortest numeric difference between the magnitude located
	at pos1 and pos. */
	double diff(const int& posA1, const int& posA2,
				const int& posB1, const int& posB2,
				const int& posC1, const int& posC2) const;
	/**< returns the [product of diff(posB1, posB2) by diff(posC1, posC2)] divided
	by diff(posA1, posA2). Check out specific version for equispaced vectors,
	which does something slightly different.  */
	double compute_Dratio(math::ratio& ratX,
						  const double& input,
						  const int& pos) const;
	/**< Given an input magnitude and the position provided by the "find_index" method, it
	modifies the input ratio object so it can be employed to compute the differential
	together with that provided by the "compute_ratio" method. It computes the values
	corresponding to the input magnitude plus one ten-thousandth (1e-4) of the
	difference between positions "pos+1" and "pos" of the input vector. Returns this
	thousandth of difference by its later use computing the differential. */
	double compute_final_diff(const double& input_dt,
							  const double difX) const;
	/**< returns the division of the input magnitude differential with time by
	the input differential */
}; // closes class pos_finder_binary

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS POS_FINDER_EQUISPACED
// ===========================
// ===========================

class MATH_API pos_finder_equispaced : public pos_finder {
private:
	const math::vec1& _Ovec1;
	/**< constant reference to vector of magnitudes */
	const math::interp& _Ointerp;
	/**< constant reference to interpolation method */
	double _diff;
	/**< difference between consecutive inputs */
public:
	pos_finder_equispaced(const math::vec1& Ovec1,
						  const math::interp& Ointerp,
						  double diff);
	/**< constructor based on vector of input magnitudes, interpolation
	method, and difference between consecutive inputs. */
	pos_finder_equispaced(const pos_finder_equispaced&);
	/**< copy constructor */
	~pos_finder_equispaced() {}
	/**< destructor */
	pos_finder_equispaced* clone() const;
	/**< cloner */

	int search(const math::vec1& vec,
			   const double& input,
			   const double& diff) const 
	{return math::search_equispaced(input,vec, diff);}
	/**< Given an input vector of magnitudes, an input magnitude and the difference
	between consecutive members	of the input vector (only if applicable), it returns
	an integer locating the latest position in the vector that is smaller OR equal
	to the input magnitude. For a size "n" vector, the response range is [-1,n-1].
	If -1, it means input is less than first (0) vector magnitude. If "n-1" (vector
	size), it means input is higher or equal than last (n-1) vector magnitude. */
	double diff(const int& pos1,
				const int& pos2) const;
	/**< obtains shortest numeric difference between the magnitude located
	at pos1 and pos. */
	double diff(const int& posA1, const int& posA2,
				const int& posB1, const int& posB2,
				const int& posC1, const int& posC2) const;
	/**< returns the [product of diff(posB1, posB2) by diff(posC1, posC2)] divided
	by diff(posA1, posA2). For equispaced vectors, it does not return this but
	always the shortest numeric difference between consecutive magnitudes. The
	way this method is called by the tables this is always the same but it is not
	if called with random numbers. */
	double compute_Dratio(math::ratio& ratX,
						  const double& input,
						  const int& pos) const;
	/**< Given an input magnitude and the position provided by the "find_index" method, it
	modifies the input ratio object so it can be employed to compute the differential
	together with that provided by the "compute_ratio" method. It computes the values
	corresponding to the input magnitude plus one ten-thousandth (1e-4) of the
	difference between positions "pos+1" and "pos" of the input vector. Returns this
	thousandth of difference by its later use computing the differential. In the case
	of equispaced vector, it only updates the input ratio, returning zero. */
	double compute_final_diff(const double& input_dt,
							  const double difX) const;
	/**< returns the division of the input magnitude differential with time by
	the input differential. In the case of equispaced vector, it neglects the input
	difX (which is zero) and replaces it by the difference between consecutive input
	points divided by one ten-thousandth (1e-4) */
}; // closes class pos_finder_equispaced

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

} // closes namespace math

#endif



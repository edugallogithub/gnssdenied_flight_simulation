#include "pos_finder.h"
#include "../interp/interp.h"

// CLASS POS_FINDER_BINARY
// =======================
// =======================

math::pos_finder_binary::pos_finder_binary(const math::vec1& Ovec1,
											  const math::interp& Ointerp)
: _Ovec1(Ovec1), _Ointerp(Ointerp) {
}
/* constructor based on vector of input magnitudes and interpolation method */

math::pos_finder_binary::pos_finder_binary(const pos_finder_binary& op2)
: _Ovec1(op2._Ovec1), _Ointerp(op2._Ointerp) {
}
/* copy constructor */

math::pos_finder_binary* math::pos_finder_binary::clone() const {
	return new math::pos_finder_binary(*this);
}
/* cloner */

double math::pos_finder_binary::diff(const int& pos1,
										const int& pos2) const {
	return _Ovec1.diff(pos1, pos2);
}
/* obtains shortest numeric difference between the magnitude located
at pos1 and pos. */

double math::pos_finder_binary::diff(const int& posA1, const int& posA2,
										const int& posB1, const int& posB2,
										const int& posC1, const int& posC2) const {
	return _Ovec1.diff(posB1, posB2) *
		   _Ovec1.diff(posC1, posC2) / _Ovec1.diff(posA1, posA2);
}
/* returns the [product of diff(posB1, posB2) by diff(posC1, posC2)] divided
by diff(posA1, posA2). Check out specific version for equispaced vectors,
which does something slightly different.  */

double math::pos_finder_binary::compute_Dratio(math::ratio& ratX,
												  const double& input,
												  const int& pos) const {											  
	return _Ointerp.compute_Dratio(ratX, input, _Ovec1, pos);
}
/* Given an input magnitude and the position provided by the "find_index" method, it
modifies the input ratio object so it can be employed to compute the differential
together with that provided by the "compute_ratio" method. It computes the values
corresponding to the input magnitude plus one ten-thousandth (1e-4) of the
difference between positions "pos+1" and "pos" of the input vector. Returns this
thousandth of difference by its later use computing the differential. */

double math::pos_finder_binary::compute_final_diff(const double& input_dt,
													  const double diffX) const {
	return input_dt / diffX;
}
/* returns the division of the input magnitude differential with time by
the input differential */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS POS_FINDER_EQUISPACED
// ===========================
// ===========================

math::pos_finder_equispaced::pos_finder_equispaced(const math::vec1& Ovec1,
													  const math::interp& Ointerp,
													  double diff)
: _Ovec1(Ovec1), _Ointerp(Ointerp), _diff(diff) {
}
/* constructor based on vector of input magnitudes, interpolation
method, and difference between consecutive inputs. */

math::pos_finder_equispaced::pos_finder_equispaced(const pos_finder_equispaced& op2)
: _Ovec1(op2._Ovec1), _Ointerp(op2._Ointerp), _diff(op2._diff) {
}
/* copy constructor */

math::pos_finder_equispaced* math::pos_finder_equispaced::clone() const {
	return new math::pos_finder_equispaced(*this);
}
/* cloner */

double math::pos_finder_equispaced::diff(const int& pos1,
											const int& pos2) const {
	return _diff * (pos1 - pos2);										  
}
/* obtains shortest numeric difference between the magnitude located
at pos1 and pos. */

double math::pos_finder_equispaced::diff(const int& posA1, const int& posA2,
											const int& posB1, const int& posB2,
											const int& posC1, const int& posC2) const {
	return _diff;				 											 
}
/* returns the [product of diff(posB1, posB2) by diff(posC1, posC2)] divided
by diff(posA1, posA2). For equispaced vectors, it does not return this but
always the shortest numeric difference between consecutive magnitudes. The
way this method is called by the tables this is always the same but it is not
if called with random numbers. */

double math::pos_finder_equispaced::compute_Dratio(math::ratio& ratX,
													  const double& input,
													  const int& pos) const {
	_Ointerp.compute_Dratio(ratX, input, _Ovec1, pos);
	//double a = _Ointerp.compute_Dratio(ratX, input, _Ovec1, pos);
	// the rat is modified but the return shall never be used
	return 0.;
}
/* Given an input magnitude and the position provided by the "find_index" method, it
modifies the input ratio object so it can be employed to compute the differential
together with that provided by the "compute_ratio" method. It computes the values
corresponding to the input magnitude plus one ten-thousandth (1e-4) of the
difference between positions "pos+1" and "pos" of the input vector. Returns this
thousandth of difference by its later use computing the differential. In the case
of equispaced vector, it only updates the input ratio, returning zero. */

double math::pos_finder_equispaced::compute_final_diff(const double& input_dt,
														  const double difX) const {											  
	return input_dt / (1e-4 * _diff);
}
/* returns the division of the input magnitude differential with time by
the input differential. In the case of equispaced vector, it neglects the input
difX (which is zero) and replaces it by the difference between consecutive input
points divided by one ten-thousandth (1e-4) */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////








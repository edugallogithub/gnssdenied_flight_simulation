#ifndef MATH_POOL
#define MATH_POOL

#include "../math.h"
#include <vector>
#include <cstddef>
#include <iostream>

/*
This file implements the template pool_, whose purpose is to facilitate
the reuse of objects which are expensive to create and delete, but can
be returned to their initial state with the method reset(). If all object
intances are taken and returned to the pool, no instance is deleted until
the pool is deleted. The pool is initialized to a given number (input) of
objects, and then increased if necessary.

MODE OF USE:
Obtain a new object pointer with the method from_pool(). When no longer
necessary, return it to the pool for resetting enabling its reutilization
with the method to_pool(). Obtain a new object pointer equal to another
one with the method copy().
*/


namespace math {

// TEMPLATE POOL
// =============
// =============

template<typename F>
class pool_ {
private:
	std::vector<F*> _vec;
	/**< vector containing typenames F. */
	pool_(const pool_<F>&);
	/**< copy constructor not implemented */
	pool_& operator=(const pool_<F>&);
	/**< overloaded operator = (assignment) not implemented */
public:
	typedef F type;
	/**< allows outside methods to use typename F without knowing what it is */
	pool_(unsigned short);
	/**< constructor */
	virtual ~pool_();
	/**< destructor */
	F* from_pool();
	/**< removes a typename F from the pool and returns it */
	void to_pool(F*);
	/**< pushes a new typename F pointer to the pool, reseting it */
	F* copy(const F&);
	/**< removes a typename F pointer from the pool, copies the	input
	into it, and returns it */
}; // closes template pool_

//template<typename F> const unsigned short pool_<F>::_init_size = 5000;
/* initialization size for the pool */

template<typename F> pool_<F>::pool_(unsigned short siz)
: _vec(siz, 0) {
	for (int i = 0; i != siz; ++i) {
		_vec[i] = new F();
	}
}
/* constructor */

template<typename F> pool_<F>::~pool_() {
	for (int i = 0; i != _vec.size(); ++i) {
		delete _vec[i];
	}
}
/* destructor */

template<typename F> F* pool_<F>::from_pool() {
	if (_vec.size() != 0) { // pool not empty
		F* result = _vec.back();
		_vec.pop_back();
		return result;
	}
	else {
		return new F();
	}
}
/* removes a typename F from the pool and returns it */

template<typename F> void pool_<F>::to_pool(F* pF) {
	pF->reset();
	_vec.push_back(pF);
}
/* pushes a new typename F pointer to the pool, reseting it */

template<typename F> F* pool_<F>::copy(const F& oF) {
	F* result = from_pool();
	*result = oF;
	return result;
}
/* removes a typename F pointer from the pool, copies the	input
into it, and returns it */

} // closes namespace math

#endif

#ifndef MATH_RATIO_MGR_H
#define MATH_RATIO_MGR_H

#include "../math.h"
#include "ratio.h"
#include "../templates/pool_.h"
#include "../templates/singleton_holder_.h"

/*
This file contains the ratio_mgr class (singleton), which contains four pools of 
ratio models (for interpolation) of classes ratio_linear, ratio_quadratic, 
ratio_cubic, and ratio_spline respectively.

ACCURACY:
- N/A

LEFT:
- N/A.

MODE OF USE:
Employ the methods (from_pool, copy, to_pool) instead of the ratio classes methods
(constructor, copy_constructor, destructor). This avoids the unnecessary (and slightly
expensive) constructions and deletions during the program execution. Once a ratio model
is no longer required, it is returned to the pool as it is (it is not reset - although
a dummy reset method is called - as it would be expensive to do so and the ratio object
is updated before further use anyway) so it can be employed again, instead of deleting
it to construct it later. This is done automatically by the to_pool method.
In case of the ratio_linear and ratio_spline models, there are no pools as the creation
is quite cheap.
*/

namespace math {

// CLASS RATIO_MANAGER
// ===================
// ===================

class MATH_API ratio_mgr {
private:
	ratio_mgr();
	/**< empty constructor */
	ratio_mgr(const ratio_mgr&);
	/**< copy constructor not implemented */
	ratio_mgr& operator=(const ratio_mgr&);
	/**< overloaded operator = (assignment) not implemented */
	typedef math::singleton_holder_<math::ratio_mgr> instance;
	/**< simplify name within class */

	static const unsigned short _init_siz;
	/**< initial size of pool */
	typedef math::pool_<math::ratio_quadratic> pool_quadratic;
	typedef math::pool_<math::ratio_cubic> pool_cubic;
	/** simplify names within class */
	pool_quadratic _pool_quadratic;
	pool_cubic     _pool_cubic;
	/**< pools of ratio_quadratic and ratio_cubic models */
public:
	friend class math::singleton_holder_<math::ratio_mgr>;
	/**< template can access private methods */
	virtual ~ratio_mgr();
	/**< destructor */

	static math::ratio_linear*    from_pool_linear();
	static math::ratio_quadratic* from_pool_quadratic();
	static math::ratio_cubic*     from_pool_cubic();
	static math::ratio_hermite*   from_pool_hermite();
	static math::ratio_spline*    from_pool_spline();
	/**< returns pointer to new ratio objects taken from pools */
	static void to_pool_linear(math::ratio_linear* p);
	static void to_pool_quadratic(math::ratio_quadratic* p);
	static void to_pool_cubic(math::ratio_cubic* p);
	static void to_pool_hermite(math::ratio_hermite* p);
	static void to_pool_spline(math::ratio_spline* p);
	/**< adds ratio object pointer to pool */
	static math::ratio_linear*    copy(const math::ratio_linear& o);
	static math::ratio_quadratic* copy(const math::ratio_quadratic& o);
	static math::ratio_cubic*     copy(const math::ratio_cubic& o);
	static math::ratio_hermite*   copy(const math::ratio_hermite& o);
	static math::ratio_spline*    copy(const math::ratio_spline& o);
	/**< returns pointer to new ratio object taken from pool equal to input */
}; // closes class ratio_mgr

} // closes namespace math

#endif

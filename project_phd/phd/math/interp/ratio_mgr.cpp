#include "ratio_mgr.h"

// CLASS RATIO_MANAGER
// ===================
// ===================

const unsigned short math::ratio_mgr::_init_siz = 10;
/* initial size of pool */

math::ratio_mgr::ratio_mgr()
: _pool_quadratic(_init_siz), _pool_cubic(_init_siz) {}
/* empty constructor */

math::ratio_mgr::~ratio_mgr() {
}
/* destructor */

math::ratio_linear* math::ratio_mgr::from_pool_linear() {
	return new math::ratio_linear();
}
math::ratio_quadratic* math::ratio_mgr::from_pool_quadratic() {
	return instance::get_instance()._pool_quadratic.from_pool();
}
math::ratio_cubic* math::ratio_mgr::from_pool_cubic() {
	return instance::get_instance()._pool_cubic.from_pool();
}
math::ratio_hermite* math::ratio_mgr::from_pool_hermite() {
	return new math::ratio_hermite();
}
math::ratio_spline* math::ratio_mgr::from_pool_spline() {
	return new math::ratio_spline();
}
/* returns pointer to new ratio objects taken from pools */

void math::ratio_mgr::to_pool_linear(math::ratio_linear* p) {
	delete p;
}
void math::ratio_mgr::to_pool_quadratic(math::ratio_quadratic* p) {
	instance::get_instance()._pool_quadratic.to_pool(p);
}
void math::ratio_mgr::to_pool_cubic(math::ratio_cubic* p) {
	instance::get_instance()._pool_cubic.to_pool(p);	
}
void math::ratio_mgr::to_pool_hermite(math::ratio_hermite* p) {
	delete p;
}
void math::ratio_mgr::to_pool_spline(math::ratio_spline* p) {
	delete p;
}
/* adds ratio object pointer to pool */

math::ratio_linear* math::ratio_mgr::copy(const math::ratio_linear& o) {
	return new math::ratio_linear(o);
}
math::ratio_quadratic* math::ratio_mgr::copy(const math::ratio_quadratic& o) {
	return instance::get_instance()._pool_quadratic.copy(o);
}
math::ratio_cubic* math::ratio_mgr::copy(const math::ratio_cubic& o) {
	return instance::get_instance()._pool_cubic.copy(o);	
}
math::ratio_hermite* math::ratio_mgr::copy(const math::ratio_hermite& o) {
	return new math::ratio_hermite(o);
}
math::ratio_spline* math::ratio_mgr::copy(const math::ratio_spline& o) {
	return new math::ratio_spline(o);
}
/* returns pointer to new ratio object taken from pool equal to input */



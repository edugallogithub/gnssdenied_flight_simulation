#ifndef MATH_SINGLETON_HOLDER
#define MATH_SINGLETON_HOLDER

#include "../math.h"
#include <boost/scoped_ptr.hpp>

/*
This file implementes the singleton holder template, useful to transform any
other class T in a singleton. 

MODE OF USE:
The singleton holder shall be a friend of the singleton T, which shall have a
public destructor, a private empty constructor (accesible to the singleton 
holder as it is a friend), and forbidden copy constructor and assignment
operators. The singleton T shall also include a singleton holder private
attribute as "typedef math::singleton_holder_<T> _instance" and a public
get_instance() method to provide access to it containing
"return _instance::get_instance()".
*/

namespace math {

template <typename T> class singleton_holder_ {
private:
	static boost::scoped_ptr<T> _instance;
	/**< smart pointer to type */
    singleton_holder_();
	/**< empty constructor not implemented */
	singleton_holder_(const singleton_holder_&);
	/**< copy constructor not implemented */
	singleton_holder_& operator=(const singleton_holder_&);
	/**< overloaded operator = (assignment) not implemented */
	~singleton_holder_() {
	}
	/**< destructor not implemented */
public:
	typedef T type;
	/**< allows outside methods to use typename T without knowing what it is */
	inline static T& get_instance() {
		if (_instance.get() == 0) {
			_instance.reset(new T);
		}
		return *_instance;
    }
	/**< return reference to single instance */
}; // closes class singleton
    
template<typename T> 
boost::scoped_ptr<T> singleton_holder_<T>::_instance;
} 

// closes namespace math

#endif










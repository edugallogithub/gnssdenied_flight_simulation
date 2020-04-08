#ifndef MATH_FACTORY
#define MATH_FACTORY

#include "../math.h"
#include <map>

/*
This file implementes the object factory template, useful to obtain pointers
to a base class based on a key that identifies it. 

The template requires a base class, a key identifying the different derivate
classes (generally strings), and a function pointer returning pointers to the
base class. The insert method adds a couple of key-function pointer to the map,
where it is stored for later use. The erase method removes it. Finally, the
create method (for which only the one coinciding with the template definition)
is compiled, returns a pointer to the base class.
*/

namespace math {
    
// TEMPLATE FACTORY_
// =================
// =================

template <class base_class, 
		  typename key_type_,
		  typename base_class_creator_ = base_class* (*)() >
class factory_ {
private:
	typedef std::map<key_type_, base_class_creator_> map;
    /**< name simplification */
	map _map;
	/**< map instance */
public:
	typedef key_type_ key_type;
	typedef base_class_creator_ base_class_creator;
	/**< allows outside methods to use typenames without knowing whet they are*/

	bool insert(const key_type& index,
				base_class_creator creator) {
		return _map.insert(typename map::value_type(index, creator)).second;
 	}
	/**< inserts the id and the creator function into the factory map,
	returning true if successful, false otherwise. 
	map::value_type is equal to std::pair */        
	bool erase(const key_type& index) {
		return _map.erase(index) == 1;
	}
	/**< erases index from factory map, returning true if successful
	(has erased 1 instance), false otherwise */        
	base_class* create(const key_type& index) {
		typename map::iterator I = _map.find(index);
        if (I != _map.end()) {return (I->second)();}
		else                 {return 0;}
		// do not assert or throw an exception if key not found, as sometimes
		// (as in XML methods) this may be OK. Client applications (implementations
		// of the template) should understand that 0 may be returned.
	}   
	/**< returns pointer to object corresponding to key, 0 otherwise */ 
	base_class* create(const key_type& index,
					   const double& value) {
		typename map::iterator I = _map.find(index);
		if (I != _map.end()) {return (I->second)(value);}
		else                 {return 0;}
		// do not assert or throw an exception if key not found, as sometimes
		// (as in XML methods) this may be OK. Client applications (implementations
		// of the template) should understand that 0 may be returned.
	}
	/**< returns pointer to object corresponding to key, 0 otherwise. Requires a 
	function pointer with inputs a double and a unit */
	base_class* create(const key_type& index,
					   const unsigned short& siz) {
		typename map::iterator I = _map.find(index);
		if (I != _map.end()) {return (I->second)(siz);}
		else                 {return 0;}
		// do not assert or throw an exception if key not found, as sometimes
		// (as in XML methods) this may be OK. Client applications (implementations
		// of the template) should understand that 0 may be returned.
	}
	/**< returns pointer to object corresponding to key, 0 otherwise. Requires a 
	function pointer with inputs a unsigned short and a unit */
}; // closes class factory_

} // closes namespace math

#endif 

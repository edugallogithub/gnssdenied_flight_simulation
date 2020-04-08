#include "unit_test.h"
#include <iostream>
#include <iomanip>

// CLASS COUNTER
// =============
// =============

jail::counter::counter()
: _flag_partial(false), _id(0), _passed(0), _failed(0), _first_failed(0) {
}
/* constructor for global counter */

jail::counter::counter(const unsigned int& id)
: _flag_partial(true), _id(id), _passed(0), _failed(0), _first_failed(0) {
}
/* constructor for partial counter based on current text index */

void jail::counter::write_results() {
    if (_flag_partial == true) {
        std::cout << std::endl;
        std::cout << "PARTIAL executed :" << std::setw(10) << _passed + _failed << std::endl;
        std::cout << "PARTIAL failed   :" << std::setw(10) << _failed << std::endl;
        if (_failed > 0) {
            std::cout << "PARTIAL 1st fail :" << std::setw(10) << _first_failed << std::endl;
        }
        std::cout << "PARTIAL passed   :" << std::setw(10) << _passed << std::endl;
        std::cout << std::endl;
    }
    else {
        std::cout << "TOTAL executed   :" << std::setw(10) << _passed + _failed << std::endl;
        std::cout << "TOTAL failed     :" << std::setw(10) << _failed << std::endl;
        if (_failed > 0) {
            std::cout << "TOTAL 1st fail   :" << std::setw(10) << _first_failed << std::endl;
        }
        std::cout << "TOTAL passed     :" << std::setw(10) << _passed << std::endl;
    }
}
/* writes the accumulated test results on the console */

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS UNIT_TEST
// ===============
// ===============

jail::unit_test::unit_test(counter& Ocounter_global)
: _Pcounter_global(&Ocounter_global), _Ocounter_partial(Ocounter_global._id) {
}
/* constructor based on global error counter */

jail::unit_test::~unit_test() {}
/* destructor (DO NOT use default as there are weak pointers) */

void jail::unit_test::finished() {
    _Ocounter_partial.write_results();
    _Pcounter_global->_passed += _Ocounter_partial._passed;
    _Pcounter_global->_failed += _Ocounter_partial._failed;
}
/* writes partial counter results, and adds them to the global counter */

void jail::unit_test::check(const std::string& st, const double& first, const double& second, const double& tol) {
    _Ocounter_partial._id++;
    _Pcounter_global->_id++;
    //std::cout << "#" << _Pcounter_global->total() + _Ocounter_partial.total() + 1 << ": ";

    std::cout << "#" << _Pcounter_global->_id << ": ";

    std::cout <<  "\t"<< st << ":\t" ;
	if ((first > (second - tol)) &&	(first < (second + tol))) { // test successful
        std::cout << "passed" << std::endl;
		_Ocounter_partial._passed++;
	}
	else { // test NOT successful
        std::cout << "failed (first= " << first << ", second= " << second << ")" << std::endl;
        if (_Ocounter_partial._first_failed == 0) {
            _Ocounter_partial._first_failed = _Ocounter_partial._id;
        }
        if (_Pcounter_global->_first_failed == 0) {
            _Pcounter_global->_first_failed = _Pcounter_global->_id;
        }
        _Ocounter_partial._failed++;
    }
}
/* evaluates two doubles with a given tolerance, writing proper messages on console */

void jail::unit_test::checkb(const std::string& st, bool first, bool second) {
	check(st, (double)(first?1:0), (double)(second?1:0));
}
/* evaluates two booleans, writing proper messages on console */

void jail::unit_test::checki(const std::string& st,	const int& first, const int& second) {
	check(st, (double)first, (double)second);
}
/* evaluates two integers, writing proper messages on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////





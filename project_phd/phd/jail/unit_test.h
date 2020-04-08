#ifndef JAIL_UNIT_TEST
#define JAIL_UNIT_TEST

#include "jail.h"
#include <string>

namespace jail {

// CLASS COUNTER
// =============
// =============

class JAIL_API counter {
private:
    /**< true for partial tests, false for global ones */
    bool _flag_partial;
public:
    /**< constructor for global counter */
    counter();
    /**< constructor for partial counter based on current text index */
    explicit counter(const unsigned int& id);
    /**< destructor */
    ~counter() = default;

    /**< copy constructor */
    counter(const counter&) = delete;
    /**< move constructor */
    counter(counter&&) = delete;
    /**< copy assignment */
    counter& operator=(const counter&) = delete;
    /**< move assignment */
    counter& operator=(counter&&) = delete;

    /**< index of current test */
    unsigned int _id;
    /**< number of passed tests */
    unsigned int _passed;
    /**< number of failed tests */
    unsigned int _failed;
    /**< index of first failed test */
    unsigned int _first_failed;

    /**< writes the accumulated test results on the console */
    void write_results();
}; // closes class counter

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

// CLASS UNIT_TEST
// ===============
// ===============

class JAIL_API unit_test {
protected:
	/**< partial error counter */
	jail::counter _Ocounter_partial;
	/**< weak pointer to global error counter */
	jail::counter* _Pcounter_global;

	/**< writes partial counter results, and adds them to the global counter */
	void finished();
public:
    /**< empty constructor */
    unit_test() = delete;
	/**< constructor based on global error counter */
	explicit unit_test(jail::counter& Ocounter);
	/**< destructor */
	virtual ~unit_test();
    /**< copy constructor */
    unit_test(const unit_test&) = delete;
    /**< move constructor */
    unit_test(unit_test&&) = delete;
    /**< copy assignment */
    unit_test& operator=(const unit_test&) = delete;
    /**< move assignment */
    unit_test& operator=(unit_test&&) = delete;

    /**< execute tests */
    virtual void run() {}
	/**< evaluates two doubles with a given tolerance, writing proper messages on console */
	void check(const std::string& st, const double& first, const double& second, const double& tolerance = 1e-8);
    /**< evaluates two booleans, writing proper messages on console */
	void checkb(const std::string& st, bool first, bool second);
	/**< evaluates two integers, writing proper messages on console */
	void checki(const std::string& st, const int& first, const int& second);
}; // closes class unit_test

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////

} // closes namespace jail

#endif 


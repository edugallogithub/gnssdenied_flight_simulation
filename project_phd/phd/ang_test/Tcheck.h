#ifndef ATT_TEST_CHECK
#define ATT_TEST_CHECK

#include "ang_test.h"
#include <jail/unit_test.h>

namespace ang {
namespace test {
	
class Tcheck: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tcheck(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;	

    /**< specific tests */
	void test1();

};

} // closes namespace test
} // closes namespace ang

#endif


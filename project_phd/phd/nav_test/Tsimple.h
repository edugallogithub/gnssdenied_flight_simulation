#ifndef NAV_TEST_SIMPLE
#define NAV_TEST_SIMPLE

#include "nav_test.h"
#include "jail/unit_test.h"

namespace nav {
namespace test {
	
class Tsimple: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tsimple(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */

    void test_straight_motion();
    void test_straight_motion2();
    void test_rotation_motion();

};

} // closes namespace test
} // closes namespace nav

#endif


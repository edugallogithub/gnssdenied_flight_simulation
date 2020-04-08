#ifndef ACFT_TEST_SENSOR
#define ACFT_TEST_SENSOR

#include "acft_test.h"
#include "jail/unit_test.h"

namespace acft {
namespace test {
	
class Tsensor: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tsensor(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */

    void test_single_inertial();
    void test_comparison_gyr_theory();
    void test_comparison_acc_theory();
    /////////////////////////////////////////////////////////////////
    // WATCH OUT -> I NEED TO DEACTIVATE THE BIAS DRIFT LIMITS
    /////////////////////////////////////////////////////////////////
};

} // closes namespace test

} // closes namespace acft
#endif


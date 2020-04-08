#ifndef ENV_TEST_TURB
#define ENV_TEST_TURB

#include "env_test.h"
#include "jail/unit_test.h"

namespace env {
namespace test {
	
class Tturb: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tturb(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */
	void test_turb();
    void test_turb_obtain_differential(const double& turb_factor);
};

}; // closes namespace test
} // closes namespace env

#endif


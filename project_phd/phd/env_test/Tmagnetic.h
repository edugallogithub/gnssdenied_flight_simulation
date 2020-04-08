#ifndef ENV_TEST_MAGNETIC
#define ENV_TEST_MAGNETIC

#include "env_test.h"
#include "jail/unit_test.h"

/*
 * This file verifies the proper behavior of the magnetism classes:
 * - test_model focuses on the model magnetic fields, which are either constant
 *   or bilinear in longitude and latitude.
 * - test_truth obtains metrics for the difference between the model and real
 *   magnetic fields.
 */

namespace env {
namespace test {
	
class Tmagnetic: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tmagnetic(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */

    void test_model();
    void test_truth();
};

}; // closes namespace test
} // closes namespace env

#endif


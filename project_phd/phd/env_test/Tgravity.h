#ifndef ENV_TEST_GRAVITY
#define ENV_TEST_GRAVITY

#include "env_test.h"
#include "jail/unit_test.h"

/*This file verifies the proper behavior of the magnetism classes:
 * - test_model focuses on the model gravity, comparing the previous EGM96 ellipsoidal
 *   gravitation plus centrifugal potential, against the new WGS84 gravity.
 * - test_truth obtains metrics for the difference between the model and real
 *   gravity fields.
 */

namespace env {
namespace test {
	
class Tgravity: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tgravity(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */

    void test_model();
    void test_truth();
};

}; // closes namespace test
} // closes namespace env

#endif


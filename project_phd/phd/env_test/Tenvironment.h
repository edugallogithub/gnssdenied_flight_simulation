#ifndef ENV_TEST_ENVIRONMENT
#define ENV_TEST_ENVIRONMENT

#include "env_test.h"
#include "jail/unit_test.h"

/*
This file contains tests to verify the proper behavior of the files in the main folder of the "env" library.
*/

namespace env {
namespace test {
	
class Tenvironment: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tenvironment(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */
	void test_atm();
    void test_DeltaT_Deltap();
    void test_atm_inverse();
    void test_offsets_ramp();
    void test_wind_ramp();
	void test_coord();
	void test_geo();
	void test_speed();
};

}; // closes namespace test
} // closes namespace env

#endif


#ifndef ATT_TEST_SO3
#define ATT_TEST_SO3

#include "ang_test.h"
#include "jail/unit_test.h"

/*
This file contains tests to verify the proper behavior of the different methods
of representing a rotation (euler, dcm, rodrigues, rotv, so3).
*/

namespace ang {
namespace test {
	
class Tso3: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tso3(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;	

    /**< specific tests */
	void test_so3();
    void test_euler();
    void test_rodrigues_jacobian();
    void test_exp_log_maps_small();
    void test_exp_log_maps();
    void test_power();
    void test_slerp();
    void test_quat_4_solutions();
};

} // closes namespace test

} // closes namespace ang

#endif


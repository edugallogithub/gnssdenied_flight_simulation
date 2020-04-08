#ifndef NAV_TEST_ALIGN_COARSE
#define NAV_TEST_ALIGN_COARSE

#include "nav_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"
#include "jail/unit_test.h"

namespace nav {
namespace test {
	
class Talign_coarse: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Talign_coarse(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */

    void test_align_single(const ang::euler& euler_nb_truth, sens::logic::GYR_ID gyr_id, sens::logic::ACC_ID acc_id, sens::logic::MAG_ID mag_id);
    void test_align_multiple(sens::logic::GYR_ID gyr_id, sens::logic::ACC_ID acc_id, sens::logic::MAG_ID mag_id);

};

} // closes namespace test
} // closes namespace nav

#endif


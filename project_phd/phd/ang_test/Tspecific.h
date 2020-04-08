#ifndef ATT_TEST_SPECIFIC
#define ATT_TEST_SPECIFIC

#include "ang_test.h"
#include <jail/unit_test.h>

/*
This test verifies the proper transformation between BFS (Body Fixed System) and
CRS (Frame Reference System), as well as between NED (North - East - Down) and
ENU (East - North - Up) vectors.
*/

namespace ang {
namespace test {
	
class Tspecific: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tspecific(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;	

    /**< specific tests */
    void test_body_camera();
    void test_skew();
};

} // closes namespace test
} // closes namespace ang

#endif


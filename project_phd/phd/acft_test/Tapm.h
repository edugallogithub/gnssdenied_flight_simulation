#ifndef ACFT_TEST_APM
#define ACFT_TEST_APM

#include "acft_test.h"
#include "jail/unit_test.h"

namespace acft {
namespace test {
	
class Tapm: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tapm(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */

    void test_aero0();   // comparison of aero0 with matlab results
    void test_aero1();   // comparison of aero1 with matlab results
    void test_aero2();   // comparison of aero2 with matlab results
    void test_aero3();   // comparison of aero3 with matlab results
    void test_prop();   // comparison of propulsion with matlab results
	void test_iner();

};

} // closes namespace test

} // closes namespace acft

#endif


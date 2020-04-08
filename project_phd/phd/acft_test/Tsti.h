#ifndef ACFT_TEST_STI
#define ACFT_TEST_STI

#include "acft_test.h"
#include "env/earth.h"
#include "jail/unit_test.h"

namespace acft {
namespace test {
	
class Tsti: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tsti(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */

    void test1(const double& psi_deg, const env::earth& Oearth);

};

} // closes namespace test

} // closes namespace acft

#endif


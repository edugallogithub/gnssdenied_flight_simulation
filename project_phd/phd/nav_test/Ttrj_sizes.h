#ifndef NAV_TEST_TRJ_SiZES
#define NAV_TEST_TRJ_SIZES

#include "nav_test.h"
#include "jail/unit_test.h"

namespace nav {
namespace test {
	
class Ttrj_sizes: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Ttrj_sizes(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */

    void test_sizes01();
    void test_sizes02();
    void test_sizes03();
    void test_sizes04();
    void test_sizes05();
    void test_sizes06();
    void test_sizes07();
    void test_sizes08();
    void test_sizes11();
    void test_sizes12();
    void test_sizes13();
    void test_sizes14();
    void test_sizes15();
    void test_sizes16();
    void test_sizes17();
    void test_sizes18();
    void test_sizes21();
    void test_sizes22();
    void test_sizes23();


};

} // closes namespace test
} // closes namespace nav

#endif


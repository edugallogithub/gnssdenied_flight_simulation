#ifndef RESULTS_SIM_TEST_PLOTS_SEEDS
#define RESULTS_SIM_TEST_PLOTS_SEEDS

#include "results_sim_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_seeds {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_seeds(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    void test_guidance_bearing  (unsigned short& seed_init, unsigned short& seed_end);
    void test_guidance_alt_speed(unsigned short& seed_init, unsigned short& seed_end);
    void test_offsets           (unsigned short& seed_init, unsigned short& seed_end);
    void test_wind              (unsigned short& seed_init, unsigned short& seed_end);
    void test_gravity           (unsigned short& seed_init, unsigned short& seed_end);
    void test_magnetic          (unsigned short& seed_init, unsigned short& seed_end);

};

} // closes namespace test
} // closes namespace nav

#endif


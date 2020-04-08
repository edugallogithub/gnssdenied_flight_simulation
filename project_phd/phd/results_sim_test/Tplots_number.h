#ifndef RESULTS_SIM_TEST_PLOTS_NUMBER
#define RESULTS_SIM_TEST_PLOTS_NUMBER

#include "results_sim_test.h"
#include <string>

namespace nav {
namespace test {
	
class Tplots_number {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_number(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< checks the aggregated means taking into account progressively from seed_init to seed_end executions,
     * and computes aggregated mean and std for atttiude, error, final geometric altitude error, and final
     * ground velocity error. Generates the following files:
     * - "error_filter_number_euler_deg.txt" --> to plot attitude aggregated metrics with nEX
     * - "error_filter_number_hend_m.txt" --> to plot final altitude aggregated metrics with nEX
     * - "error_filter_number_vnend_mps.txt" --> to plot final ground velocity aggregated metrics with nEX */
    void obtain_optimum_number(const std::string& st_first_folder, const unsigned short& seed_init, const unsigned short& seed_end);

};



} // closes namespace test
} // closes namespace nav

#endif


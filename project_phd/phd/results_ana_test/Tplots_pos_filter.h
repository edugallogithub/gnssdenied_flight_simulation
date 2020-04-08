#ifndef RESULTS_ANA_TEST_PLOTS_POS_FILTER
#define RESULTS_ANA_TEST_PLOTS_POS_FILTER

#include "results_ana_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_pos_filter {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_pos_filter(unsigned short& seed_init, unsigned short& seed_end);

    /**< execute tests and write results on console */
	void run();

    /**< reads the final horizontal position results from three sets of files, the baseline and two alternatives.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_integr_pos_hor_m_pc_part1.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_integr_pos_hor_m_pc_part2.txt" --> to plot horizontal error mean and std variation with time */
    void obtain_xhor_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final vertical position results from two sets of files, the baseline and one alternatives.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_integr_pos_h_m_part1.txt" --> to plot vertical error mean and std variation with time
     * - "versus_integr_pos_h_m_part2.txt" --> to plot vertical error mean and std variation with time */
    void obtain_h_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

};

} // closes namespace test
} // closes namespace nav

#endif


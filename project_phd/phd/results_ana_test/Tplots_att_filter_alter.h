#ifndef RESULTS_ANA_TEST_PLOTS_ATT_FILTER_ALTER
#define RESULTS_ANA_TEST_PLOTS_ATT_FILTER_ALTER

#include "results_ana_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_att_filter_alter {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_att_filter_alter(unsigned short& seed_init, unsigned short& seed_end);

    /**< execute tests and write results on console */
	void run();

    /**< reads the body attitude estimation results from three sets of files, the baseline, and two employing
     * different attitude filter implementations.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_att_filter_alter_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_att_filter_alter_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_att_filter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, and two employing
     * different attitude filter implementations.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_att_filter_alter_vned_mps.txt" --> to plot ground speed final error
     * - "versus_att_filter_alter_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_att_filter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads those time instants where the aircraft is turning and fills up a tet file "versus_att_filter_turn_location.txt" */
    void obtain_turn_location_att_filter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, nd two employing
     * different attitude filter implementations.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_att_filter_alter_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_att_filter_alter_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_att_filter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);
};

} // closes namespace test
} // closes namespace nav

#endif


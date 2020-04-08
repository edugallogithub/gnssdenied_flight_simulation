#ifndef RESULTS_ANA_TEST_PLOTS_GNSS
#define RESULTS_ANA_TEST_PLOTS_GNSS

#include "results_ana_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_gnss {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_gnss(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests */
	void run();

    /**< reads the body attitude estimation results from two sets of files, the baseline and the one employing GNSS.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_gnss_att_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_gnss_att_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_metrics(const std::string& st_first_folder, const std::string& st_first_folder_gnss, unsigned short& seed_init, unsigned short& seed_end);


    /**< reads the final altitude estimation results from two sets of files, the baseline and the one employing GNSS.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_gnss_pos_h_m.txt" --> to plot altitude mean and std variation with time
     * - "versus_gnss_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_h_metrics(const std::string& st_first_folder, const std::string& st_first_folder_gnss, unsigned short& seed_init, unsigned short& seed_end);


    /**< reads the final horizontal position results from two sets of files, the baseline and the one employing GNSS.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_gnss_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_gnss_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_metrics(const std::string& st_first_folder, const std::string& st_first_folder_gnss, unsigned short& seed_init, unsigned short& seed_end);

};

} // closes namespace test
} // closes namespace nav

#endif


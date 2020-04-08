#ifndef RESULTS_SIM_TEST_PLOTS_ATTITUDE
#define RESULTS_SIM_TEST_PLOTS_ATTITUDE

#include "results_sim_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_attitude {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_attitude(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< reads the body attitude estimation results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_att_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "error_filter_att_euler_deg_lsqfit.txt" --> to plot least squares on previous plot
     * - "error_filter_att_euler_table.tex" --> script to directly generate table in Latex
     * - "error_filter_att_euler_lsqfit_table.tex --> script to directly generate lsq table in Latex
     * - "error_filter_att_euler_scatter.txt" --> for scatter plot of mean versus std */
    void obtain_euler_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);
    /**< reads the total gyroscope errors estimation results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_att_Egyr_dps.txt" --> to plot euler mean and std variation with time
     * - "error_filter_att_Egyr_dps_lsqfit.txt" --> to plot least squares on previous plot
     * - "error_filter_att_Egyr_table.tex" --> script to directly generate table in Latex
     * - "error_filter_att_Egyr_lsqfit_table.tex --> script to directly generate lsq table in Latex */
    void obtain_Egyr_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from a single file and generates the file
     * "error_filter_att_single_euler_deg.txt" (to be added to thesis, not Matlab) */
    void obtain_euler_single(const std::string& st_folder);
    /**< reads the total gyroscope error estimation results from a single file and generates the file
     * "error_filter_att_single_Egyr_dps.txt" (to be added to thesis, not Matlab) */
    void obtain_Egyr_single(const std::string& st_folder);

    /**< reads the body attitude estimation results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_att_alter_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "error_filter_att_alter_euler_table.tex" --> script to directly generate table in Latex
     * - "error_filter_att_alter_euler_scatter.txt" --> for scatter plot of mean versus std */
    void obtain_euler_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);
};

} // closes namespace test
} // closes namespace nav

#endif


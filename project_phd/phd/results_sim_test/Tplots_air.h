#ifndef RESULTS_SIM_TEST_PLOTS_AIR
#define RESULTS_SIM_TEST_PLOTS_AIR

#include "results_sim_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_air {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_air(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< reads the airspeed vector air data filter results from a group of files (those between the initial
     * and final seeds) for the trajectories identified by the initial string (which includes a seed but may
     * not be read at all if it does not fall in between the input seeds). It shows results on console and
     * also generates the following files (for thesis, not Matlab):
     * - "error_filter_air_vtasb_mpsdeg.txt" --> to plot vtas, alpha, and beta mean and std variation with time (only if flag_bias is false)
     * - "error_filter_air_vtasb_mpsdeg_lsqfit.txt" --> to plot least squares on previous plot (only if flag_bias is false)
     * - "error_filter_air_vtasb_table.tex" --> script to directly generate table in Latex (only if flag_bias is false)
     * - "error_filter_air_vtasb_bias_table.tex" --> script to directly generate table in Latex including biases (only if flag_bias is true)
     * - "error_filter_air_vtasb_lsqfit_table.tex --> script to directly generate lsq table in Latex (only if flag_bias is false) */
    void obtain_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end, bool flag_bias);

    /**< reads the other air data filter results from a group of files (those between the initial
     * and final seeds) for the trajectories identified by the initial string (which includes a seed but may
     * not be read at all if it does not fall in between the input seeds). It shows results on console and
     * also generates the following files (for thesis, not Matlab):
     * - "error_filter_air_other_mdeg.txt" --> to plot T, Hp, DeltaT mean and std variation with time
     * - "error_filter_air_other_mdeg_lsqfit.txt" --> to plot least squares on previous plot
     * - "error_filter_air_other_table.tex" --> script to directly generate table in Latex
     * - "error_filter_air_other_lsqfit_table.tex --> script to directly generate lsq table in Latex */
    void obtain_other_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the airspeed estimation results from a single file and generates the file
     * "error_filter_air_single_vtas_mps.txt" (to be added to thesis, not Matlab) */
    void obtain_vtas_single(const std::string& st_folder);
    /**< reads the pressure altitude estimation results from a single file and generates the file
     * "error_filter_air_single_Hp_m.txt" (to be added to thesis, not Matlab) */
    void obtain_Hp_single(const std::string& st_folder);
    /**< reads the temperature offset results from a single file and generates the file
     * "error_filter_air_single_DeltaT_degK.txt" (to be added to thesis, not Matlab) */
    void obtain_DeltaT_single(const std::string& st_folder);
};

} // closes namespace test
} // closes namespace nav

#endif


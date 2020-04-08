#ifndef RESULTS_SIM_TEST_PLOTS_CONTROL
#define RESULTS_SIM_TEST_PLOTS_CONTROL

#include "results_sim_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_control {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_control(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< reads the pressure altitude and true airspeed control results from a group of files (those between the
     * initial and final seeds) for the trajectories identified by the initial string (which includes a seed
     * but may not be read at all if it does not fall in between the input seeds). It shows results on console
     * and also generates the following files (for thesis, not Matlab):
     * - error_control_Hp_m.txt --> to plot Hp mean and std variation with time
     * - error_control_Hp_m_lsqfit.txt --> to plot least squares on previous plot
     * - error_control_vtas_m.txt --> to plot vtas mean and std variation with time
     * - error_control_vtas_mps_lsqfit.txt --> to plot least squares on previous plot
     * - error_control_Hp_vtas_table.tex --> script to directly generate table in Latex
     * - error_control_Hp_vtas_lsqfit_table.tex --> script to directly generate lsq table in Latex */
    void obtain_Hp_vtas_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);


    /**< reads the absolute bearing and sideslip control results from a group of files (those between the
     * initial and final seeds) for the trajectories identified by the initial string (which includes a seed
     * but may not be read at all if it does not fall in between the input seeds). It shows results on console
     * and also generates the following files (for thesis, not MatLab):
     * - error_control_chi_deg.txt --> to plot chi mean and std variation with time
     * - error_control_chi_deg_lsqfit.txt --> to plot least squares on previous plot
     * - error_control_beta_deg.txt --> to pot beta mean and std variation with time
     * - error_control_beta_deg_lsqfit.txt --> to plot least squares on previous plot
     * - error_control_chi_beta_table.tex --> script to directly generate table in Latex
     * - error_control_chi_beta_lsqfit_table.tex --> script to directly generate lsq table in Latex */
    void obtain_chi_beta_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

};



} // closes namespace test
} // closes namespace nav

#endif


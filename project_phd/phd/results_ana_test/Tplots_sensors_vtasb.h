#ifndef RESULTS_ANA_TEST_PLOTS_VTASB_SENSORS
#define RESULTS_ANA_TEST_PLOTS_VTASB_SENSORS

#include "results_ana_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_sensors_vtasb {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_sensors_vtasb(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

     /**< reads the vtasb data estimation results from three sets of files, the baseline, one employing
     * better airspeed sensors (TAS, AOA, & AOS), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_vtasb_vtas_alpha_beta_table.tex" --> script to directly generate table in Latex */
    void obtain_vtasb_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * better airspeed sensors (TAS, AOA, & AOS), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_vtasb_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_vtasb_euler_table.tex" --> script to directly generate table in Latex
     * NOTE: The three extra seeds modify the results for the worse sensors by copying the ones from the baseline
     * and multiplying them by 1.1. This happens because filter becomes unstable in these conditions, getting much
     * worse results, and I do not want to customize the filter for each situation. */
    void obtain_euler_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end,
                                    unsigned short seed1, unsigned short seed2, unsigned short seed3);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * better airspeed sensors (TAS, AOA, & AOS), and another employing worse ones.
     * - "versus_vtasb_vned_mps.txt" --> to plot ground speed final error
     * - "versus_vtasb_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * better airspeed sensors (TAS, AOA, & AOS), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_vtasb_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_vtasb_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * better airspeed sensors (TAS, AOA, & AOS), and another employing worse.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_vtasb_alter_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_vtasb_alter_euler_table.tex" --> script to directly generate table in Latex
     * NOTE: The two extra seeds modify the results for the worse sensors by copying the ones from the baseline
     * and multiplying them by 1.1. This happens because filter becomes unstable in these conditions, getting much
     * worse results, and I do not want to customize the filter for each situation. */
    void obtain_euler_vtasb_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end,
                                          unsigned short seed1, unsigned short seed2);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * better airspeed sensors (TAS, AOA, & AOS), and another employing worse ones.
     * - "versus_vtasb_alter_vned_mps.txt" --> to plot ground speed final error
     * - "versus_vtasb_alter_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_vtasb_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * better airspeed sensors (TAS, AOA, & AOS), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_vtasb_alter_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_vtasb_alter_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_vtasb_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);
};

} // closes namespace test
} // closes namespace nav

#endif


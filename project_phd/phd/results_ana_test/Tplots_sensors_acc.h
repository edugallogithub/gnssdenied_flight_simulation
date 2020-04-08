#ifndef RESULTS_ANA_TEST_PLOTS_ACC_SENSORS
#define RESULTS_ANA_TEST_PLOTS_ACC_SENSORS

#include "results_ana_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_sensors_acc {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_sensors_acc(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * better accelerometers (ACC), and another employing worse.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_acc_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_acc_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_acc_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * better accelerometers (ACC), and another employing worse ones.
     * - "versus_acc_vned_mps.txt" --> to plot ground speed final error
     * - "versus_acc_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_acc_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * better accelerometers (ACC), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_acc_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_acc_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_acc_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * better accelerometers (ACC), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_acc_alter_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_acc_alter_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_acc_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * better accelerometers (ACC), and another employing worse ones.
     * - "versus_acc_alter_vned_mps.txt" --> to plot ground speed final error
     * - "versus_acc_alter_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_acc_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * better accelerometers (ACC), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_acc_alter_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_acc_alter_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_acc_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);


};

} // closes namespace test
} // closes namespace nav

#endif


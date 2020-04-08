#ifndef RESULTS_ANA_TEST_PLOTS_INIT
#define RESULTS_ANA_TEST_PLOTS_INIT

#include "results_ana_test.h"
#include <string>

namespace nav {
namespace test {
	
class Tplots_init {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_init(unsigned short& seed_init, unsigned short& seed_end);

    /**< execute tests and write results on console */
	void run();

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * more accurate body attitude initial conditions, and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_att_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_init_att_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_init_att_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * more accurate body attitude initial conditions, and another employing worse ones.
     * - "versus_init_att_vned_mps.txt" --> to plot ground speed final error
     * - "versus_init_att_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_init_att_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * more accurate body attitude initial conditions, and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_att_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_init_att_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_init_att_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from two sets of files, the baseline and one employing
     * worse gyroscope bias initialization.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_gyr_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_init_gyr_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_init_gyr_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from two sets of files, the baseline and one employing
     * worse gyroscope bias initialization.
     * - "versus_init_gyr_vned_mps.txt" --> to plot ground speed final error
     * - "versus_init_gyr_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_init_gyr_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from two sets of files, the baseline and one employing
     * worse gyroscope bias initialization.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_gyr_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_init_gyr_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_init_gyr_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from two sets of files, the baseline and one employing
     * worse accelerometer bias initialization.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_acc_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_init_acc_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_init_acc_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from two sets of files, the baseline and one employing
     * worse accelerometer bias initialization.
     * - "versus_init_acc_vned_mps.txt" --> to plot ground speed final error
     * - "versus_init_acc_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_init_acc_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from two sets of files, the baseline and one employing
     * worse accelerometer bias initialization.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_acc_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_init_acc_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_init_acc_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from two sets of files, the baseline and one employing
     * worse magnetometer bias initialization.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_mag_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_init_mag_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_init_mag_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from two sets of files, the baseline and one employing
     * worse magnetometer bias initialization.
     * - "versus_init_mag_vned_mps.txt" --> to plot ground speed final error
     * - "versus_init_mag_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_init_mag_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from two sets of files, the baseline and one employing
     * worse magnetometer bias initialization.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_mag_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_init_mag_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_init_mag_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from two sets of files, the baseline and one employing
     * worse magnetic deviations initialization.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_mgn_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_init_mgn_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_init_mgn_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from two sets of files, the baseline and one employing
     * worse magnetic deviations initialization.
     * - "versus_init_mgn_vned_mps.txt" --> to plot ground speed final error
     * - "versus_init_mgn_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_init_mgn_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from two sets of files, the baseline and one employing
     * worse magnetic deviations initialization.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_init_mgn_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_init_mgn_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_init_mgn_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);




};

} // closes namespace test
} // closes namespace nav

#endif


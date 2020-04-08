#ifndef RESULTS_ANA_TEST_PLOTS_MAG_SENSORS
#define RESULTS_ANA_TEST_PLOTS_MAG_SENSORS

#include "results_ana_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_sensors_mag {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_sensors_mag(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * better magnetometers (MAG), and another employing worse.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_mag_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_mag_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_mag_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * better magnetometers (MAG), and another employing worse ones.
     * - "versus_mag_vned_mps.txt" --> to plot ground speed final error
     * - "versus_mag_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_mag_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * better magnetometers (MAG), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_mag_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_mag_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_mag_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * better magnetometers (MAG), and another employing worse.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_mag_alter_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_mag_alter_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_mag_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * better magnetometers (MAG), and another employing worse ones.
     * - "versus_mag_alter_vned_mps.txt" --> to plot ground speed final error
     * - "versus_mag_alter_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_mag_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * better magnetometers (MAG), and another employing worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_mag_alter_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_mag_alter_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_mag_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);
};

} // closes namespace test
} // closes namespace nav

#endif


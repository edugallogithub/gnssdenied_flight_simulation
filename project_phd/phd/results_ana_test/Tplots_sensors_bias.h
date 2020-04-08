#ifndef RESULTS_ANA_TEST_PLOTS_BIAS_SENSORS
#define RESULTS_ANA_TEST_PLOTS_BIAS_SENSORS

#include "results_ana_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_sensors_bias {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_sensors_bias(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * worse (from bias stability range point of view) gyroscopes and accelerometers, and another employing even worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_bias_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_bias_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_bias_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * worse (from bias stability range point of view) gyroscopes and accelerometers, and another employing even worse ones.
     * - "versus_bias_vned_mps.txt" --> to plot ground speed final error
     * - "versus_bias_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_bias_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * worse (from bias stability range point of view) gyroscopes and accelerometers, and another employing even worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_bias_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_bias_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_bias_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the total gyroscope error estimation results from a single file and generates the file
     * "versus_bias_att_single_Egyr_dps.txt" (to be added to thesis, not Matlab) */
    void obtain_Egyr_bias_single(const std::string& st_folder);

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * worse (from bias stability range point of view) gyroscopes and accelerometers, and another employing even worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_bias_alter_euler_deg.txt" --> to plot euler mean and std variation with time
     * - "versus_bias_alter_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_bias_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from three sets of files, the baseline, one employing
     * worse (from bias stability range point of view) gyroscopes and accelerometers, and another employing even worse ones.
     * - "versus_bias_alter_vned_mps.txt" --> to plot ground speed final error
     * - "versus_bias_alter_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_bias_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final horizontal position results from three sets of files, the baseline, one employing
     * worse (from bias stability range point of view) gyroscopes and accelerometers, and another employing even worse ones.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_bias_alter_pos_hor_m_pc.txt" --> to plot horizontal error mean and std variation with time
     * - "versus_bias_alter_pos_h.tex" --> script to directly generate table in Latex */
    void obtain_xhor_bias_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);



};

} // closes namespace test
} // closes namespace nav

#endif


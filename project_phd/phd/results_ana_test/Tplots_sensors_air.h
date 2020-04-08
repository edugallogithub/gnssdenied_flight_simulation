#ifndef RESULTS_ANA_TEST_PLOTS_AIR_SENSORS
#define RESULTS_ANA_TEST_PLOTS_AIR_SENSORS

#include "results_ana_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_sensors_air {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_sensors_air(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< reads the air data estimation results from three sets of files, the baseline, one employing
     * worse air data sensors (OSP & OAT), and another employing even worse sensors.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_air_T_Hp_DeltaT_table.tex" --> script to directly generate table in Latex */
    void obtain_other_air_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the body attitude estimation results from three sets of files, the baseline, one employing
     * worse air data sensors (OSP & OAT), and another employing even worse sensors.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_air_euler_table.tex" --> script to directly generate table in Latex */
    void obtain_euler_air_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the final altitude estimation results from three sets of files, the baseline, one employing
     * worse air data sensors (OSP & OAT), and another employing even worse sensors.
     * It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "versus_air_h_m.txt" --> to plot altitude mean and std variation with time
     * - "versus_air_h.tex" --> script to directly generate table in Latex */
    void obtain_h_air_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);


};



} // closes namespace test
} // closes namespace nav

#endif


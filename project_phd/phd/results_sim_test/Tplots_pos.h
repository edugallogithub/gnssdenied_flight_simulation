#ifndef RESULTS_SIM_TEST_PLOTS_POS
#define RESULTS_SIM_TEST_PLOTS_POS

#include "results_sim_test.h"
#include "ang/rotate/euler.h"
#include "acft/sens/logic.h"

namespace nav {
namespace test {
	
class Tplots_pos {
private:
    unsigned short _seed_init;
    unsigned short _seed_end;
public:
	/**< constructor based on counter */
	explicit Tplots_pos(unsigned short& seed_init, unsigned short& seed_end);

	/**< execute tests and write results on console */
	void run();

    /**< reads the vertical position (altitude) estimation results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_pos_h_m.txt" --> to plot altitude final error
     * - "error_filter_pos_h_table.tex" --> script to directly generate table in Latex
     * - "error_filter_pos_h_compare_table.tex" --> script to directly generate comparison table in Latex */
    void obtain_h_metrics(const std::string& st_first_folder, const std::string& st_first_folder_gnss, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_pos_vned_mps.txt" --> to plot ground speed final error
     * - "error_filter_pos_vned_table.tex" --> script to directly generate table in Latex
     * - "error_filter_pos_vned_compare_table.tex" --> script to directly generate comparison table in Latex */
    void obtain_vn_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal position results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_pos_hor_m_pc.txt" --> to plot horizontal final error
     * - "error_filter_pos_hor_m.txt" --> to plot lateral tracks
     * - "error_filter_pos_hor_table.tex" --> script to directly generate table in Latex
     * - "error_filter_pos_hor_compare_table.tex" --> script to directly generate comparison table in Latex */
    void obtain_xhor_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the vertical position estimation results from a single file and generates the file
     * "error_filter_pos_single_h_m.txt" (to be added to thesis, not Matlab) */
    void obtain_h_single(const std::string& st_folder);

    /**< reads the East ground velocity estimation results from a single file and generates the file
     * "error_filter_pos_single_vnii_mps.txt" (to be added to thesis, not Matlab) */
    void obtain_vn_single(const std::string& st_folder);


    /**< reads the vertical position (altitude) estimation results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_pos_alter_h_m.txt" --> to plot altitude final error
     * - "error_filter_pos_alter_h_table.tex" --> script to directly generate table in Latex */
    void obtain_h_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal ground velocity estimation results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_pos_alter_vned_mps.txt" --> to plot ground speed final error
     * - "error_filter_pos_alter_vned_table.tex" --> script to directly generate table in Latex */
    void obtain_vn_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);

    /**< reads the horizontal position results from a group of files (those between the initial and final seeds) for the
     * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
     * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
     * - "error_filter_pos_alter_hor_m_pc.txt" --> to plot horizontal final error
     * - "error_filter_pos_alter_hor_m.txt" --> to plot lateral tracks
     * - "error_filter_pos_alter_hor_table.tex" --> script to directly generate table in Latex */
    void obtain_xhor_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end);




};

} // closes namespace test
} // closes namespace nav

#endif


#ifndef ACFT_TEST_APM_PLOTS
#define ACFT_TEST_APM_PLOTS

#include "acft_test.h"
#include "jail/unit_test.h"

/*
This file contains tests to generate the PhD APM plots.
*/

namespace acft {
    namespace test {

        class Tapm_plots: public ::jail::unit_test {
        public:
            /**< constructor based on counter */
            explicit Tapm_plots(jail::counter&);
            /**< execute tests and write results on console */
            void run() override;
            /**< specific tests */

            void test1_cl_cd_cmm_alpha__deltaE();
            void test2_cl_cd_cmm_alpha__beta();
            void test3_cy_cml_cmn_beta__deltaR();
            void test4_cy_cml_cmn_beta__deltaA();
            void test5_cmn_alpha__deltaR();
            void test6_cml_alpha__deltaA();
            void test7_P_F_Hp__deltaT();
            void test8_ct_cp_eta__J();
            void test9_n_T_M_P__vtas();

        };

    } // closes namespace test

} // closes namespace acft

#endif


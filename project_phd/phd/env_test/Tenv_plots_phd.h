#ifndef ENV_TEST_PLOTS_PHD
#define ENV_TEST_PLOTS_PHD

#include "env_test.h"
#include "jail/unit_test.h"

/*
This file contains tests to generate the PhD envENV plots.
*/

namespace env {
    namespace test {

        class Tenv_plots_phd: public ::jail::unit_test {
        public:
            /**< constructor based on counter */
            explicit Tenv_plots_phd(jail::counter&);
            /**< execute tests and write results on console */
            void run() override;
            /**< specific tests */

            void test_Tp_Hp__DeltaT();
            void test_H_h__phi();
            void test_H_vs_Hp__DeltaT();

        };

    } // closes namespace test

} // closes namespace env

#endif


#ifndef ATT_TEST_INTEGRATION
#define ATT_TEST_INTEGRATION

#include "ang_test.h"
#include "ang/quat.h"
#include "ang/rotate/rodrigues.h"
#include <jail/unit_test.h>
#include <Eigen/Core>
#include <vector>

/*
Test of integration of the motion of a rigid body with the different rotation
and transformation methods.
01 --> space and rodrigues
02 --> space and dcm
03 --> space and rotv
11 --> body  and rodrigues
12 --> body  and dcm
13 --> body  and rotv
*/

namespace ang {
    namespace test {

        class Tintegration: public ::jail::unit_test {
        public:
            /**< constructor based on counter */
            explicit Tintegration(jail::counter&);
            /**< execute tests and write results on console */
            void run() override;

            /**< specific tests */
            void test01(std::vector<double>& res, const double& Deltat_sec, const double& t_sec_end, const Eigen::Vector3d& x0_wbw_m, const ang::rodrigues& q0_wb, const Eigen::Vector3d& v0_w_mps, const Eigen::Vector3d& w0_bww_rps, const Eigen::Vector3d& x_bpb_m, const Eigen::Vector3d& a_b_mps2, const Eigen::Vector3d& alpha_b_rps2);
            void test02(std::vector<double>& res, const double& Deltat_sec, const double& t_sec_end, const Eigen::Vector3d& x0_wbw_m, const ang::dcm& R0_wb,       const Eigen::Vector3d& v0_w_mps, const Eigen::Vector3d& w0_bww_rps, const Eigen::Vector3d& x_bpb_m, const Eigen::Vector3d& a_b_mps2, const Eigen::Vector3d& alpha_b_rps2);
            void test03(std::vector<double>& res, const double& Deltat_sec, const double& t_sec_end, const Eigen::Vector3d& x0_wbw_m, const ang::rotv& rv0_wb,     const Eigen::Vector3d& v0_w_mps, const Eigen::Vector3d& w0_bww_rps, const Eigen::Vector3d& x_bpb_m, const Eigen::Vector3d& a_b_mps2, const Eigen::Vector3d& alpha_b_rps2);
            void test11(std::vector<double>& res, const double& Deltat_sec, const double& t_sec_end, const Eigen::Vector3d& x0_wbw_m, const ang::rodrigues& q0_wb, const Eigen::Vector3d& v0_b_mps, const Eigen::Vector3d& w0_bwb_rps, const Eigen::Vector3d& x_bpb_m, const Eigen::Vector3d& a_b_mps2, const Eigen::Vector3d& alpha_b_rps2);
            void test12(std::vector<double>& res, const double& Deltat_sec, const double& t_sec_end, const Eigen::Vector3d& x0_wbw_m, const ang::dcm& R0_wb,       const Eigen::Vector3d& v0_b_mps, const Eigen::Vector3d& w0_bwb_rps, const Eigen::Vector3d& x_bpb_m, const Eigen::Vector3d& a_b_mps2, const Eigen::Vector3d& alpha_b_rps2);
            void test13(std::vector<double>& res, const double& Deltat_sec, const double& t_sec_end, const Eigen::Vector3d& x0_wbw_m, const ang::rotv& rv0_wb,     const Eigen::Vector3d& v0_b_mps, const Eigen::Vector3d& w0_bwb_rps, const Eigen::Vector3d& x_bpb_m, const Eigen::Vector3d& a_b_mps2, const Eigen::Vector3d& alpha_b_rps2);
     };

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


    } // closes namespace test
} // closes namespace ang

#endif


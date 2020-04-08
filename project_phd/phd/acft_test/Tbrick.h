#ifndef ACFT_TEST_BRICK
#define ACFT_TEST_BRICK

#include "acft_test.h"
#include "env/geo.h"
#include "ang/quat.h"

#include "jail/unit_test.h"

namespace acft {
namespace test {

class brick_motion {
private:
    static void aux(const double& t_sec,
                    env::geodetic_coord& dx_gdt_rad_m,
                    Eigen::Vector3d& dv_bfs_mps_dt,
                    ang::quat& dq_nedbfs_dt,
                    Eigen::Vector3d& dh_bfsirsbfs_Nms_dt,
                    const env::geodetic_coord& x_gdt_rad_m,
                    const Eigen::Vector3d& v_bfs_mps,
                    const ang::rodrigues& q_nedbfs,
                    const Eigen::Vector3d& h_bfsirsbfs_Nms,
                    const env::geo& Ogeo,
                    const Eigen::Matrix3d& I_kgm2);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< integrates the motion of a brick for the input time */
    static std::vector<double> solve(double t_sec_end);

}; // closes class brick_motion

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

class Tbrick: public ::jail::unit_test {
public:
	/**< constructor based on counter */
	explicit Tbrick(jail::counter&);
	/**< execute tests and write results on console */
	void run() override;
	/**< specific tests */
	void test_brick_motion();

};

} // closes namespace test

} // closes namespace acft
#endif


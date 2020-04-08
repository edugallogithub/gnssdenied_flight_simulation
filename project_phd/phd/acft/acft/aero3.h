#ifndef ACFT_AERO3
#define ACFT_AERO3

#include "../acft.h"
#include "aero.h"

/*
 */

namespace acft {

// CLASS AERO3
// ===========
// ===========

class ACFT_API aero3 : public aero {
private:
    /**< name required internally */
    static const std::string _filename;
    /**< amount to add to force coefficient along xBFS */
    double _cf_bfsi_extra;
    /**< differential of force coefficient in BFS with respect to
    angular velocity of BFS over NED expressed in BFS */
    Eigen::Matrix3d _dcf_bfs_dw_nedbfsbfs;
    /**< differential of moment coefficient in BFS with respect to
    angular velocity of BFS over NED expressed in BFS */
    Eigen::Matrix3d _dcm_bfs_dw_nedbfsbfs;
    /**< differential of force coefficient in BFS with respect to control deflections */
    Eigen::Matrix3d _dcf_bfs_dsurf_deg;
    /**< differential of moment coefficient in BFS with respect to control deflections */
    Eigen::Matrix3d _dcm_bfs_dsurf_deg;
    /**< base force coefficient in BFS */
    Eigen::Vector3d _cf_bfs;
    /**< base moment coefficient in BFS */
    Eigen::Vector3d _cm_bfs;
    /**< differential of force coefficient in BFS with respect to sideslip */
    Eigen::Vector3d _dcf_bfs_dbeta_rad;
    /**< differential of moment coefficient in BFS with respect to sideslip */
    Eigen::Vector3d _dcm_bfs_dbeta_rad;
    /**< differential of force coefficient in BFS with respect to elevator control parameter */
    Eigen::Vector3d _dcf_bfs_ddeltaE_deg;
    /**< differential of moment coefficient in BFS with respect to elevator control parameter */
    Eigen::Vector3d _dcm_bfs_ddeltaE_deg;
    /**< differential of force coefficient in BFS with respect to angle of attack */
    Eigen::Vector3d _dcf_bfs_dalpha_rad;
    /**< differential of moment coefficient in BFS with respect to angle of attack */
    Eigen::Vector3d _dcm_bfs_dalpha_rad;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< empty constructor */
    aero3();
    /**< copy constructor */
    aero3(const aero3&) = delete;
    /**< move constructor */
    aero3(aero3&&) = delete;
    /**< destructor */
    ~aero3() override = default;
    /**< copy assignment */
    aero3& operator=(const aero3&) = delete;
    /**< move assignment */
    aero3& operator=(aero3&&) = delete;

    /**< computes the aerodynamic forces coefficient based on the true airspeed, the Euler angles
    from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
    velocity of BFS over NED expressed in BFS. */
    Eigen::Vector3d obtain_cf_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const;
    /**< computes the aerodynamics moments coefficient based on the true airspeed, the Euler angles
    from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
    velocity of BFS over NED expressed in BFS. */
    Eigen::Vector3d obtain_cm_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const;
}; // closes class aero3

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

} // closes namespace acft

#endif


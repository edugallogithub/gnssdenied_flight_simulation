#ifndef ACFT_AERO0
#define ACFT_AERO0

#include "../acft.h"
#include "aero.h"
#include "ang/rotate/euler.h"
#include "math/vec/vec1.h"
#include "math/pred/pred3v/f_table3V.h"
#include <string>

/*
 */

namespace acft {

// CLASS AERO0
// ===========
// ===========

class ACFT_API aero0 : public aero {
private:
    /**< name required internally */
    static const std::string _filename;
    /**< vector of angle of attack positions */
    math::vec1 _vec_alpha_rad;
    /**< vector of elevator positions */
    math::vec1 _vec_deltaE_deg;
    /**< vector of angle of sideslip positions */
    math::vec1 _vec_beta_rad;
    /**< vector of aileron positions */
    math::vec1 _vec_deltaA_deg;
    /**< vector of rudder positions */
    math::vec1 _vec_deltaR_deg;

    /**< amount to add to force coefficient along xBFS */
    double _cf_bfsi_extra;

    /**< force coefficient */
    math::f_table3V* _Pfun_cf_bfsi;
    math::f_table3V* _Pfun_cf_bfsii;
    math::f_table3V* _Pfun_CF_bfsii;
    math::f_table3V* _Pfun_cf_bfsiii;

    /**< force coefficient relationship with wi_bfsned_bfs */
    math::f_table3V* _Pfun_dcf_bfsi_dwi_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcf_bfsii_dwi_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCF_bfsii_dwi_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcf_bfsiii_dwi_bfsnedbfs_rps;

    /**< force coefficient relationship with wii_bfsned_bfs */
    math::f_table3V* _Pfun_dcf_bfsi_dwii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcf_bfsii_dwii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCF_bfsii_dwii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcf_bfsiii_dwii_bfsnedbfs_rps;

    /**< force coefficient relationship with wiii_bfsned_bfs */
    math::f_table3V* _Pfun_dcf_bfsi_dwiii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcf_bfsii_dwiii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCF_bfsii_dwiii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcf_bfsiii_dwiii_bfsnedbfs_rps;

    /**< force coefficient relationship with surfi_deg */
    math::f_table3V* _Pfun_dcf_bfsi_dsurfi_deg;
    math::f_table3V* _Pfun_dcf_bfsii_dsurfi_deg;
    math::f_table3V* _Pfun_dCF_bfsii_dsurfi_deg;
    math::f_table3V* _Pfun_dcf_bfsiii_dsurfi_deg;

    /**< force coefficient relationship with surfii_deg */
    math::f_table3V* _Pfun_dcf_bfsi_dsurfii_deg;
    math::f_table3V* _Pfun_dcf_bfsii_dsurfii_deg;
    math::f_table3V* _Pfun_dCF_bfsii_dsurfii_deg;
    math::f_table3V* _Pfun_dcf_bfsiii_dsurfii_deg;

    /**< force coefficient relationship with surfiii_deg */
    math::f_table3V* _Pfun_dcf_bfsi_dsurfiii_deg;
    math::f_table3V* _Pfun_dcf_bfsii_dsurfiii_deg;
    math::f_table3V* _Pfun_dCF_bfsii_dsurfiii_deg;
    math::f_table3V* _Pfun_dcf_bfsiii_dsurfiii_deg;

    /**< force coefficient relationship with alpha_rad */
    math::f_table3V* _Pfun_dCF_bfsii_dalpha_rad;

    /**< moment coefficient */
    math::f_table3V* _Pfun_cm_bfsi;
    math::f_table3V* _Pfun_CM_bfsi;
    math::f_table3V* _Pfun_cm_bfsii;
    math::f_table3V* _Pfun_cm_bfsiii;
    math::f_table3V* _Pfun_CM_bfsiii;

    /**< moment coefficient relationship with wi_bfsned_bfs */
    math::f_table3V* _Pfun_dcm_bfsi_dwi_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCM_bfsi_dwi_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcm_bfsii_dwi_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcm_bfsiii_dwi_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCM_bfsiii_dwi_bfsnedbfs_rps;

    /**< moment coefficient relationship with wii_bfsned_bfs */
    math::f_table3V* _Pfun_dcm_bfsi_dwii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCM_bfsi_dwii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcm_bfsii_dwii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcm_bfsiii_dwii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCM_bfsiii_dwii_bfsnedbfs_rps;

    /**< moment coefficient relationship with wiii_bfsned_bfs */
    math::f_table3V* _Pfun_dcm_bfsi_dwiii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCM_bfsi_dwiii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcm_bfsii_dwiii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dcm_bfsiii_dwiii_bfsnedbfs_rps;
    math::f_table3V* _Pfun_dCM_bfsiii_dwiii_bfsnedbfs_rps;

    /**< moment coefficient relationship with surfi_deg */
    math::f_table3V* _Pfun_dcm_bfsi_dsurfi_deg;
    math::f_table3V* _Pfun_dCM_bfsi_dsurfi_deg;
    math::f_table3V* _Pfun_dcm_bfsii_dsurfi_deg;
    math::f_table3V* _Pfun_dcm_bfsiii_dsurfi_deg;
    math::f_table3V* _Pfun_dCM_bfsiii_dsurfi_deg;

    /**< moment coefficient relationship with surfii_deg */
    math::f_table3V* _Pfun_dcm_bfsi_dsurfii_deg;
    math::f_table3V* _Pfun_dCM_bfsi_dsurfii_deg;
    math::f_table3V* _Pfun_dcm_bfsii_dsurfii_deg;
    math::f_table3V* _Pfun_dcm_bfsiii_dsurfii_deg;
    math::f_table3V* _Pfun_dCM_bfsiii_dsurfii_deg;

    /**< moment coefficient relationship with surfiii_deg */
    math::f_table3V* _Pfun_dcm_bfsi_dsurfiii_deg;
    math::f_table3V* _Pfun_dCM_bfsi_dsurfiii_deg;
    math::f_table3V* _Pfun_dcm_bfsii_dsurfiii_deg;
    math::f_table3V* _Pfun_dcm_bfsiii_dsurfiii_deg;
    math::f_table3V* _Pfun_dCM_bfsiii_dsurfiii_deg;

    /**< moment coefficient relationship with alpha_rad */
    math::f_table3V* _Pfun_dCM_bfsi_dalpha_rad;
    math::f_table3V* _Pfun_dCM_bfsiii_dalpha_rad;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< constructor based on interpolation method */
    aero0(math::logic::INTERP_MODE interp_mode = math::logic::biparabolic);
    /**< copy constructor */
    aero0(const aero0&) = delete;
    /**< move constructor */
    aero0(aero0&&) = delete;
    /**< destructor */
    ~aero0() override;
    /**< copy assignment */
    aero0& operator=(const aero0&) = delete;
    /**< move assignment */
    aero0& operator=(aero0&&) = delete;

    /**< computes the aerodynamic forces coefficient based on the true airspeed, the Euler angles
    from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
    velocity of BFS over NED expressed in BFS. */
    Eigen::Vector3d obtain_cf_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const;
    /**< computes the aerodynamics moments coefficient based on the true airspeed, the Euler angles
    from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
    velocity of BFS over NED expressed in BFS. */
    Eigen::Vector3d obtain_cm_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const;
}; // closes class aero0

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

} // closes namespace acft

#endif


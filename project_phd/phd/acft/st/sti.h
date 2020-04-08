#ifndef ACFT_STI
#define ACFT_STI

#include "../acft.h"
#include "../st/trj_truth.h"
#include "../st/trj_cntr.h"
#include "math/math/func.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace env {
    class earth;
}
namespace control {
    class guid;
}
namespace st {

    namespace logic {
        /**< enumeration that contains the different starting points */
        enum STI_ID {
            sti_h3000_tas25_psi00  = 0,  // h [m] 3000.0, tas [mps] 25.0, psi [deg]   0.0
            sti_h3000_tas30_psi00  = 1,  // h [m] 3000.0, tas [mps] 30.0, psi [deg]   0.0
            sti_h3000_tas30_psi30  = 2,  // h [m] 3000.0, tas [mps] 30.0, psi [deg]  30.0
            sti_Hp3000_tas30_psi00 = 3,  // Hp [m] 3000.0, tas [mps] 30.0, psi [deg]  0.0
            sti_Hp3000_tas30_psi90 = 4,  // Hp [m] 3000.0, tas [mps] 30.0, psi [deg] 90.0
            sti_Hp3000_tas30_chi00 = 5,  // Hp [m] 3000.0, tas [mps] 30.0, chi [deg]  0.0
            sti_Hp3000_tas30_chi90 = 6,  // Hp [m] 3000.0, tas [mps] 30.0, chi [deg] 90.0
            sti_Hp3000_tas30_chiXX = 7,  // Hp [m] 3000.0, tas [mps] 30.0, chi [deg] from guidance
            sti_Hp3000_tas30_psiXX = 8,  // Hp [m] 3000.0, tas [mps] 30.0, psi [deg] from guidance
            sti_HpXX_tasXX_chiXX   = 9,  // Hp [m], tas [mps], and chi [deg] from guidance
            sti_HpXX_tasXX_psiXX   = 10, // Hp [m], tas [mps], and psi [deg] from guidance
            sti_moses              = 11,
            sti_moses2             = 12,
            sti_moses3             = 13,
            sti_rozas1             = 14,
            sti_size               = 15
        };
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS INITIAL CONDITIONS STI
// ============================
// ============================

class ACFT_API sti {
protected:
    /**< initial time [sec] */
    double _t_sec;
    /**< initial mass [kg] */
    double _m_kg;
    /**< initial true airspeed [mps] */
    double _vtas_mps;

    /**< initial longitude [deg-rad] */
    double _lambda_deg;
    double _lambda_rad;
    /**< initial latitude [deg-rad] */
    double _phi_deg;
    double _phi_rad;
    /**< initial altitude [m] */
    double _h_m;
    /**< initial pressure altitude [m] */
    double _Hp_m;

    /**< initial body yaw angle [deg-rad] */
    double _psi_deg;
    double _psi_rad;
    /**< initial body pitch angle [deg-rad] */
    double _theta_deg;
    double _theta_rad;
    /**< initial body bank angle [deg-rad] */
    double _xi_deg;
    double _xi_rad;
    /**< initial ground speed yaw angle [deg-rad] */
    double _chi_deg;
    double _chi_rad;
    /**< initial ground pitch angle [deg-rad] */
    double _gamma_deg;
    double _gamma_rad;

    /**< initial angle of attack [deg-rad] */
    double _alpha_deg;
    double _alpha_rad;
    /**< initial sideslip angle [deg-rad] */
    double _beta_deg;
    double _beta_rad;
    /**< initial aircraft angular velocity [rps] */
    Eigen::Vector3d _w_nbb_rps;
    /**< initial ground altitude [m] */
    double _hground_m;

    /**< initial control parameters (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d _delta_control;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    sti() = default;
    /**< copy constructor */
    sti(const sti&);
    /**< move constructor */
    sti(sti&&) = delete;
    /**< destructor */
    ~sti() = default;
    /**< copy assignment */
    sti& operator=(const sti&) = delete;
    /**< move assignment */
    sti& operator=(sti&&) = delete;

    /**< creates initial conditions based on guidance and initial time */
    static sti* create_sti(const control::guid& Oguid, const double& t_sec_init);
    /**< fill up state vector with initial conditions */
    virtual void complete_and_fill_up_st_truth(st::st_truth& Ost_init, const env::earth& Oearth) = 0;

    /**< add time, mass, true airspeed, angle of attack, angle of sideslip, aircraft rotation speed, and ground altitude */
    virtual void complete_misc(const double& t_sec, const double& m_kg, const double& vtas_mps, const double& alpha_deg, const double& beta_deg, const Eigen::Vector3d& w_nbb_rps, const double& hground_m);
    /**< add control parameters (throttle, elevator, ailerons, rudder) */
    virtual void complete_control(const double& delta_thr, const double& delta_elv, const double& delta_ail, const double& delta_rud);

    /**< get initial time */
    virtual const double& get_t_sec() const {return _t_sec;}
    /**< get initial mass [kg] */
    virtual const double& get_m_kg() const {return _m_kg;}
    /**< get initial true airspeed */
    virtual const double& get_vtas_mps() const {return _vtas_mps;}

    /**< get initial longitude [deg-rad] */
    virtual const double& get_lambda_deg() const {return _lambda_deg;}
    virtual const double& get_lambda_rad() const {return _lambda_rad;}
    /**< get initial latitude [deg-rad] */
    virtual const double& get_phi_deg() const {return _phi_deg;}
    virtual const double& get_phi_rad() const {return _phi_rad;}
    /**< get initial altitude [m] */
    const double& get_h_m() const {return _h_m;}
    /**< get initial pressure altitude [m] */
    const double& get_Hp_m() const {return _Hp_m;}

    /**< get initial body yaw angle [deg-rad] */
    const double& get_psi_deg() const {return _psi_deg;}
    const double& get_psi_rad() const {return _psi_rad;}
    /**< get initial body pitch angle [deg-rad] */
    virtual const double& get_theta_deg() const {return _theta_deg;}
    virtual const double& get_theta_rad() const {return _theta_rad;}
    /**< get initial body bank angle [deg-rad] */
    virtual const double& get_xi_deg() const {return _xi_deg;}
    virtual const double& get_xi_rad() const {return _xi_rad;}
    /**< get initial ground speed yaw angle [deg-rad] */
    const double& get_chi_deg() const {return _chi_deg;}
    const double& get_chi_rad() const {return _chi_rad;}
    /**< get initial ground pitch angle [deg-rad] */
    virtual const double& get_gamma_deg() const {return _gamma_deg;}
    virtual const double& get_gamma_rad() const {return _gamma_rad;}

    /**< get initial angle of attack [deg-rad] */
    virtual const double& get_alpha_deg() const {return _alpha_deg;}
    virtual const double& get_alpha_rad() const {return _alpha_rad;}
    /**< get initial sideslip angle [deg-rad] */
    virtual const double& get_beta_deg() const {return _beta_deg;}
    virtual const double& get_beta_rad() const {return _beta_rad;}
    /**< get initial aircraft angular velocity [rps] */
    virtual const Eigen::Vector3d& get_w_nbb_rps() const {return _w_nbb_rps;}
    /**< get initial ground altitude [m] */
    virtual const double& get_hground_m() const {return _hground_m;}

    /**< get initial control parameters (throttle, elevator, ailerons, rudder) */
    virtual const Eigen::Array4d& get_delta_control() const {return _delta_control;}
}; // closes class sti

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS INITIAL CONDITIONS STI_h_psi
// ==================================
// ==================================

class ACFT_API sti_h_psi : public sti {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    sti_h_psi() = default;
    /**< copy constructor */
    sti_h_psi(const sti_h_psi&);
    /**< move constructor */
    sti_h_psi(sti_h_psi&&) = delete;
    /**< destructor */
    ~sti_h_psi() = default;
    /**< copy assignment */
    sti_h_psi& operator=(const sti_h_psi&) = delete;
    /**< move assignment */
    sti_h_psi& operator=(sti_h_psi&&) = delete;

    /**< add geodetic position (with geometric altitude) */
    void complete_lambda_phi_h(const double& lambda_deg, const double& phi_deg, const double& h_m);
    /**< add body yaw, body pitch, body roll */
    void complete_psi_theta_xi(const double& psi_deg, const double& theta_deg, const double& xi_deg);
    /**< fill up state vector with initial conditions */
    void complete_and_fill_up_st_truth(st::st_truth& Ost_init, const env::earth& Oearth) override;
}; // closes class sti_h_psi

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS INITIAL CONDITIONS STI_h_chi
// ==================================
// ==================================

class ACFT_API sti_h_chi : public sti {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    sti_h_chi() = default;
    /**< copy constructor */
    sti_h_chi(const sti_h_chi&);
    /**< move constructor */
    sti_h_chi(sti_h_chi&&) = delete;
    /**< destructor */
    ~sti_h_chi() = default;
    /**< copy assignment */
    sti_h_chi& operator=(const sti_h_chi&) = delete;
    /**< move assignment */
    sti_h_chi& operator=(sti_h_chi&&) = delete;

    /**< add geodetic position */
    void complete_lambda_phi_h(const double& lambda_deg, const double& phi_deg, const double& h_m);
    /**< add ground speed yaw, body pitch, body roll */
    void complete_chi_theta_xi(const double& chi_deg, const double& theta_deg, const double& xi_deg);
    /**< fill up state vector with initial conditions */
    void complete_and_fill_up_st_truth(st::st_truth& Ost_init, const env::earth& Oearth) override;
private:
    /**< function that returns difference between input body bearing psi and what can be computed from rest of inputs.
     * Difference should be zero. */
    static double compute_psi_diff(const double& psi_rad, const double& chi_rad, const double& theta_rad, const double& xi_rad,
                                   const double& vtas_bi_mps, const double& vtas_bii_mps, const double& vtas_biii_mps,
                                   const double& vwind_ni_mps, const double& vwind_nii_mps, const double& vwind_niii_mps,
                                   const double& vturb_bi_mps, const double& vturb_bii_mps, const double& vturb_biii_mps);
    /**< encapsulates minimization function to obtain body bearing psi */
    class psi_u : public math::func {
    public:
        double exec(const double& psi_rad, const std::vector<double>& par) override;
    };
}; // closes class sti_h_chi

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS INITIAL CONDITIONS STI_Hp_chi
// ===================================
// ===================================

class ACFT_API sti_Hp_chi : public sti {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    sti_Hp_chi() = default;
    /**< copy constructor */
    sti_Hp_chi(const sti_Hp_chi&);
    /**< move constructor */
    sti_Hp_chi(sti_Hp_chi&&) = delete;
    /**< destructor */
    ~sti_Hp_chi() = default;
    /**< copy assignment */
    sti_Hp_chi& operator=(const sti_Hp_chi&) = delete;
    /**< move assignment */
    sti_Hp_chi& operator=(sti_Hp_chi&&) = delete;

    /**< add pressure geodetic position (based on pressure altitude) */
    void complete_lambda_phi_Hp(const double& lambda_deg, const double& phi_deg, const double& Hp_m);
    /**< add ground speed yaw, body pitch, body roll */
    void complete_chi_theta_xi(const double& chi_deg, const double& theta_deg, const double& xi_deg);
    /**< fill up state vector with initial conditions */
    void complete_and_fill_up_st_truth(st::st_truth& Ost_init, const env::earth& Oearth) override;
private:
    /**< function that returns difference between input body bearing psi and what can be computed from rest of inputs.
     * Difference should be zero. */
    static double compute_psi_diff(const double& psi_rad, const double& chi_rad, const double& theta_rad, const double& xi_rad,
                                   const double& vtas_bi_mps, const double& vtas_bii_mps, const double& vtas_biii_mps,
                                   const double& vwind_ni_mps, const double& vwind_nii_mps, const double& vwind_niii_mps,
                                   const double& vturb_bi_mps, const double& vturb_bii_mps, const double& vturb_biii_mps);
    /**< encapsulates minimization function to obtain body bearing psi */
    class psi_u : public math::func {
    public:
        double exec(const double& psi_rad, const std::vector<double>& par) override;
    };
}; // closes class sti_Hp_chi

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS INITIAL CONDITIONS STI_Hp_psi
// ===================================
// ===================================

class ACFT_API sti_Hp_psi : public sti {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    sti_Hp_psi() = default;
    /**< copy constructor */
    sti_Hp_psi(const sti_Hp_psi&);
    /**< move constructor */
    sti_Hp_psi(sti_Hp_psi&&) = delete;
    /**< destructor */
    ~sti_Hp_psi() = default;
    /**< copy assignment */
    sti_Hp_psi& operator=(const sti_Hp_psi&) = delete;
    /**< move assignment */
    sti_Hp_psi& operator=(sti_Hp_psi&&) = delete;

    /**< add pressure geodetic position (based on pressure altitude) */
    void complete_lambda_phi_Hp(const double& lambda_deg, const double& phi_deg, const double& Hp_m);
    /**< add body yaw, body pitch, body roll */
    void complete_psi_theta_xi(const double& psi_deg, const double& theta_deg, const double& xi_deg);
    /**< fill up state vector with initial conditions */
    void complete_and_fill_up_st_truth(st::st_truth& Ost_init, const env::earth& Oearth) override;
}; // closes class sti_Hp_psi

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////

} // closes namespace st

#endif

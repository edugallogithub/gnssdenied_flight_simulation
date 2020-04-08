#ifndef ACFT_PLATFORM
#define ACFT_PLATFORM

#include "../acft.h"
#include "logic.h"
#include "env/coord.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/euler.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace acft {
    class iner;
}

namespace sens {

// CLASS PLATFORM
// ==============
// ==============

class ACFT_API platform {
private:
    /**< seed generator  */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;

    /**< standard deviation in each direction when measuring location of platform frame */
    double _sigma_T_rpb_m;
    /**< standard deviation of real yaw (not error) when fixing platform to aircraft */
    double _sigma_yaw_bp_deg;
    /**< standard deviation of real pitch (not error) when fixing platform to aircraft */
    double _sigma_pitch_bp_deg;
    /**< standard deviation of real bank (not error) when fixing platform to aircraft */
    double _sigma_bank_bp_deg;
    /**< standard deviation in all three Euler angles when measuring orientation of platform frame */
    double _sigma_euler_bp_deg;

    /**< true translation from aircraft reference (wing trailing edge) "r" to platform "p" viewed in body */
    Eigen::Array3d _Trpb_m_truth;
    /**< estimated translation from aircraft reference (wing trailing edge) "r" to platform "p" viewed in body */
    Eigen::Array3d _Trpb_m_est;

    /**< true euler angles from body to platform frames */
    ang::euler _euler_bp_truth;
    /**< estimated euler angles from body to platform frames */
    ang::euler _euler_bp_est;

    /**< true rotation matrix from body to platform frames */
    ang::dcm _R_bp_truth;
    /**< true rotation matrix from platform to body frames */
    ang::dcm _R_pb_truth;
    /**< estimated rotation matrix from body to platform frames */
    ang::dcm _R_bp_est;
    /**< estimated rotation matrix from platform to body frames */
    ang::dcm _R_pb_est;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    platform() = delete;
    /**< constructor based on standard deviations (longitudinal when measuring the platform location in all three axes, three Euler
     * angles when fixing platform to body, and angular - in 3 Euler angles - when measuring the platform orientation), plus
     * the seed. Flag true to show summary in console. */
    platform(const double& sigma_T_rpb_m, const double& sigma_yaw_bp_deg, const double& sigma_pitch_bp_deg, const double& sigma_bank_bp_deg, const double& sigma_euler_bp_deg, const int& acft_seed, bool flag_console = true);
    /**< copy constructor */
    platform(const platform&) = delete;
    /**< move constructor */
    platform(platform&&) = delete;
    /**< destructor */
    ~platform() = default;
    /**< copy assignment */
    platform& operator=(const platform&) = delete;
    /**< move assignment */
    platform& operator=(platform&&) = delete;

    /**< create gyroscope model. Flag true to show summary in console. */
    static sens::platform* create_platform(const int& acft_seed, bool flag_console = true);

    /**< get standard deviation in each direction when measuring location of platform frame */
    const double get_sigma_T_rpb_m() const {return _sigma_T_rpb_m;}
    /**< get standard deviation of real yaw (not error) when fixing platform to aircraft */
    const double get_sigma_yaw_bp_deg() const {return _sigma_yaw_bp_deg;}
    /**< get standard deviation of real pitch (not error) when fixing platform to aircraft */
    const double get_sigma_pitch_bp_deg() const {return _sigma_pitch_bp_deg;}
    /**< get standard deviation of real bank (not error) when fixing platform to aircraft */
    const double get_sigma_bank_bp_deg() const {return _sigma_bank_bp_deg;}
    /**< get standard deviation in all three Euler angles when measuring orientation of platform frame */
    const double get_sigma_euler_bp_deg() const {return _sigma_euler_bp_deg;}


    /**< get true translation from aircraft reference (wing trailing edge) "r" to platform viewed in body */
    const Eigen::Array3d& get_Trpb_m_truth() const {return _Trpb_m_truth;}
    /**< get estimated translation from aircraft reference (wing trailing edge) "r" to platform viewed in body */
    const Eigen::Array3d& get_Trpb_m_est() const {return _Trpb_m_est;}

    /**< get true euler angles from body to platform frames */
    const ang::euler& get_euler_bp_truth() const {return _euler_bp_truth;}
    /**< get estimated euler angles from body to platform frames */
    const ang::euler& get_euler_bp_est() const {return _euler_bp_est;}

    /**< get true rotation matrix from body to platform */
    const ang::dcm get_R_bp_truth() const {return _R_bp_truth;}
    /**< get estimated rotation matrix from body to platform */
    const ang::dcm get_R_bp_est() const {return _R_bp_est;}
    /**< get true rotation matrix from platform to body */
    const ang::dcm get_R_pb_truth() const {return _R_pb_truth;}
    /**< get estimated rotation matrix from platform to body */
    const ang::dcm get_R_pb_est() const {return _R_pb_est;}

    /**< get true translation from body (center of gravity) to platform viewed in body based on mass ratio */
    Eigen::Array3d get_Tbpb_m_truth(const double& ratio, const acft::iner& Oiner) const;
    /**< get estimated translation from body (center of gravity) to platform viewed in body based on mass ratio */
    Eigen::Array3d get_Tbpb_m_est(const double& ratio, const acft::iner& Oiner) const;

    /**< describe platform model in stream */
    void create_text(std::ostream& Ostream) const;
}; // closes class platform

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace sens

#endif

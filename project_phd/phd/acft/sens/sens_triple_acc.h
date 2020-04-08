#ifndef ACFT_SENS_TRIPLE_ACC
#define ACFT_SENS_TRIPLE_ACC

#include "../acft.h"
#include "logic.h"
#include "env/coord.h"
#include "ang/rotate/dcm.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sens {
    class platform;

// CLASS SENS_TRIPLE_ACC
// =====================
// =====================

class ACFT_API sens_triple_acc {
private:
    /**< bias drift [unit / sec^0.5] */
    double _sigma_u;
    /**< white noise [unit * sec^0.5] */
	double _sigma_v;
    /**< scale factor [-] */
    double _s;
    /**< cross coupling [-] */
    double _m;
    /**< bias offset [unit] */
    double _B0;
    /**< time interval between measurements [sec] */
    double _Deltat_sec;

    /**< seed generator for scale factor and cross coupling */
    std::ranlux24_base _gen_acft;
    /**< standard normal distribution for scale factor and cross coupling */
    std::normal_distribution<double> _dist_acft;
    /**< seed generator for bias drift, bias offset and white noise */
    std::ranlux24_base _gen_run;
    /**< standard normal distribution for bias drift, bias offset and white noise */
    std::normal_distribution<double> _dist_run;

    /**< weak pointer to platform */
    const sens::platform* _Pplat;

    /**< bias drift limit in standard deviations */
    double _limit;
    /**< distance from limit at which random walk is tricked */
    double _safety;
    /**< _sigma_u * Delta_t^0.5 */
    double _sigma_u_Delta_t05;
    /**< _sigma_v / Delta_t^0.5 */
    double _sigma_v_Delta_tn05;

    /**< scale factor and cross coupling error matrix (without added identity matrix) */
    Eigen::Matrix3d _M;
    /**< scale factor and cross coupling error matrix (with added identity matrix) */
    Eigen::Matrix3d _IM;
    /**< _B0 * Nu0 */
    Eigen::Vector3d _B0Nu0;
    /**< Sum_Nui */
    Eigen::Vector3d _Sum_Nui;

    /**< modifies the input variable by adding one stop of a random walk */
    void grow_random_walk(double& y);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    sens_triple_acc() = delete;
    /**< constructor based on bias drift, white noise, scale factor, cross coupling, bias offset, time interval between measurements,
     * aircraft seed (distinguishes among different aircraft, this is, scale factor and cross coupling), and run seed
     * (distinguishes among different runs of the same aircraft, this is, bias drift, bias offset, bias drift, and
     * white noise), and platform frame. */
	sens_triple_acc(const double& sigma_u, const double& sigma_v, const double& s, const double& m, const double& B0, const double& limit, const double& Deltat_sec, const int& acft_seed, const int& run_seed, const sens::platform& Oplat);
    /**< copy constructor */
    sens_triple_acc(const sens_triple_acc&) = delete;
    /**< move constructor */
    sens_triple_acc(sens_triple_acc&&) = delete;
    /**< destructor */
    ~sens_triple_acc() = default;
    /**< copy assignment */
    sens_triple_acc& operator=(const sens_triple_acc&) = delete;
    /**< move assignment */
    sens_triple_acc& operator=(sens_triple_acc&&) = delete;

    /**< return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */
    Eigen::Vector3d eval_noB_noM_noR(const Eigen::Vector3d& f_ibb_mps2, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror);
    /**< return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */
    Eigen::Vector3d eval_noM_noR(const Eigen::Vector3d& f_ibb_mps2, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror);
    /**< return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */
    Eigen::Vector3d eval_noR(const Eigen::Vector3d& f_ibb_mps2, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror);
    /**< return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */
    Eigen::Vector3d eval_complete(const Eigen::Vector3d& f_ibb_mps2, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror,
                                  const Eigen::Vector3d& w_ibb_rps_truth, const Eigen::Vector3d& w_ibb_rps_est,
                                  const Eigen::Vector3d& T_bpb_m_truth, const Eigen::Vector3d& T_bpb_m_est,
                                  const Eigen::Vector3d& alpha_ibb_rps2_truth, const Eigen::Vector3d& alpha_ibb_rps2_est);

    /**< create accelerometer model */
    static sens::sens_triple_acc* create_accelerometer(sens::logic::ACC_ID, sens::logic::BAND_ID,
                                                       const int& acft_seed, const int& run_seed, const sens::platform& Oplat, const double& Deltat_sec);

    /**< get bias drift [unit / sec^0.5] to read */
    const double& get_sigma_u() const {return _sigma_u;}
    /**< get white noise [unit * sec^0.5] to read */
    const double& get_sigma_v() const {return _sigma_v;}
    /**< get scale factor [-] to read */
    const double& get_s() const {return _s;}
    /**< get cross coupling [-] to read */
    const double& get_m() const {return _m;}
    /**< get bias offset [unit] to read */
    const double& get_B0() const {return _B0;}
    /**< get time interval between measurements [sec] to read */
    const double& get_Deltat_sec() const {return _Deltat_sec;}
    /**< get white noise [unit] to read */
    const double& get_std_white_noise() const {return _sigma_v_Delta_tn05;}
    /**< get bias drift [unit] to read */
    const double& get_std_bias_drift() const {return _sigma_u_Delta_t05;}

    /**< get scale factor and cross coupling error matrix (varies from object to object).
     * Note it includes the Identity matrix added to it */
    const Eigen::Matrix3d& get_IM() const {return _IM;}
    /**< get bias offset realization (varies from object to object) */
    const Eigen::Vector3d& get_B0Nu0() const {return _B0Nu0;}
}; // closes class sens_triple_acc

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace sens

#endif

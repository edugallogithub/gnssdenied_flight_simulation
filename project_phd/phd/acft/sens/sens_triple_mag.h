#ifndef ACFT_SENS_TRIPLE_MAG
#define ACFT_SENS_TRIPLE_MAG

#include "../acft.h"
#include "logic.h"
#include "env/coord.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sens {

// CLASS SENS_TRIPLE_MAG
// =====================
// =====================

class ACFT_API sens_triple_mag {
private:
    /**< white noise [unit * sec^0.5] */
    double _sigma_v;
    /**< scale factor [-] */
    double _s;
    /**< cross coupling [-] */
    double _m;
    /**< bias offset [unit] */
    double _B0offset;
    /**< hard iron [unit] */
    double _B0hi;
    /**< time interval between measurements [sec] */
    double _Deltat_sec;

    /**< seed order */
    unsigned short _seed_order;
    /**< seed generator for soft iron (scale factor & cross coupling) plus hard iron */
    std::ranlux24_base _gen_acft;
    /**< standard normal distribution for soft iron (scale factor & cross coupling) plus hard iron */
    std::normal_distribution<double> _dist_acft;
    /**< seed generator for bias offset and white noise */
    std::ranlux24_base _gen_run;
    /**< standard normal distribution for bias offset and white noise */
    std::normal_distribution<double> _dist_run;

    /**< _sigma_v / Delta_t^0.5 */
    double _sigma_v_Delta_tn05;

    /**< scale factor and cross coupling error matrix (depends on aircraft seed), not including identity */
    Eigen::Matrix3d _M_seed;
    /**< scale factor and cross coupling error matrix (depends on aircraft seed), including identity */
    Eigen::Matrix3d _IM_seed;
    /**< hard iron (depends on aircraft seed) */
    Eigen::Vector3d _Bhi_seed;
    /**< bias offset (depends on run seed) */
    Eigen::Vector3d _Boffset_seed;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    sens_triple_mag() = delete;
    /**< constructor based on white noise, scale factor = soft iron, cross coupling = soft iron, run-to-run bias offset,
     * fixed bias offset or hard iron, time interval between measurements, aircraft seed (distinguishes among different
     * aircraft, this is, scale factor, cross coupling, and fixed bias offset), run seed (distinguishes among different
     * runs of the same aircraft, this is, run-to-run bias offset and white noise), and seed order (origin of both
     * aircraft and run seed, and used to access stored data). */
    sens_triple_mag(const double& sigma_v, const double& s, const double& m, const double& B0offset, const double& B0hi, const double& Deltat_sec, const int& acft_seed, const int& run_seed, const unsigned short& seed_order);
    /**< copy constructor */
    sens_triple_mag(const sens_triple_mag&) = delete;
    /**< move constructor */
    sens_triple_mag(sens_triple_mag&&) = delete;
    /**< destructor */
    ~sens_triple_mag() = default;
    /**< copy assignment */
    sens_triple_mag& operator=(const sens_triple_mag&) = delete;
    /**< move assignment */
    sens_triple_mag& operator=(sens_triple_mag&&) = delete;

    /**< return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */
    Eigen::Vector3d eval_noB_noM(const Eigen::Vector3d& B_b_nT_truth, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror);
    /**< return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */
    Eigen::Vector3d eval_noM(const Eigen::Vector3d& B_b_nT_truth, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror);
    /**< return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */
    Eigen::Vector3d eval_complete(const Eigen::Vector3d& B_b_nT_truth, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror);

    /**< create magnetometer model */
    static sens::sens_triple_mag* create_magnetometer(sens::logic::MAG_ID, const int& acft_seed, const int& run_seed, const unsigned short& seed_order, const double& Deltat_sec);

    /**< get white noise [unit * sec^0.5] to read */
    const double& get_sigma_v() const {return _sigma_v;}
    /**< get scale factor [-] to read */
    const double& get_s() const {return _s;}
    /**< get cross coupling [-] to read */
    const double& get_m() const {return _m;}
    /**< get bias offset [unit] */
    const double& get_B0offset() const {return _B0offset;}
    /**< get hard iron [unit] */
    const double& get_B0hi() const {return _B0hi;}
    /**< get time interval between measurements [sec] to read */
    const double& get_Deltat_sec() const {return _Deltat_sec;}
    /**< get white noise [unit] to read */
    const double& get_std_white_noise() const {return _sigma_v_Delta_tn05;}

    /**< get scale factor and cross coupling error matrix (depends on aircraft seed), not including identity */
    const Eigen::Matrix3d& get_M_seed() const {return _M_seed;}
    /**< get scale factor and cross coupling error matrix (depends on aircraft seed), including identity */
    const Eigen::Matrix3d& get_IM_seed() const {return _IM_seed;}
    /**< get hard iron (depends on aircraft seed) */
    const Eigen::Vector3d& get_Bhi_seed() const {return _Bhi_seed;}
    /**< get bias offset (depends on run seed) */
    const Eigen::Vector3d& get_Boffset_seed() const {return _Boffset_seed;}
}; // closes class sens_triple_mag

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace sens

#endif

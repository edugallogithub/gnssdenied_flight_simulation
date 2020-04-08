#ifndef ACFT_SENS_SINGLE_INERTIAL
#define ACFT_SENS_SINGLE_INERTIAL

#include "../acft.h"
#include "logic.h"
#include "env/coord.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sens {

// CLASS SENS_SINGLE_INERTIAL
// ==========================
// ==========================

// It is the same as inertial but only in one axis. Intended for testing exclusively.
// It is not exactly the same because of the the scale factor and cross coupling.
// It also does not include any band to the random walk, so it directly translates into
// the theoretical values.

class ACFT_API sens_single_inertial {
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

    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;

    /**< bias drift limit in standard deviations */
    double _limit;
    /**< distance from limit at which random walk is tricked */
    double _safety;
    /**< _sigma_u * Delta_t^0.5 */
    double _sigma_u_Delta_t05;
    /**< _sigma_v / Delta_t^0.5 */
    double _sigma_v_Delta_tn05;

    /**< scale factor and cross coupling error matrix (1x1) */
    double _M;
    /**< _B0 * Nu0 */
    double _B0Nu0;
    /**< Sum_Nui */
    double _Sum_Nui;

    /**< modifies the input variable by adding one stop of a random walk */
    void grow_random_walk(double& y);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    sens_single_inertial() = delete;
    /**< constructor based on bias drift, white noise, scale factor, cross coupling, bias offset, time interval between measurements, and seed */
    sens_single_inertial(const double& sigma_u, const double& sigma_v, const double& s, const double& m, const double& B0, const double& Deltat_sec, const int& seed);
    /**< copy constructor */
    sens_single_inertial(const sens_single_inertial&) = delete;
    /**< move constructor */
    sens_single_inertial(sens_single_inertial&&) = delete;
    /**< destructor */
    ~sens_single_inertial() = default;
    /**< copy assignment */
    sens_single_inertial& operator=(const sens_single_inertial&) = delete;
    /**< move assignment */
    sens_single_inertial& operator=(sens_single_inertial&&) = delete;

    /**< return new sensor measurement based on sensor input, also filling up the input bias and scale-cross error */
    double eval(const double& Oinput, double& Obias_offset, double& Obias_rwalk, double& Oscalecross);
    /**< create gyroscope model */
    static sens::sens_single_inertial* create_gyroscope(sens::logic::GYR_ID, const int& seed, const double& Deltat_sec);
    /**< create accelerometer model */
    static sens::sens_single_inertial* create_accelerometer(sens::logic::ACC_ID, const int& seed, const double& Deltat_sec);
}; // closes class sens_single_inertial

}; // closes namespace sens

#endif

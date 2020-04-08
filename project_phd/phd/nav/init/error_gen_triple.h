#ifndef NAV_ERROR_GEN_TRIPLE
#define NAV_ERROR_GEN_TRIPLE

#include "../nav.h"
#include "logic.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nav {

// CLASS ERROR_GENERATOR_TRIPLE
// ============================
// ============================

class NAV_API error_gen_triple {
private:
    /**< white noise [unit] */
    double _sigma;
    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;

    /**< evaluation result, so it can be retrieved multiple times */
    Eigen::Vector3d _res;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    error_gen_triple() = delete;
    /**< constructor based on white noise and seed */
    error_gen_triple(const double& sigma, const int& seed);
    /**< copy constructor */
    error_gen_triple(const error_gen_triple&) = delete;
    /**< move constructor */
    error_gen_triple(error_gen_triple&&) = delete;
    /**< destructor */
    ~error_gen_triple() = default;
    /**< copy assignment */
    error_gen_triple& operator=(const error_gen_triple&) = delete;
    /**< move assignment */
    error_gen_triple& operator=(error_gen_triple&&) = delete;

    /**< return sensor measurement */
    const Eigen::Vector3d& eval() const;

    /**< create initial accelerometer error generator */
    static nav::error_gen_triple* create_init_acc_error_generator(nav::logic::INITACC_ID, const int& seed);
    /**< create initial gyroscope error generator */
    static nav::error_gen_triple* create_init_gyr_error_generator(nav::logic::INITGYR_ID, const int& seed);
    /**< create initial magnetometer error generator */
    static nav::error_gen_triple* create_init_mag_error_generator(nav::logic::INITMAG_ID, const int& seed);
    /**< create initial NED magnetic field error generator */
    static nav::error_gen_triple* create_init_mgn_error_generator(nav::logic::INITMGN_ID, const int& seed);

    /**< get white noise [unit] to read */
    const double& get_sigma() const {return _sigma;}
}; // closes class error_gen_triple

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif

#ifndef ACFT_SENS_SINGLE_ONE_USE
#define ACFT_SENS_SINGLE_ONE_USE

#include "../acft.h"
#include "logic.h"
#include "env/coord.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sens {

// CLASS SENS_SINGLE_ONE_USE
// ===========================
// ===========================

class ACFT_API sens_single_one_use {
private:
    /**< white noise [unit] */
    double _sigma;
    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    sens_single_one_use() = delete;
    /**< constructor based on white noise and seed */
    sens_single_one_use(const double& sigma, const int& seed);
    /**< copy constructor */
    sens_single_one_use(const sens_single_one_use&) = delete;
    /**< move constructor */
    sens_single_one_use(sens_single_one_use&&) = delete;
    /**< destructor */
    ~sens_single_one_use() = default;
    /**< copy assignment */
    sens_single_one_use& operator=(const sens_single_one_use&) = delete;
    /**< move assignment */
    sens_single_one_use& operator=(sens_single_one_use&&) = delete;

    /**< return new sensor measurement */
    double eval();

    /**< get white noise [unit] to read */
    const double& get_sigma() const {return _sigma;}
}; // closes class sens_single_one_use

}; // closes namespace sens

#endif

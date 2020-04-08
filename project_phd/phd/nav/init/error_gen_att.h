#ifndef NAV_ERROR_GEN_ATT
#define NAV_ERROR_GEN_ATT

#include "../nav.h"
#include "logic.h"
#include "ang/rotate/rodrigues.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace nav {

// CLASS ERROR_GENERATOR_ATTITUDE
// ==============================
// ==============================

class NAV_API error_gen_att {
private:
    /**< rotation size standard deviation [deg] */
    double _sigma_deg;
    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist_normal;
    /**< uniform integer distribution */
    std::uniform_int_distribution<int> _dist_uniform;

    /**< evaluation result, so it can be retrieved multiple times */
    ang::rodrigues _q_res;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    error_gen_att() = delete;
    /**< constructor based on rotation size standard deviation and seed */
    error_gen_att(const double& sigma_deg, const int& seed);
    /**< copy constructor */
    error_gen_att(const error_gen_att&) = delete;
    /**< move constructor */
    error_gen_att(error_gen_att&&) = delete;
    /**< destructor */
    ~error_gen_att() = default;
    /**< copy assignment */
    error_gen_att& operator=(const error_gen_att&) = delete;
    /**< move assignment */
    error_gen_att& operator=(error_gen_att&&) = delete;

    /**< return quaternion */
    const ang::rodrigues& eval() const;

    /**< create initial Euler angle error generator */
    static nav::error_gen_att* create_init_eul_error_generator(nav::logic::INITEUL_ID, const int& seed);

    /**< get rotation size standard deviation [deg] to read */
    const double& get_sigma_deg() const {return _sigma_deg;}
}; // closes class error_gen_att

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif

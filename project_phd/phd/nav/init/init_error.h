#ifndef NAV_INIT_ERROR
#define NAV_INIT_ERROR

#include "../nav.h"
#include "error_gen_triple.h"
#include "error_gen_att.h"
#include "math/logic/seeder.h"
#include "acft/st/sti.h"
#include "acft/st/trj_sens_out.h"

namespace nav {

// CLASS INIT_ERROR
// ================
// ================

class NAV_API init_error {
private:
    /**< pointer to initial Euler angles error generator */
    nav::error_gen_att* _Piniterr_eul;
    /**< pointer to initial accelerometer error generator */
    nav::error_gen_triple* _Piniterr_acc;
    /**< pointer to initial gyroscope error generator */
    nav::error_gen_triple* _Piniterr_gyr;
    /**< pointer to initial magnetometer error generator */
    nav::error_gen_triple* _Piniterr_mag;
    /**< pointer to initial NED magnetic field error generator */
    nav::error_gen_triple* _Piniterr_mgn;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    init_error() = delete;
    /**< constructor based on seed order and identifiers */
    init_error(math::seeder& Oseeder,
               nav::logic::INITEUL_ID initeul_id, nav::logic::INITACC_ID initacc_id, nav::logic::INITGYR_ID initgyr_id,
               nav::logic::INITMAG_ID initmag_id, nav::logic::INITMGN_ID initmgn_id);
    /**< copy constructor */
    init_error(const init_error&) = delete;
    /**< move constructor */
    init_error(init_error&&) = delete;
    /**< destructor */
    ~init_error();
    /**< copy assignment */
    init_error& operator=(const init_error&) = delete;
    /**< move assignment */
    init_error& operator=(init_error&&) = delete;

    /**< fills up the initial estimated aircraft attitude based on the initial conditions,
     * and writes results in input stream */
    void eval_eul(const st::sti& Osti, ang::rodrigues& q_nb_init, std::ostream& Ostream) const;
    /**< fills up the initial estimated full accelerometer error based on the initial conditions,
     * and writes the results in input stream */
    void eval_acc(const st::st_sens_out& Ost_sens_out_ini, Eigen::Vector3d& Eacc_init_std_mps2, Eigen::Vector3d& Eacc_init_mps2, std::ostream& Ostream) const;
    /**< fills up the initial estimated full gyroscope error based on the initial conditions,
     * and writes the results in input stream */
    void eval_gyr(const st::st_sens_out& Ost_sens_out_ini, Eigen::Vector3d& Egyr_init_std_rps, Eigen::Vector3d& Egyr_init_rps, std::ostream& Ostream) const;
    /**< fills up the initial estimated full magnetometer error based on the initial conditions,
     * and writes the results in input stream */
    void eval_mag(const st::st_sens_out& Ost_sens_out_ini, Eigen::Vector3d& Emag_init_std_nT, Eigen::Vector3d& Emag_init_nT, std::ostream& Ostream) const;
    /**< fills up the initial estimated difference of the magnetic field between the models are reality
     * based on the initial conditions, and writes results in input stream */
    void eval_mgn(const st::sti& Osti, const env::earth& Oearth, Eigen::Vector3d& Berror_init_std_nT, Eigen::Vector3d& Berror_init_nT, std::ostream& Ostream) const;

}; // closes class init_error

}; // closes namespace nav

#endif



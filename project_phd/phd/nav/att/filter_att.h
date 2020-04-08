#ifndef NAV_FILTER_ATT
#define NAV_FILTER_ATT

#include "../nav.h"
#include "../kalman/ekf_handler_.h"
#include "acft/st/trj_nav_in.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_sens_in.h"
#include "acft/st/trj_sens_out.h"
#include "acft/sens/suite.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>

namespace nav {

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER ATTITUDE
// =====================
// =====================

class NAV_API filter_att {
protected:
    /**< weak pointer to sensor suite */
    const sens::suite* const _Psuite;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    filter_att(const sens::suite& Osuite) : _Psuite(&Osuite) {}
    /**< default constructor */
    filter_att() = delete;
    /**< copy constructor */
    filter_att(const filter_att&) = delete;
    /**< move constructor */
    filter_att(filter_att&&) = delete;
    /**< destructor */
    virtual ~filter_att() = default;
    /**< copy assignment */
    filter_att& operator=(const filter_att&) = delete;
    /**< move assignment */
    filter_att& operator=(filter_att&&) = delete;

    /**< complete constructor with navigation time and size of navigation vector, which are not available at construction time */
    virtual void finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav) = 0;
    /**< initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
     * the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other randomly computed variables required
     * for filter initialization, such as initial attitude, initial gyroscope error, initial magnetometer error, initial gravity vector, and
     * initial magnetic field). */
    virtual void initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_nav_in& Ost_nav_in_init,
                            const ang::rodrigues& q_nb_init,
                            const Eigen::Vector3d& E_gyr_init, const Eigen::Vector3d& E_gyr_std_init,
                            const Eigen::Vector3d& E_mag_init, const Eigen::Vector3d& E_mag_std_init,
                            const Eigen::Vector3d& E_B_n_init, const Eigen::Vector3d& E_B_n_std_init) = 0;
    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    virtual void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) = 0;
    /**< resize of all filter components to input size */
    virtual void resize(const unsigned long& nel) = 0;
        /**< returns size of all filter components */
    virtual unsigned long get_size() const = 0;
    /**< switch to no gps mode */
    virtual void switch_to_gps_lost_mode() = 0;
    /**< creates text files describing the filter performance */
    virtual void textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const = 0;
}; // closes class filter_att

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif

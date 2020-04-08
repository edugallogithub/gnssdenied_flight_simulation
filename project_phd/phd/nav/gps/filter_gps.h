#ifndef NAV_FILTER_GPS
#define NAV_FILTER_GPS

#include "../nav.h"
#include "../kalman/ekf_handler_.h"
#include "../kalman/ekfd_handler_.h"
#include "math/math/low_pass.h"
#include "acft/st/trj_nav_in.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_sens_in.h"
#include "acft/st/trj_sens_out.h"
#include "acft/st/trj_gps_out.h"
#include "acft/st/trj_truth.h"
#include "acft/sens/suite.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>

namespace nav {

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER GPS
// ================
// ================

class NAV_API filter_gps {
protected:
    /**< weak pointer to sensor suite */
    const sens::suite* const _Psuite;
    /**< low pass filter to smooth the pressure offset results */
    math::low_pass_single* _Plpf_Deltap_pa;
    /**< low pass filter to smooth the NED low frequency wind results */
    math::low_pass_triple* _Plpf_windlf_ned_mps;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    explicit filter_gps(const sens::suite& Osuite) : _Psuite(&Osuite), _Plpf_Deltap_pa(nullptr), _Plpf_windlf_ned_mps(nullptr) {}
    /**< default constructor */
    filter_gps() = delete;
    /**< copy constructor */
    filter_gps(const filter_gps&) = delete;
    /**< move constructor */
    filter_gps(filter_gps&&) = delete;
    /**< destructor */
    virtual ~filter_gps() {
        delete _Plpf_Deltap_pa;
        delete _Plpf_windlf_ned_mps;
    };
    /**< copy assignment */
    filter_gps& operator=(const filter_gps&) = delete;
    /**< move assignment */
    filter_gps& operator=(filter_gps&&) = delete;

    /**< complete constructor with navigation time and sizes of navigation vector and gps vectors, which are not available at construction time */
    virtual void finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav, const unsigned int& nel_gps) = 0;
    /**< initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
     * the initial GPS output state vector Ost_gps_out_init, the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other
     * randomly computed variables required for filter initialization. */
    virtual void initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_gps_out& Ost_gps_out_init, const st::st_nav_in& Ost_nav_in,
                            const Eigen::Vector3d& E_acc_init, const Eigen::Vector3d& E_acc_std_init) = 0;
    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the sensors output state vector Ost_sens_out, the GPS output
     * state vector Ost_gps_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), the current positions for navigation and
     * GPS in their respective trajectory vectors, and the flag indicating whether there is a new GPS measurement or not. */
    virtual void execute_step(st::st_nav_out& Ost_nav_out, const st::st_sens_out& Ost_sens_out, const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps) = 0;
    /**< resize of all filter components to input size */
    virtual void resize(const unsigned long& nel_fast, const unsigned long& nel_slow) = 0;
    /**< returns size of all filter components */
    virtual unsigned long get_size_fast() const = 0;
    virtual unsigned long get_size_slow() const = 0;

    /**< creates text files describing the filter performance */
    virtual void textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const = 0;

    /**< get low pass filter to smooth the pressure offset results */
    virtual const math::low_pass_single& get_lpf_Deltap_pa() const {return *_Plpf_Deltap_pa;}
    /**< get low pass filter to smooth the low frequency NED wind results */
    virtual const math::low_pass_triple& get_lpf_windlf_ned_mps() const {return *_Plpf_windlf_ned_mps;}

}; // closes class filter_gps

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif

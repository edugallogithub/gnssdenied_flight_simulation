#ifndef NAV_FILTER_NAV
#define NAV_FILTER_NAV

#include "../nav.h"
#include "../air/filter_air.h"
#include "../att/filter_att.h"
#include "../gps/filter_gps01.h"
#include "../pos/filter_pos.h"

#include "env/earth.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_sens_out.h"
#include "acft/st/trj_gps_out.h"
#include "acft/st/trj_nav_in.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>

namespace nav {
    class filter_nav;

    namespace logic {
        /**< Enumeration that contains the different navigation filters */
        enum NAV_ID {
            nav_id010201 = 0,
            nav_id010202 = 1,
            nav_id020203 = 2,
            nav_id020202 = 3,
            nav_id020302 = 4,
            nav_id020402 = 5,
            nav_size     = 6
        };
    } // closes namespace logic

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS EXECUTE_STEP_FUNCTOR
// ==========================
// ==========================

class NAV_API execute_step_functor {
protected:
    /** weak pointer to navigation filter */
    nav::filter_nav* const _Pnav;
public:
    /**< constructor based on navigation filter */
    explicit execute_step_functor(nav::filter_nav& Onav): _Pnav(&Onav) {}
    /**< execute step */
    virtual void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps) = 0;
    /**< resize of all filter components to input size */
    virtual void resize(const unsigned long& s, const unsigned long& g) = 0;
}; // closes class execute_step_functor

class NAV_API execute_step_functor_gps : public execute_step_functor {
public:
    /**< constructor based on navigation filter */
    explicit execute_step_functor_gps(nav::filter_nav& Onav): execute_step_functor(Onav) {}
    /**< execute step */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps) override;
    /**< resize of all filter components to input size */
    void resize(const unsigned long& s, const unsigned long& g) override;
}; // closes class execute_step_functor_gps

class NAV_API execute_step_functor_pos : public execute_step_functor {
public:
    /**< constructor based on navigation filter */
    explicit execute_step_functor_pos(nav::filter_nav& Onav): execute_step_functor(Onav) {}
    /**< execute step */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps) override;
    /**< resize of all filter components to input size */
    void resize(const unsigned long& s, const unsigned long& g) override;
}; // closes class execute_step_functor_pos

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER_NAV
// ================
// ================

class NAV_API filter_nav {
    friend class execute_step_functor;
    friend class execute_step_functor_gps;
    friend class execute_step_functor_pos;
private:
    /**< weak pointer to Earth model */
    const env::geo* const _Pgeo;

    /**< pointer to air data filter */
    nav::filter_air* _Pair;
    /**< pointer to attitude filter */
    nav::filter_att* _Patt;
    /**< pointer to position filter with gps active */
    nav::filter_gps01* _Pgps;
    /**< pointer to position filter with gps lost */
    nav::filter_pos* _Ppos;
    /**< pointer to execute step functor */
    nav::execute_step_functor* _Pstep;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    filter_nav() = delete;
    /**< constructor based on enumeration specifying filter type, suite of sensors, earth model, and turbulence factor */
    filter_nav(nav::logic::NAV_ID, const sens::suite& Osuite, const env::earth& Oearth, const double& turb_factor);
    /**< copy constructor */
    filter_nav(const filter_nav&) = delete;
    /**< move constructor */
    filter_nav(filter_nav&&) = delete;
    /**< destructor */
    virtual ~filter_nav();
    /**< copy assignment */
    filter_nav& operator=(const filter_nav&) = delete;
    /**< move assignment */
    filter_nav& operator=(filter_nav&&) = delete;

    /**< complete constructor with navigation time and sizes of navigation vector, gps vector, gps filter, and pos filter, which are not available at construction time */
    void finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav, const unsigned int& nel_gps, const unsigned int& nel_nav_gps_actv, const unsigned int& nel_nav_gps_lost);
    /**< initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
     * the initial GPS output state vector Ost_gps_out_init, the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only), and other
     * randomly computed variables required for filter initialization, such as initial attitude, initial gyroscope error, initial magnetometer error, initial gravity
     * vector, and initial magnetic field). */
    void initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_gps_out& Ost_gps_out_init, const st::st_nav_in& Ost_nav_in_init,
                    const ang::rodrigues& q_nb_init,
                    const Eigen::Vector3d& E_gyr_init, const Eigen::Vector3d& E_gyr_std_init,
                    const Eigen::Vector3d& E_mag_init, const Eigen::Vector3d& E_mag_std_init,
                    const Eigen::Vector3d& E_acc_init, const Eigen::Vector3d& E_acc_std_init,
                    const Eigen::Vector3d& E_B_n_init, const Eigen::Vector3d& E_B_n_std_init);
    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the GPS output state vector Ost_gps_out, the navigation input state vector Ost_nav_in (for filter evaluation
     * purposes only), the current positions for navigation and GPS in their respective trajectory vectors, and the flag indicating whether there is a new GPS
     * measurement or not. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out,
                      const st::st_gps_out& Ost_gps_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s, const unsigned int& g, bool flag_gps);
    /**< resize of all filter components to input size */
    void resize(const unsigned long& s, const unsigned long& g);
    /**< move from gps active to gps lost mode */
    void switch_to_gps_lost_mode(const st::st_nav_out& Ost_nav_out_init, const unsigned int& s_init);
    /**< create text files with filter results for later plotting in Matlab */
    void text_filter(const boost::filesystem::path& path_folder,
                     const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out, const st::trj_gps_out& Otrj_gps_out) const;

    /**< get reference to air data filter */
    const nav::filter_air& get_filter_air() const {return *_Pair;}
    /**< get reference to attitude filter */
    const nav::filter_att& get_filter_att() const {return *_Patt;}
    /**< get reference to position filter with gps active */
    const nav::filter_gps& get_filter_gps() const {return *_Pgps;}
    /**< get reference to position filter with gps lost */
    const nav::filter_pos& get_filter_pos() const {return *_Ppos;}
}; // closes class filter_nav

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif


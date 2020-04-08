#ifndef NAV_FILTER_POS01
#define NAV_FILTER_POS01

#include "../nav.h"
#include "filter_pos.h"

namespace nav {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS01A
// ===================
// ===================

class NAV_API filter_pos01A : public filter_pos {
private:
    /**< vertical radius of curvature from previous filter step */
    double _N_m_prev;
    /**< meridian radius of curvature from previous filter step */
    double _M_m_prev;
    /**< angular velocity from earth to NED viewed in NED from previous filter step */
    Eigen::Vector3d _w_enn_rps_prev;
    /**< angular velocity from inertial to earth viewed in NED from previous filter step */
    Eigen::Vector3d _w_ien_rps_prev;
    /**< gravity vector (model) from previous filter step */
    Eigen::Vector3d _gc_n_mps2_model_prev;
    /** coriolis acceleration from previous filter step */
    Eigen::Vector3d _a_cor_n_mps2_prev;
    /**< ground speed differential with time viewed in NED */
    Eigen::Vector3d _v_n_dot_mps2;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_pos01A(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos01A() = delete;
    /**< copy constructor */
    filter_pos01A(const filter_pos01A&) = delete;
    /**< move constructor */
    filter_pos01A(filter_pos01A&&) = delete;
    /**< destructor */
    ~filter_pos01A() = default;
    /**< copy assignment */
    filter_pos01A& operator=(const filter_pos01A&) = delete;
    /**< move assignment */
    filter_pos01A& operator=(filter_pos01A&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos01A

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS01B
// ===================
// ===================

class NAV_API filter_pos01B : public filter_pos {
private:
    /**< vertical radius of curvature from previous filter step */
    double _N_m_prev;
    /**< meridian radius of curvature from previous filter step */
    double _M_m_prev;
    /**< angular velocity from earth to NED viewed in NED from previous filter step */
    Eigen::Vector3d _w_enn_rps_prev;
    /**< angular velocity from inertial to earth viewed in NED from previous filter step */
    Eigen::Vector3d _w_ien_rps_prev;
    /**< gravity vector (model) from previous filter step */
    Eigen::Vector3d _gc_n_mps2_model_prev;
    /** coriolis acceleration from previous filter step */
    Eigen::Vector3d _a_cor_n_mps2_prev;
    /**< ground speed differential with time viewed in NED */
    Eigen::Vector3d _v_n_dot_mps2;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_pos01B(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos01B() = delete;
    /**< copy constructor */
    filter_pos01B(const filter_pos01B&) = delete;
    /**< move constructor */
    filter_pos01B(filter_pos01B&&) = delete;
    /**< destructor */
    ~filter_pos01B() = default;
    /**< copy assignment */
    filter_pos01B& operator=(const filter_pos01B&) = delete;
    /**< move assignment */
    filter_pos01B& operator=(filter_pos01B&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos01B

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS01C
// ===================
// ===================

class NAV_API filter_pos01C : public filter_pos {
private:
    /**< vertical radius of curvature from previous filter step */
    double _N_m_prev;
    /**< meridian radius of curvature from previous filter step */
    double _M_m_prev;
    /**< angular velocity from earth to NED viewed in NED from previous filter step */
    Eigen::Vector3d _w_enn_rps_prev;
    /**< angular velocity from inertial to earth viewed in NED from previous filter step */
    Eigen::Vector3d _w_ien_rps_prev;
    /**< gravity vector (model) from previous filter step */
    Eigen::Vector3d _gc_n_mps2_model_prev;
    /** coriolis acceleration from previous filter step */
    Eigen::Vector3d _a_cor_n_mps2_prev;
    /**< ground speed differential with time viewed in NED */
    Eigen::Vector3d _v_n_dot_mps2;
    /**< differential with time of geodetic coordinates */
    Eigen::Vector3d _x_gdt_rad_m_dot;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_pos01C(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos01C() = delete;
    /**< copy constructor */
    filter_pos01C(const filter_pos01C&) = delete;
    /**< move constructor */
    filter_pos01C(filter_pos01C&&) = delete;
    /**< destructor */
    ~filter_pos01C() = default;
    /**< copy assignment */
    filter_pos01C& operator=(const filter_pos01C&) = delete;
    /**< move assignment */
    filter_pos01C& operator=(filter_pos01C&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos01C

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif

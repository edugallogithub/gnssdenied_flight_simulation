#ifndef NAV_FILTER_POS03
#define NAV_FILTER_POS03

#include "../nav.h"
#include "filter_pos.h"

namespace nav {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS03A
// ===================
// ===================

class NAV_API filter_pos03A : public filter_pos {
private:
    /**< true airspeed in body */
    Eigen::Vector3d _vtas_b_mps;
    /**< true airspeed differential with time in body */
    Eigen::Vector3d _vtas_b_dot_mps2;
    /**< true airspeed in NED */
    Eigen::Vector3d _vtas_n_mps;
    /**< true airspeed differential with time in NED */
    Eigen::Vector3d _vtas_n_dot_mps2;

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
    filter_pos03A(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos03A() = delete;
    /**< copy constructor */
    filter_pos03A(const filter_pos03A&) = delete;
    /**< move constructor */
    filter_pos03A(filter_pos03A&&) = delete;
    /**< destructor */
    ~filter_pos03A() = default;
    /**< copy assignment */
    filter_pos03A& operator=(const filter_pos03A&) = delete;
    /**< move assignment */
    filter_pos03A& operator=(filter_pos03A&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos03A

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS03B
// ===================
// ===================

class NAV_API filter_pos03B : public filter_pos {
private:
    /**< true airspeed in body */
    Eigen::Vector3d _vtas_b_mps;
    /**< true airspeed differential with time in body */
    Eigen::Vector3d _vtas_b_dot_mps2;
    /**< true airspeed in NED */
    Eigen::Vector3d _vtas_n_mps;
    /**< true airspeed differential with time in NED */
    Eigen::Vector3d _vtas_n_dot_mps2;

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
    filter_pos03B(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos03B() = delete;
    /**< copy constructor */
    filter_pos03B(const filter_pos03B&) = delete;
    /**< move constructor */
    filter_pos03B(filter_pos03B&&) = delete;
    /**< destructor */
    ~filter_pos03B() = default;
    /**< copy assignment */
    filter_pos03B& operator=(const filter_pos03B&) = delete;
    /**< move assignment */
    filter_pos03B& operator=(filter_pos03B&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos03B

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS03C
// ===================
// ===================

class NAV_API filter_pos03C : public filter_pos {
private:
    /**< true airspeed in body */
    Eigen::Vector3d _vtas_b_mps;
    /**< true airspeed differential with time in body */
    Eigen::Vector3d _vtas_b_dot_mps2;
    /**< true airspeed in NED */
    Eigen::Vector3d _vtas_n_mps;
    /**< true airspeed differential with time in NED */
    Eigen::Vector3d _vtas_n_dot_mps2;

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
    filter_pos03C(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos03C() = delete;
    /**< copy constructor */
    filter_pos03C(const filter_pos03C&) = delete;
    /**< move constructor */
    filter_pos03C(filter_pos03C&&) = delete;
    /**< destructor */
    ~filter_pos03C() = default;
    /**< copy assignment */
    filter_pos03C& operator=(const filter_pos03C&) = delete;
    /**< move assignment */
    filter_pos03C& operator=(filter_pos03C&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos03C

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


}; // closes namespace nav

#endif

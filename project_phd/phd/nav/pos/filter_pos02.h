#ifndef NAV_FILTER_POS02
#define NAV_FILTER_POS02

#include "../nav.h"
#include "filter_pos.h"

namespace nav {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS02A
// ===================
// ===================

class NAV_API filter_pos02A : public filter_pos {
private:
    /**< vertical radius of curvature from previous filter step */
    double _N_m_prev;
    /**< meridian radius of curvature from previous filter step */
    double _M_m_prev;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_pos02A(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos02A() = delete;
    /**< copy constructor */
    filter_pos02A(const filter_pos02A&) = delete;
    /**< move constructor */
    filter_pos02A(filter_pos02A&&) = delete;
    /**< destructor */
    ~filter_pos02A() = default;
    /**< copy assignment */
    filter_pos02A& operator=(const filter_pos02A&) = delete;
    /**< move assignment */
    filter_pos02A& operator=(filter_pos02A&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos02A

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS02B
// ===================
// ===================

class NAV_API filter_pos02B : public filter_pos {
private:
    /**< vertical radius of curvature from previous filter step */
    double _N_m_prev;
    /**< meridian radius of curvature from previous filter step */
    double _M_m_prev;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_pos02B(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos02B() = delete;
    /**< copy constructor */
    filter_pos02B(const filter_pos02B&) = delete;
    /**< move constructor */
    filter_pos02B(filter_pos02B&&) = delete;
    /**< destructor */
    ~filter_pos02B() = default;
    /**< copy assignment */
    filter_pos02B& operator=(const filter_pos02B&) = delete;
    /**< move assignment */
    filter_pos02B& operator=(filter_pos02B&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos02B

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS02C
// ===================
// ===================

class NAV_API filter_pos02C : public filter_pos {
private:
    /**< vertical radius of curvature from previous filter step */
    double _N_m_prev;
    /**< meridian radius of curvature from previous filter step */
    double _M_m_prev;
    /**< differential with time of geodetic coordinates */
    Eigen::Vector3d _x_gdt_rad_m_dot;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_pos02C(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos02C() = delete;
    /**< copy constructor */
    filter_pos02C(const filter_pos02C&) = delete;
    /**< move constructor */
    filter_pos02C(filter_pos02C&&) = delete;
    /**< destructor */
    ~filter_pos02C() = default;
    /**< copy assignment */
    filter_pos02C& operator=(const filter_pos02C&) = delete;
    /**< move assignment */
    filter_pos02C& operator=(filter_pos02C&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos02C

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS02D
// ===================
// ===================

class NAV_API filter_pos02D : public filter_pos {
private:
    /**< vertical radius of curvature from previous filter step */
    double _N_m_prev;
    /**< meridian radius of curvature from previous filter step */
    double _M_m_prev;
    /**< differential with time of geodetic coordinates */
    Eigen::Vector3d _x_gdt_rad_m_dot;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_pos02D(const sens::suite& Osuite, const env::geo& Ogeo) : filter_pos(Osuite, Ogeo) {}
    /**< default constructor */
    filter_pos02D() = delete;
    /**< copy constructor */
    filter_pos02D(const filter_pos02D&) = delete;
    /**< move constructor */
    filter_pos02D(filter_pos02D&&) = delete;
    /**< destructor */
    ~filter_pos02D() = default;
    /**< copy assignment */
    filter_pos02D& operator=(const filter_pos02D&) = delete;
    /**< move assignment */
    filter_pos02D& operator=(filter_pos02D&&) = delete;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
}; // closes class filter_pos02D

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
}; // closes namespace nav

#endif

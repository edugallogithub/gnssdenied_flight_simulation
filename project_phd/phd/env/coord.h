#ifndef ENV_COORD
#define ENV_COORD

#include "env.h"
#include "ang/auxiliary.h"
#include "math/logic/constant.h"
#include <cmath>

/*
This file contains three different coordinate classes: "geocentric_coord" for geocentric
coordinates, "geodetic_coord" for geodetic ones, and "cartesian_coord" for cartesian ones.
For conversions among the different coordinates, refer to the "em" (Earth model) class.
*/

namespace env {

// CLASS GEOCENTRIC_COORD
// ======================
// ======================

class ENV_API geocentric_coord {
private:
    /**< vector containing the three geocentric coordinates (colatitude, longitude, geocentric distance) */
    Eigen::Array3d _data;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor (all attributes undefined) */
    geocentric_coord() = default;
    /**< constructor based on colatitude, longitude, and geocentric distance */
    geocentric_coord(double theta_rad, double lambda_rad, double r_m) : _data(theta_rad, lambda_rad, r_m) {}
    /**< constructor based on a vector containing colatitude, longitude, and geocentric distance (in this order) */
    geocentric_coord(const Eigen::Array3d& data) : _data(data) {}
    /**< copy constructor */
    geocentric_coord(const geocentric_coord&) = default;
    /**< move constructor */
    geocentric_coord(geocentric_coord&&) = default;
    /**< destructor */
    ~geocentric_coord() = default;
    /**< copy assignment */
    geocentric_coord& operator=(const geocentric_coord&) = default;
    /**< move assignment */
    geocentric_coord& operator=(geocentric_coord&&) = default;

    /**< overloaded operator () (function) to read */
    const Eigen::Array3d& operator()() const {return _data;}
    /**< overloaded operator () (function) to write */
    Eigen::Array3d& operator()() {return _data;}

    /**< modify all attributes simultaneously */
    void set(const double& theta_rad, const double& lambda_rad, const double& r_m);
    /**< modify all attributes simultaneously */
    void set(const Eigen::Array3d& data);

    /**< access to geocentric colatitude to read or write*/
	const double& get_theta_rad() const {return _data(0);}
	double& get_theta_rad() {return _data(0);}
	/**< access to longitude to read or write */
	const double& get_lambda_rad() const	{return _data(1);}
	double& get_lambda_rad() {return _data(1);}
	/**< access to geocentric distance to read or write */
	const double& get_r_m() const {return _data(2);}
	double& get_r_m() {return _data(2);}
}; // closes class geocentric_coord

/**< adds the input geocentric coordinates (in [deg-m]) object to the stream */
ENV_API inline std::ostream& operator <<(std::ostream & out_str, const geocentric_coord& O) {
    out_str << O()(0) * math::constant::R2D() << " " << O()(1) * math::constant::R2D() << " " << O()(2);
    return out_str;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// CLASS GEODETIC_COORD
// ====================
// ====================

class ENV_API geodetic_coord {
private:
    /**< vector containing the three geodetic coordinates (longitude, latitude, altitude) */
    Eigen::Array3d _data;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< ===== ===== Constructors ===== ===== */
    /**< default constructor (all attributes undefined) */
    geodetic_coord() = default;
    /**< constructor based on longitude, latitude, and geometric altitude. */
    geodetic_coord(double lambda_rad, double phi_rad, double h_m) : _data(lambda_rad, phi_rad, h_m) {}
    /**< constructor based on an array containing longitude, latitude, and geometric altitude (in this order) */
    explicit geodetic_coord(const Eigen::Array3d& data) : _data(data) {}
    /**< copy constructor */
    geodetic_coord(const geodetic_coord&) = default;
    /**< destructor */
    ~geodetic_coord() = default;

    /**< ===== ===== Move Constructors ===== ===== */
    /**< move constructor */
    geodetic_coord(geodetic_coord&&) = default;
    /**< move constructor based on an array containing longitude, latitude, and geometric altitude (in this order) */
    explicit geodetic_coord(Eigen::Array3d&& data) : _data(data) {}

    /**< ===== ===== Assignments ===== ===== */
    /**< copy assignment */
    geodetic_coord& operator=(const geodetic_coord&) = default;
    /**< copy assignment based on an array containing longitude, latitude, and geometric altitude (in this order) */
    geodetic_coord& operator=(const Eigen::Array3d&& data);

    /**< ===== ===== Move Assignments ===== ===== */
    /**< move assignment */
    geodetic_coord& operator=(geodetic_coord&&) = default;
    /**< move assignment based on an array containing longitude, latitude, and geometric altitude (in this order) */
    geodetic_coord& operator=(Eigen::Array3d&& data);

	/**< overloaded operator + (addition) */
	geodetic_coord operator+(const geodetic_coord& op2) const;
    /**< overloaded operator + (addition) */
    geodetic_coord operator+(const Eigen::Array3d& op2) const;
    /**< overloaded operator - (substraction) */
    geodetic_coord operator-(const geodetic_coord& op2) const;
    /**< overloaded operator - (substraction) */
    geodetic_coord operator-(const Eigen::Array3d& op2) const;
	/**< overloaded operator * (scalar product) */
	geodetic_coord operator*(const double& op2) const;
    /**< overloaded operator () (function) to read */
    const Eigen::Array3d& operator()() const {return _data;}
    /**< overloaded operator () (function) to write */
    Eigen::Array3d& operator()() {return _data;}

    /**< modify all attributes simultaneously */
    void set(const double& lambda_rad, const double& phi_rad, const double& h_m);
    /**< modify all attributes simultaneously */
    void set(const Eigen::Array3d& data);

	/**< access to longitude to read or write */
    const double& get_lambda_rad() const {return _data(0);}
	double& get_lambda_rad() {return _data(0);}
	/**< access to latitude to read or write */
	const double& get_phi_rad() const {return _data(1);}
	double& get_phi_rad() {return _data(1);}
	/**< access to geometric altitude to read or write */
	const double& get_h_m() const {return _data(2);}
	double& get_h_m() {return _data(2);}
}; // closes class geodetic_coord

/**< adds the input geodetic coordinates (in [deg-m]) object to the stream */
ENV_API inline std::ostream& operator <<(std::ostream & out_str, const geodetic_coord& O) {
    out_str << O()(0) * math::constant::R2D() << " " << O()(1) * math::constant::R2D() << " " << O()(2);
    return out_str;
}

/**< takes the stream data into the geodetic coordinates (phi comes before lambda !!!!!!!!!) */
ENV_API inline std::istream& operator >>(std::istream & in_str, geodetic_coord& O) {
    in_str >> O.get_phi_rad() >> O.get_lambda_rad() >> O.get_h_m();
    return in_str;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// CLASS CARTESIAN_COORD
// =====================
// =====================

class ENV_API cartesian_coord {
private:
    /**< vector containing the three cartesian coordinates */
    Eigen::Array3d _data;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor (all attributes undefined) */
    cartesian_coord() = default;
    /**< constructor based on references to three cartesian distances. */
    cartesian_coord(double x1_m, double x2_m, double x3_m) : _data(x1_m, x2_m, x3_m) {}
    /**< constructor based on a vector containing three cartesian coordinates (in this order) */
    cartesian_coord(const Eigen::Array3d& data) : _data(data) {}
    /**< copy constructor */
    cartesian_coord(const cartesian_coord&) = default;
    /**< move constructor */
    cartesian_coord(cartesian_coord&&) = default;
    /**< destructor */
    ~cartesian_coord() = default;
    /**< copy assignment */
    cartesian_coord& operator=(const cartesian_coord&) = default;
    /**< move assignment */
    cartesian_coord& operator=(cartesian_coord&&) = default;

    /**< overloaded operator () (function) to read */
    const Eigen::Array3d& operator()() const {return _data;}
    /**< overloaded operator () (function) to write */
    Eigen::Array3d& operator()() {return _data;}

    /**< modify all attributes simultaneously */
    void set(const double& x1_m, const double& x2_m, const double& x3_m);
    /**< modify all attributes simultaneously */
    void set(const Eigen::Array3d& data);

	/**< access to first cartesian coordinate to read or write */
	const double& get_x1_m() const {return _data(0);}
	double& get_x1_m() {return _data(0);}
	/**< access to second cartesian coordinate to read or write */
	const double& get_x2_m() const {return _data(1);}
	double& get_x2_m() {return _data(1);}
	/**< access to third cartesian coordinate to read or write */
	const double& get_x3_m() const {return _data(2);}
	double& get_x3_m() {return _data(2);}
}; // closes class cartesian_coord

/**< adds the input cartesian coordinates (in [m]) object to the stream */
ENV_API inline std::ostream& operator <<(std::ostream & out_str, const cartesian_coord& O) {
    out_str << O()(0) << " " << O()(1) << " " << O()(2);
    return out_str;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

} // closes namespace env

#endif


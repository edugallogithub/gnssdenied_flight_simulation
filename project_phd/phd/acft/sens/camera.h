#ifndef ACFT_CAMERA
#define ACFT_CAMERA

#include "../acft.h"
#include "ang/rotate/euler.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/rodrigues.h"

namespace sens {

// CLASS CAMERA
// ============
// ============

class ACFT_API camera {
private:
    /**< number of horizontal pixels (number of columns) */
    unsigned int _width_px;
    /**< number of vertical pixels (number of rows) */
    unsigned int _height_px;
    /**< field of view */
    double _vfov_deg;
    /**< focal length [pixels] */
    double _f_px;
    /**< horizontal principal point location (from left side) */
    double _cx_px;
    /**< vertical principal point location (from top) */
    double _cy_px;
    /**< Euler angles from body to camera */
    ang::euler _euler_bc;
    /**< rotation matrix from body to camera */
    ang::dcm _R_bc;
    /**< quaternion from body to camera */
    ang::rodrigues _q_bc;
public:
    /**< default constructor */
    camera() = delete;
    /**< constructor based on images width and height, vertical field of view, and Euler angles from body to camera */
    camera(const unsigned int& width_px, const unsigned int& height_px, const double& vfov_deg, const ang::euler& euler_bc);
    /**< copy constructor */
    camera(const camera&) = delete;
    /**< move constructor */
    camera(camera&&) = delete;
    /**< destructor */
    ~camera() = default;
    /**< copy assignment */
    camera& operator=(const camera&) = delete;
    /**< move assignment */
    camera& operator=(camera&&) = delete;

    /**< projection from pixels to world coordiantes, returning unitary bearing vector */
    Eigen::Vector3d convert_img2uscrs(const double& x_img_pxi, const double& x_img_pxii) const;
    /**< projection from pixels to world coordiantes, returning unitary bearing vector */
    Eigen::Vector3d convert_img2uscrs(const Eigen::Vector2d& x_img_px) const;
    /**< transforms cartesian coordinates from camera reference system (CRS) to image reference system (IMG).
    Unit sphere or unit plane camera reference systems (USCRS or UPCRS) are also acceptable inputs. */
    Eigen::Vector2d convert_crs2img(const Eigen::Vector3d& x_crs_m) const;
    /**< transforms cartesian coordinates from unit plane (3rd coord == 1) camera reference system (UPCRS) to image reference system (IMG) */
    Eigen::Vector2d convert_upcrs2img(const Eigen::Vector2d& x_crs_up) const;
    /**< returns true if input image reference system (IMG) coordinates are inside frame and not closer than input boundary to frame border */
    bool is_in_frame(const Eigen::Vector2i& x_img_px, int boundary=0) const;
    /**< returns true if input image reference system (IMG) coordinates (at input pyramid level) are inside frame and not closer than input boundary to frame border */
    bool is_in_frame(const Eigen::Vector2i& x_img_px, int boundary, int level) const;

    /**< get number of horizontal pixels (number of columns) */
    const unsigned int& get_width_px() const {return _width_px;}
    /**< get number of vertical pixels (number of rows) */
    const unsigned int& get_height_px() const {return _height_px;}
    /**< get field of view */
    const double& get_vfov_deg() const {return _vfov_deg;}
    /**< get focal length [pixels] */
    const double& get_f_px() const {return _f_px;}
    /**< get horizontal principal point location (from left side) */
    const double& get_cx_px() const {return _cx_px;}
    /**< get vertical principal point location (from top) */
    const double& get_cy_px() const {return _cy_px;}
    /**< get Euler angles from body to camera */
    const ang::euler& get_euler_bc() const {return _euler_bc;}
    /**< get rotation matrix from body to camera */
    const ang::dcm& get_R_bc() const {return _R_bc;}
    /**< get quaternion from body to camera */
    const ang::rodrigues& get_q_bc() const {return _q_bc;}
}; // closes class camera

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

} // closes namespace sens

#endif


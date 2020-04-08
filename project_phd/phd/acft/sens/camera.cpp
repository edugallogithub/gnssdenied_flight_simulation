#include "camera.h"
#include "math/templates/math_.h"

// CLASS CAMERA
// ============
// ============

///////////////////////////////////////////////////////
// LLAMARLO ASÏ:
// auto             Pcam     = new sens::camera(768, 1024, 49.2255, euler_bc); //focal length of 19 [mm] equals vfov of 49.2255 [deg]
//////////////////////////////////////////////////////

sens::camera::camera(const unsigned int& width_px, const unsigned int& height_px, const double& vfov_deg, const ang::euler& euler_bc)
: _width_px(width_px), _height_px(height_px), _vfov_deg(vfov_deg), _f_px(height_px / (2.0 * tan(0.5 * vfov_deg * math::constant::D2R()))),
  _cx_px(0.5 * width_px - 0.5), _cy_px(0.5 * height_px - 0.5), _euler_bc(euler_bc), _R_bc(euler_bc), _q_bc(euler_bc) {
}
/* constructor based on images width and height, vertical field of view, and Euler angles from body to camera */

Eigen::Vector3d sens::camera::convert_img2uscrs(const Eigen::Vector2d& x_img_px) const {
    return this->convert_img2uscrs(x_img_px[0], x_img_px[1]);
}
/* projection from pixels to world coordiantes, returning unitary bearing vector */

Eigen::Vector3d sens::camera::convert_img2uscrs(const double& x_img_pxi, const double& x_img_pxii) const {
    Eigen::Vector3d xyz;
    xyz[0] = (x_img_pxi - _cx_px) / _f_px;
    xyz[1] = (x_img_pxii - _cy_px) / _f_px;
    xyz[2] = 1.0;
    return xyz.normalized();
}
/* projection from pixels to world coordiantes, returning unitary bearing vector */

Eigen::Vector2d sens::camera::convert_crs2img(const Eigen::Vector3d& x_crs_m) const {
    return this->convert_upcrs2img(Eigen::Vector2d(x_crs_m(0)/x_crs_m(2), x_crs_m(1)/x_crs_m(2)));
}
/* transforms cartesian coordinates from camera reference system (CRS) to image reference system (IMG).
Unit sphere or unit plane camera reference systems (USCRS or UPCRS) are also acceptable inputs. */

Eigen::Vector2d sens::camera::convert_upcrs2img(const Eigen::Vector2d& x_crs_up) const {
    return Eigen::Vector2d(_f_px * x_crs_up(0) + _cx_px, _f_px * x_crs_up(1) + _cy_px);
}
/* transforms cartesian coordinates from unit plane (3rd coord == 1) camera reference system (UPCRS) to image reference system (IMG) */

bool sens::camera::is_in_frame(const Eigen::Vector2i& x_car_img_px, int boundary) const {
    if ((x_car_img_px[0] >= boundary) && (x_car_img_px[0] < _width_px-boundary) &&
        (x_car_img_px[1] >= boundary) && (x_car_img_px[1] < _height_px-boundary)) {
        return true;
    }
    return false;
}
/* returns true if input image reference system (IMG) coordinates are inside frame and not closer than input boundary to frame border */

bool sens::camera::is_in_frame(const Eigen::Vector2i& x_car_img_px, int boundary, int level) const {
    if ((x_car_img_px[0] >= boundary) && (x_car_img_px[0] < _width_px / math::pyramid_scale(level) - boundary) &&
        (x_car_img_px[1] >= boundary) && (x_car_img_px[1] < _height_px / math::pyramid_scale(level) - boundary)) {
        return true;
    }
    return false;
}
/* returns true if input image reference system (IMG) coordinates (at input pyramid level) are inside frame and not closer than input boundary to frame border */






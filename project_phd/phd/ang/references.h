#ifndef ANG_SPECIFIC_REFERENCES
#define ANG_SPECIFIC_REFERENCES

#include "ang.h"
#include "rotate/so3.h"
#include "math/logic/constant.h"

namespace ang {

// CLASS BODY_CAMERA
// =================
// =================

/*
This class controls the conversion between BFS (Body Fixed System) and CRS (Camera Reference System) SO3 groups
when the camera is rigidly attached to the aircraft body in a strap down configuration.

VERY IMPORTANT: Please note that the SO3 objects used as input and output in the functions below are those whose
SO3.get_euler() methods directly provide the classical aeronautical Euler angles (yaw, pitch, roll). This may require
the use of the SO3.inverse() function for both the inputs and the outputs.
*/

class ANG_API body_camera {
public:
    /**< convert the input SO3 object containing the rotation of the BFS (Body Fixed System) with respect to a
    different reference system (usually NED - North East Down, but can be anything) into an SO3 object containing
    the rotation of the CRS (Camera Reference System) with respect to the same different system (usually NED) */
    virtual ang::SO3 bfs2crs(const ang::SO3& R_nedbfs) const = 0;

    /**< convert the input SO3 object containing the rotation of the CRS (Camera Reference System) with respect to a
    different reference system (usually NED - North East Down, but can be anything) into an SO3 object containing
    the rotation of the BFS (Body Fixed System) with respect to the same different system (usually NED) */
    virtual ang::SO3 crs2bfs(const ang::SO3& R_nedcrs) const = 0;
}; // closes class body_camera

class ANG_API body_camera_ninety : public body_camera {
/* The camera CRS "x" axis coincides with the aircraft BFS "y" axis.
 * The camera CRS "y" axis coincides with the aircraft negative BFS "x" axis.
 * The camera CRS "z" axis coincides with the aircraft BFS "z" axis.
 */
public:
    /**< convert the input SO3 object containing the rotation of the BFS (Body Fixed System) with respect to a
    different reference system (usually NED - North East Down, but can be anything) into an SO3 object containing
    the rotation of the CRS (Camera Reference System) with respect to the same different system (usually NED) */
    ang::SO3 bfs2crs(const ang::SO3& R_nedbfs) const {
        ang::SO3 R_bfscrs(math::constant::PIHALF(), 0., 0.);
        return R_nedbfs * R_bfscrs;
    }
    /**< convert the input SO3 object containing the rotation of the CRS (Camera Reference System) with respect to a
    different reference system (usually NED - North East Down, but can be anything) into an SO3 object containing
    the rotation of the BFS (Body Fixed System) with respect to the same different system (usually NED) */
    ang::SO3 crs2bfs(const ang::SO3& R_nedcrs) const {
        ang::SO3 R_crsbfs(-math::constant::PIHALF(), 0., 0.);
        return R_nedcrs * R_crsbfs;
    }
}; // closes class body_camera_ninety

class ANG_API body_camera_zero : public body_camera {
/* The camera CRS "x" axis coincides with the aircraft BFS "x" axis.
 * The camera CRS "y" axis coincides with the aircraft BFS "y" axis.
 * The camera CRS "z" axis coincides with the aircraft BFS "z" axis.
 */
public:
    /**< convert the input SO3 object containing the rotation of the BFS (Body Fixed System) with respect to a
    different reference system (usually NED - North East Down, but can be anything) into an SO3 object containing
    the rotation of the CRS (Camera Reference System) with respect to the same different system (usually NED) */
    ang::SO3 bfs2crs(const ang::SO3& R_nedbfs) const {
        return R_nedbfs;
    }
    /**< convert the input SO3 object containing the rotation of the CRS (Camera Reference System) with respect to a
    different reference system (usually NED - North East Down, but can be anything) into an SO3 object containing
    the rotation of the BFS (Body Fixed System) with respect to the same different system (usually NED) */
    ang::SO3 crs2bfs(const ang::SO3& R_nedcrs) const {
        return R_nedcrs;
    }
}; // closes class body_camera_zero

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

} // closes namespace ang

#endif


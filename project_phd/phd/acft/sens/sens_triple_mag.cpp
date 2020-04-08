#include "sens_triple_mag.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

// CLASS SENS_TRIPLE_MAG
// =====================
// =====================

sens::sens_triple_mag::sens_triple_mag(const double& sigma_v, const double& s, const double& m, const double& B0offset, const double& B0hi, const double& Deltat_sec,
                                       const int& acft_seed, const int& run_seed, const unsigned short& seed_order)
: _sigma_v(sigma_v), _s(s), _m(m), _B0hi(B0hi), _B0offset(B0offset), _gen_acft(acft_seed), _gen_run(run_seed), _Deltat_sec(Deltat_sec), _dist_acft(0.,1.), _dist_run(0.,1.),
  _sigma_v_Delta_tn05(sigma_v / sqrt(Deltat_sec)), _seed_order(seed_order) {
    _M_seed << _s * _dist_acft(_gen_acft), _m * _dist_acft(_gen_acft), _m * _dist_acft(_gen_acft),
               _m * _dist_acft(_gen_acft), _s * _dist_acft(_gen_acft), _m * _dist_acft(_gen_acft),
               _m * _dist_acft(_gen_acft), _m * _dist_acft(_gen_acft), _s * _dist_acft(_gen_acft);

    // we need 9 execution of _dist_run to bias results below are the same as they were when _M
    // was filled up based on _dist_run and not _dist_acft
    _dist_run(_gen_run);
    _dist_run(_gen_run);
    _dist_run(_gen_run);
    _dist_run(_gen_run);
    _dist_run(_gen_run);
    _dist_run(_gen_run);
    _dist_run(_gen_run);
    _dist_run(_gen_run);
    _dist_run(_gen_run);

    _IM_seed = Eigen::Matrix3d::Identity() + _M_seed;
    _Bhi_seed << _B0hi * _dist_acft(_gen_acft), _B0hi * _dist_acft(_gen_acft), _B0hi * _dist_acft(_gen_acft);
    //_Bhi_seed << 0., 0., 0.;
    _Boffset_seed << _B0offset * _dist_run(_gen_run), _B0offset * _dist_run(_gen_run), _B0offset * _dist_run(_gen_run);
}
/* constructor based on white noise, scale factor = soft iron, cross coupling = soft iron, run-to-run bias offset,
 * fixed bias offset or hard iron, time interval between measurements, aircraft seed (distinguishes among different
 * aircraft, this is, scale factor, cross coupling, and fixed bias offset), run seed (distinguishes among different
 * runs of the same aircraft, this is, run-to-run bias offset and white noise), and seed order (origin of both
 * aircraft and run seed, and used to access stored data). */

Eigen::Vector3d sens::sens_triple_mag::eval_noB_noM(const Eigen::Vector3d& B_b_nT_truth, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    Obias = Eigen::Vector3d::Zero();
    Oscalecross = Eigen::Vector3d::Zero();
    Oerror = Obias + Oscalecross;
    return B_b_nT_truth + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

Eigen::Vector3d sens::sens_triple_mag::eval_noM(const Eigen::Vector3d& B_b_nT_truth, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    Obias = _Bhi_seed + _Boffset_seed;
    Oscalecross = Eigen::Vector3d::Zero();
    Oerror = Obias + Oscalecross;
    return B_b_nT_truth + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

Eigen::Vector3d sens::sens_triple_mag::eval_complete(const Eigen::Vector3d& B_b_nT_truth, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    Obias = _Bhi_seed + _Boffset_seed;
    Oscalecross = _M_seed * B_b_nT_truth;
    Oerror = Obias + Oscalecross;
    return B_b_nT_truth + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

sens::sens_triple_mag* sens::sens_triple_mag::create_magnetometer(sens::logic::MAG_ID mag_id, const int& acft_seed, const int& run_seed, const unsigned short& seed_order, const double& Deltat_sec) {
    sens::sens_triple_mag* Pres = nullptr;
    switch(mag_id) {
        case sens::logic::mag_id_zero:
            Pres = new sens::sens_triple_mag(0., 0., 0., 0., 0., Deltat_sec, acft_seed, run_seed, seed_order);
            break;
        case sens::logic::mag_id_base:
            Pres = new sens::sens_triple_mag(5.00e0, 7.50e-4, 9.16e-4, 5.00e2, 1.75e2, Deltat_sec, acft_seed, run_seed, seed_order);
            break;
        case sens::logic::mag_id_better:
            Pres = new sens::sens_triple_mag(3.00e0, 5.00e-4, 7.00e-4, 3.00e2, 1.25e2, Deltat_sec, acft_seed, run_seed, seed_order);
            break;
        case sens::logic::mag_id_worse:
            Pres = new sens::sens_triple_mag(1.00e1, 1.25e-3, 1.50e-3, 7.50e2, 3.50e2, Deltat_sec, acft_seed, run_seed, seed_order);
            break;
        case sens::logic::mag_size:
            throw std::runtime_error("Magnetometer model not available");
        default:
            throw std::runtime_error("Magnetometer model not available");
    }
    return Pres;
}
/* create magnetometer model */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////















#include "sens_triple_gyr.h"
#include "platform.h"
#include "../acft/iner.h"

#include <iostream>

// CLASS SENS_TRIPLE_GYR
// =====================
// =====================

sens::sens_triple_gyr::sens_triple_gyr(const double& sigma_u, const double& sigma_v, const double& s, const double& m, const double& B0, const double& limit, const double& Deltat_sec, const int& acft_seed, const int& run_seed, const sens::platform& Oplat)
: _sigma_u(sigma_u), _sigma_v(sigma_v), _s(s), _m(m), _B0(B0), _gen_acft(acft_seed), _gen_run(run_seed), _Deltat_sec(Deltat_sec), _dist_acft(0.,1.), _dist_run(0.,1.),
  _limit(limit), _safety(20.0), _sigma_u_Delta_t05(sigma_u * sqrt(Deltat_sec)), _sigma_v_Delta_tn05(sigma_v / sqrt(Deltat_sec)), _Pplat(&Oplat) {

  _M << _s * _dist_acft(_gen_acft), _m * _dist_acft(_gen_acft), _m * _dist_acft(_gen_acft),
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

    _IM = _M + Eigen::Matrix3d::Identity();
    _B0Nu0 << _B0 * _dist_run(_gen_run), _B0 * _dist_run(_gen_run), _B0 * _dist_run(_gen_run);
    _Sum_Nui << 0., 0., 0.;
}
/* constructor based on bias drift, white noise, scale factor, cross coupling, bias offset, time interval between measurements,
 * aircraft seed (distinguishes among different aircraft, this is, scale factor and cross coupling), and run seed
 * (distinguishes among different runs of the same aircraft, this is, bias drift, bias offset, bias drift, and
 * white noise), and platform frame. */

Eigen::Vector3d sens::sens_triple_gyr::eval_noB_noM_noR(const Eigen::Vector3d& w_ibb_rps, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    this->grow_random_walk(_Sum_Nui(0));
    this->grow_random_walk(_Sum_Nui(1));
    this->grow_random_walk(_Sum_Nui(2));
    Obias = Eigen::Vector3d::Zero();
    Oscalecross = Eigen::Vector3d::Zero();
    Oerror = Obias + Oscalecross;
    return w_ibb_rps + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

Eigen::Vector3d sens::sens_triple_gyr::eval_noM_noR(const Eigen::Vector3d& w_ibb_rps, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    this->grow_random_walk(_Sum_Nui(0));
    this->grow_random_walk(_Sum_Nui(1));
    this->grow_random_walk(_Sum_Nui(2));
    Obias = _B0Nu0 + _sigma_u_Delta_t05 * _Sum_Nui;
    Oscalecross = Eigen::Vector3d::Zero();
    Oerror = Obias + Oscalecross;
    return w_ibb_rps + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

Eigen::Vector3d sens::sens_triple_gyr::eval_noR(const Eigen::Vector3d& w_ibb_rps, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    this->grow_random_walk(_Sum_Nui(0));
    this->grow_random_walk(_Sum_Nui(1));
    this->grow_random_walk(_Sum_Nui(2));
    Obias = _B0Nu0 + _sigma_u_Delta_t05 * _Sum_Nui;
    Oscalecross = _M * w_ibb_rps;
    Oerror = Obias + Oscalecross;
    return w_ibb_rps + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

Eigen::Vector3d sens::sens_triple_gyr::eval_complete(const Eigen::Vector3d& w_ibb_rps, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    // this function assumes that R_bp_truth and R_bp_est coincide, and hence there is no need to transform the input back and forth to obtain the same result.
    // if this is not the case, it is necessary to replace w_ibb_rps by _R_bp_est * _R_pb_truth * w_ibb_rps in the last line
    this->grow_random_walk(_Sum_Nui(0));
    this->grow_random_walk(_Sum_Nui(1));
    this->grow_random_walk(_Sum_Nui(2));
    Obias = _B0Nu0 + _sigma_u_Delta_t05 * _Sum_Nui;
    Oscalecross = _Pplat->get_R_bp_est() * (_M * _Pplat->get_R_pb_truth() * w_ibb_rps);
    Oerror = Obias + Oscalecross;
    return w_ibb_rps + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

void sens::sens_triple_gyr::grow_random_walk(double& y) {
    if ((_limit - y) < _safety) {
        y += (_dist_run(_gen_run) - 0.1 * (std::fabs(_limit - _safety - y)) / _safety);
        return;
    }
    if ((y + _limit) < _safety) {
        y += (_dist_run(_gen_run) + 0.1 * (std::fabs(-_limit + _safety - y)) / _safety);
        return;
    }
    y += _dist_run(_gen_run);
}
/* modifies the input variable by adding one stop of a random walk */

sens::sens_triple_gyr* sens::sens_triple_gyr::create_gyroscope(sens::logic::GYR_ID gyr_id, sens::logic::BAND_ID band_id, const int& acft_seed, const int& run_seed, const sens::platform& Oplat, const double& Deltat_sec) {
    double limit = 0.;
    switch (band_id) {
        case sens::logic::band_id_base:
            limit = 100.0;
            break;
        case sens::logic::band_id_bigger:
            limit = 300.0;
            break;
        case sens::logic::band_id_biggest:
            limit = 1000.0;
            break;
        case sens::logic::band_id_size:
            throw std::runtime_error("Random walk band id not available");
        default:
            throw std::runtime_error("Random walk band id not available");
    }

    sens::sens_triple_gyr* Pres = nullptr;
    double d2r = math::constant::D2R();
    switch(gyr_id) {
        case sens::logic::gyr_id_zero:
            Pres = new sens::sens_triple_gyr(0., 0., 0., 0., 0., limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::gyr_id_base:
            Pres = new sens::sens_triple_gyr(1.42e-4 * d2r, 4.30e-3 * d2r, 1.50e-5, 4.35e-5, 2.00e-1 * d2r, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::gyr_id_better:
            Pres = new sens::sens_triple_gyr(1.38e-5 * d2r, 2.50e-3 * d2r, 5.00e-6, 1.50e-5, 3.00e-2 * d2r, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::gyr_id_worse:
            Pres = new sens::sens_triple_gyr(5.00e-4 * d2r, 8.00e-3 * d2r, 5.00e-5, 1.50e-4, 7.50e-1 * d2r, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::gyr_id_worst:
            Pres = new sens::sens_triple_gyr(1.50e-3 * d2r, 2.50e-2 * d2r, 1.00e-4, 4.50e-4, 1.50e-0 * d2r, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::gyr_id53:
            Pres = new sens::sens_triple_gyr(1.39e-6 * d2r, 2.00e-4 * d2r, 2.50e-6, 5.00e-6, 5.55e-4 * d2r, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::gyr_id54:
            Pres = new sens::sens_triple_gyr(4.17e-7 * d2r, 5.00e-5 * d2r, 2.00e-6, 4.00e-6, 5.55e-5 * d2r, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::gyr_id_noise_only:
            Pres = new sens::sens_triple_gyr(0.0 * d2r, 4.30e-3 * d2r, 0.0, 0.0, 0.0 * d2r, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::gyr_size:
            throw std::runtime_error("Gyroscope model not available");
        default:
            throw std::runtime_error("Gyroscope model not available");
    }
    return Pres;
}
/* create gyroscope model */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////












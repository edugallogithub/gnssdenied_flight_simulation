#include "sens_triple_acc.h"
#include "platform.h"
#include "../acft/iner.h"

#include <iostream>

// CLASS SENS_TRIPLE_ACC
// =====================
// =====================

sens::sens_triple_acc::sens_triple_acc(const double& sigma_u, const double& sigma_v, const double& s, const double& m, const double& B0, const double& limit, const double& Deltat_sec, const int& acft_seed, const int& run_seed, const sens::platform& Oplat)
: _sigma_u(sigma_u), _sigma_v(sigma_v), _s(s), _m(m), _B0(B0), _gen_acft(acft_seed), _gen_run(run_seed), _Deltat_sec(Deltat_sec), _dist_acft(0.,1.), _dist_run(0.,1.),
  _limit(limit), _safety(20.0), _sigma_u_Delta_t05(sigma_u * sqrt(Deltat_sec)), _sigma_v_Delta_tn05(sigma_v / sqrt(Deltat_sec)), _Pplat(&Oplat) {

    _M << _s * _dist_acft(_gen_acft), 0.0,                        0.0,
          _m * _dist_acft(_gen_acft), _s * _dist_acft(_gen_acft), 0.0,
          _m * _dist_acft(_gen_acft), _m * _dist_acft(_gen_acft), _s * _dist_acft(_gen_acft);

    // we need 9 execution of _dist_run to bias results below are the same as they were when _M
    // was filled up based on _dist_run and not _dist_acft. Because of some temporary errors,
    // they were 9 and not six. Do not worry.
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

Eigen::Vector3d sens::sens_triple_acc::eval_noB_noM_noR(const Eigen::Vector3d& f_ibb_mps2, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    this->grow_random_walk(_Sum_Nui(0));
    this->grow_random_walk(_Sum_Nui(1));
    this->grow_random_walk(_Sum_Nui(2));
    Obias = Eigen::Vector3d::Zero();
    Oscalecross = Eigen::Vector3d::Zero();
    Oerror = Obias + Oscalecross;
    return f_ibb_mps2 + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

Eigen::Vector3d sens::sens_triple_acc::eval_noM_noR(const Eigen::Vector3d& f_ibb_mps2, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    this->grow_random_walk(_Sum_Nui(0));
    this->grow_random_walk(_Sum_Nui(1));
    this->grow_random_walk(_Sum_Nui(2));
    Obias = _B0Nu0 + _sigma_u_Delta_t05 * _Sum_Nui;
    Oscalecross = Eigen::Vector3d::Zero();
    Oerror = Obias + Oscalecross;
    return f_ibb_mps2 + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

Eigen::Vector3d sens::sens_triple_acc::eval_noR(const Eigen::Vector3d& f_ibb_mps2, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror) {
    this->grow_random_walk(_Sum_Nui(0));
    this->grow_random_walk(_Sum_Nui(1));
    this->grow_random_walk(_Sum_Nui(2));
    Obias = _B0Nu0 + _sigma_u_Delta_t05 * _Sum_Nui;
    Oscalecross = _M * f_ibb_mps2;
    Oerror = Obias + Oscalecross;
    return f_ibb_mps2 + Oerror + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

Eigen::Vector3d sens::sens_triple_acc::eval_complete(const Eigen::Vector3d& f_ibb_mps2, Eigen::Vector3d& Obias, Eigen::Vector3d& Oscalecross, Eigen::Vector3d& Oerror,
                                                     const Eigen::Vector3d& w_ibb_rps_truth, const Eigen::Vector3d& w_ibb_rps_est,
                                                     const Eigen::Vector3d& T_bpb_m_truth, const Eigen::Vector3d& T_bpb_m_est,
                                                     const Eigen::Vector3d& alpha_ibb_rps2_truth, const Eigen::Vector3d& alpha_ibb_rps2_est) {
    this->grow_random_walk(_Sum_Nui(0));
    this->grow_random_walk(_Sum_Nui(1));
    this->grow_random_walk(_Sum_Nui(2));
    Obias = _B0Nu0 + _sigma_u_Delta_t05 * _Sum_Nui;

    Eigen::Vector3d lever_mps2_truth  = w_ibb_rps_truth.cross(w_ibb_rps_truth.cross(T_bpb_m_truth)) + alpha_ibb_rps2_truth.cross(T_bpb_m_truth);
    Eigen::Vector3d lever_mps2_est    = w_ibb_rps_est.cross  (w_ibb_rps_est.cross  (T_bpb_m_est))   + alpha_ibb_rps2_est.cross  (T_bpb_m_est);

    Oscalecross = _Pplat->get_R_bp_est() * (_IM * _Pplat->get_R_pb_truth() * (f_ibb_mps2 + lever_mps2_truth)) - lever_mps2_est - f_ibb_mps2;
    Oerror = Obias + Oscalecross;

    return f_ibb_mps2 + Obias + Oscalecross + _sigma_v_Delta_tn05 * Eigen::Vector3d(_dist_run(_gen_run), _dist_run(_gen_run),_dist_run(_gen_run));
}
/* return new sensor measurement based on sensor input, also filling up the bias, scale cross error, and their sum (full error minus noise) */

void sens::sens_triple_acc::grow_random_walk(double& y) {
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

sens::sens_triple_acc* sens::sens_triple_acc::create_accelerometer(sens::logic::ACC_ID acc_id, sens::logic::BAND_ID band_id, const int& acft_seed, const int& run_seed, const sens::platform& Oplat, const double& Deltat_sec) {
    double limit;
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

    sens::sens_triple_acc* Pres = nullptr;
    switch(acc_id) {
        case sens::logic::acc_id_zero:
            Pres = new sens::sens_triple_acc(0., 0., 0., 0., 0., limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::acc_id_base:
            Pres = new sens::sens_triple_acc(6.86e-5, 4.83e-4, 5.00e-5, 3.05e-5, 1.57e-1, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::acc_id_better:
            Pres = new sens::sens_triple_acc(4.90e-5, 3.30e-4, 1.50e-5, 1.50e-5, 1.96e-2, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::acc_id_worse:
            Pres = new sens::sens_triple_acc(8.50e-5, 5.00e-4, 8.50e-5, 5.00e-5, 4.50e-1, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::acc_id_worst:
            Pres = new sens::sens_triple_acc(1.20e-4, 6.50e-4, 1.40e-4, 9.50e-5, 8.50e-1, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::acc_id53:
            Pres = new sens::sens_triple_acc(2.94e-5, 2.50e-4, 5.00e-6, 5.00e-6, 2.94e-3, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::acc_id_noise_only:
            Pres = new sens::sens_triple_acc(0.0, 4.83e-4, 0.0, 0.0, 0.0, limit, Deltat_sec, acft_seed, run_seed, Oplat);
            break;
        case sens::logic::acc_size:
            throw std::runtime_error("Accelerometer model not available");
        default:
            throw std::runtime_error("Accelerometer model not available");
    }
    return Pres;
}
/* create accelerometer model */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////












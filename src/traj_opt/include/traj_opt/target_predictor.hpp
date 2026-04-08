/*
    Target Predictor — interface between EKF state estimation and
    the trajectory optimizer.

    Responsibilities:
      1. Wrap CtrvEkf: drive predict/update from external measurements
      2. Generate a discrete prediction sequence by CTRV forward propagation
      3. Provide interpolated queries: getPos(t), getVel(t), getNormal(t), etc.
      4. Propagate covariance along prediction horizon (for future innovation V1)

    The prediction sequence is generated once per planning cycle and stays
    frozen during a single L-BFGS optimization — the optimizer reads from it
    but does not modify it.

    Graceful degradation: when EKF is not yet initialized, the predictor
    can be seeded with a constant-velocity model (car_p + car_v * t) so that
    the optimizer behaves identically to the original codebase.

    Reference: T-RO 2024 extension paper, Section III-C.
*/

#pragma once

#include <Eigen/Dense>
#include <cassert>
#include <cmath>
#include <vector>

#include "traj_opt/ctrv_ekf.hpp"

namespace traj_opt {

// A single point on the predicted target trajectory
struct PredictedState {
  double time;               // time relative to prediction start
  Eigen::Vector3d pos;       // position in world frame
  Eigen::Vector3d vel;       // 3D velocity (from v_h, θ, v_v)
  Eigen::Vector3d normal;    // landing surface normal z_s = R_z(θ) · z̄_s
  double yaw;                // heading angle θ
  CtrvEkf::StateMat cov;    // propagated covariance P(t) (for innovation V1)
};

class TargetPredictor {
 public:
  TargetPredictor() = default;

  // ================================================================
  //  Configuration (call before first measurement)
  // ================================================================

  // Set EKF noise parameters
  inline void setProcessNoise(const CtrvEkf::StateMat& Q) {
    ekf_.setProcessNoise(Q);
  }
  inline void setMeasurementNoise(const Eigen::Matrix3d& R) {
    ekf_.setMeasurementNoise(R);
  }

  // Set the platform surface normal in body frame.
  // Default is (0,0,1) = horizontal platform.
  // For a tilted platform, set this to the actual normal in vehicle frame.
  inline void setSurfaceNormal(const Eigen::Vector3d& z_bar_s) {
    z_bar_s_ = z_bar_s.normalized();
  }

  // ================================================================
  //  Measurement input (called from ROS callback)
  // ================================================================

  inline void feedMeasurement(const Eigen::Vector3d& meas_pos,
                              double timestamp) {
    ekf_.processMeasurement(meas_pos, timestamp);
  }

  // ================================================================
  //  Constant-velocity fallback (for graceful degradation)
  // ================================================================

  // Seed the predictor with a constant-velocity initial state.
  // Once EKF receives real measurements it will naturally take over.
  inline void seedConstantVelocity(const Eigen::Vector3d& pos,
                                   const Eigen::Vector3d& vel,
                                   const Eigen::Vector3d& surface_normal,
                                   double timestamp) {
    // Construct a CTRV state from pos + vel
    double vh = vel.head<2>().norm();
    double yaw = std::atan2(vel.y(), vel.x());
    double vv = vel.z();

    CtrvEkf::StateVec x0;
    x0 << pos.x(), pos.y(), pos.z(), yaw, vh, vv, 0.0;

    // Large covariance = low confidence, EKF will quickly converge to real data
    CtrvEkf::StateMat P0 = CtrvEkf::StateMat::Identity() * 100.0;
    P0.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 0.01;

    ekf_.initFromState(x0, P0, timestamp);
    setSurfaceNormal(surface_normal);
  }

  // ================================================================
  //  Prediction generation (called once per planning cycle)
  // ================================================================

  // Generate prediction sequence from current EKF state (or CV fallback).
  // duration: total prediction horizon in seconds
  // dt:       time step between prediction points
  inline void generatePrediction(double duration, double dt) {
    assert(dt > 0.0 && duration > 0.0);
    pred_dt_ = dt;
    pred_duration_ = duration;

    int n_steps = static_cast<int>(std::ceil(duration / dt)) + 1;
    pred_seq_.resize(n_steps);

    if (!ekf_.initialized()) {
      // No state estimate yet — fill with zeros, caller should check ready()
      for (int i = 0; i < n_steps; ++i) {
        auto& s = pred_seq_[i];
        s.time = i * dt;
        s.pos.setZero();
        s.vel.setZero();
        s.normal = z_bar_s_;
        s.yaw = 0.0;
        s.cov = CtrvEkf::StateMat::Identity() * 1e6;
      }
      return;
    }
    generateCTRV(n_steps, dt);
  }

  // ================================================================
  //  Query interface (called from optimizer, time relative to pred start)
  // ================================================================

  inline Eigen::Vector3d getPos(double t) const {
    return interpolate(t, [](const PredictedState& s) { return s.pos; });
  }

  inline Eigen::Vector3d getVel(double t) const {
    return interpolate(t, [](const PredictedState& s) { return s.vel; });
  }

  inline Eigen::Vector3d getNormal(double t) const {
    return computeNormal(getYaw(t));
  }

  inline double getYaw(double t) const {
    if (pred_seq_.empty()) return 0.0;
    int idx;
    double alpha;
    timeToIndexAlpha(t, idx, alpha);
    double y0 = pred_seq_[idx].yaw;
    double y1 = pred_seq_[idx + 1].yaw;
    // Shortest-path angular interpolation
    double diff = y1 - y0;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    double result = y0 + alpha * diff;
    while (result > M_PI) result -= 2.0 * M_PI;
    while (result < -M_PI) result += 2.0 * M_PI;
    return result;
  }

  inline CtrvEkf::StateMat getCovariance(double t) const {
    if (pred_seq_.empty()) return CtrvEkf::StateMat::Identity();
    int idx;
    double alpha;
    timeToIndexAlpha(t, idx, alpha);
    // Linear interpolation on covariance matrices (sufficient for frozen weights)
    return pred_seq_[idx].cov * (1.0 - alpha) + pred_seq_[idx + 1].cov * alpha;
  }

  // ================================================================
  //  State accessors
  // ================================================================

  inline bool ready() const { return ekf_.initialized(); }
  inline const CtrvEkf::StateVec& ekfState() const { return ekf_.state(); }
  inline const CtrvEkf::StateMat& ekfCovariance() const { return ekf_.covariance(); }
  inline const std::vector<PredictedState>& predictionSeq() const { return pred_seq_; }
  inline double predDt() const { return pred_dt_; }
  inline double predDuration() const { return pred_duration_; }

  inline void reset() {
    ekf_.reset();
    pred_seq_.clear();
    pred_dt_ = 0.0;
    pred_duration_ = 0.0;
  }

 private:
  // ----------------------------------------------------------------
  //  CTRV forward propagation (Phase A prediction)
  // ----------------------------------------------------------------
  inline void generateCTRV(int n_steps, double dt) {
    // Start from current EKF estimate
    CtrvEkf::StateVec x = ekf_.state();
    CtrvEkf::StateMat P = ekf_.covariance();

    for (int i = 0; i < n_steps; ++i) {
      auto& s = pred_seq_[i];
      s.time = i * dt;
      s.pos = Eigen::Vector3d(x(Ctrv::X), x(Ctrv::Y), x(Ctrv::Z));
      s.vel = toVelocity3D(x);
      s.yaw = CtrvEkf::normalizeAngle(x(Ctrv::YAW));
      s.normal = computeNormal(x(Ctrv::YAW));
      s.cov = P;

      // Propagate to next step (pure predict, no update — P grows)
      if (i < n_steps - 1) {
        CtrvEkf::StateMat F = jacobianFromState(x, dt);
        x = transitionFromState(x, dt);
        P = F * P * F.transpose() + ekf_.processNoise() * dt;
      }
    }
  }

  // ----------------------------------------------------------------
  //  CTRV state transition (duplicated from CtrvEkf for forward prop)
  //  These operate on an arbitrary state vector, not the EKF's internal state.
  // ----------------------------------------------------------------
  static inline CtrvEkf::StateVec transitionFromState(
      const CtrvEkf::StateVec& x, double dt) {
    const double yaw = x(Ctrv::YAW);
    const double vh = x(Ctrv::VH);
    const double w = x(Ctrv::OMEGA);
    const double yaw1 = yaw + w * dt;

    CtrvEkf::StateVec xn = x;
    if (std::fabs(w) > 1e-4) {
      const double vw = vh / w;
      xn(Ctrv::X) = x(Ctrv::X) + vw * (std::sin(yaw1) - std::sin(yaw));
      xn(Ctrv::Y) = x(Ctrv::Y) + vw * (-std::cos(yaw1) + std::cos(yaw));
    } else {
      xn(Ctrv::X) = x(Ctrv::X) + vh * std::cos(yaw) * dt;
      xn(Ctrv::Y) = x(Ctrv::Y) + vh * std::sin(yaw) * dt;
    }
    xn(Ctrv::Z) = x(Ctrv::Z) + x(Ctrv::VV) * dt;
    xn(Ctrv::YAW) = yaw1;
    return xn;
  }

  static inline CtrvEkf::StateMat jacobianFromState(
      const CtrvEkf::StateVec& x, double dt) {
    const double yaw = x(Ctrv::YAW);
    const double vh = x(Ctrv::VH);
    const double w = x(Ctrv::OMEGA);
    const double yaw1 = yaw + w * dt;
    const double s0 = std::sin(yaw), c0 = std::cos(yaw);
    const double s1 = std::sin(yaw1), c1 = std::cos(yaw1);

    CtrvEkf::StateMat F = CtrvEkf::StateMat::Identity();

    if (std::fabs(w) > 1e-4) {
      const double w_inv = 1.0 / w;
      const double vw = vh * w_inv;
      F(Ctrv::X, Ctrv::YAW) = vw * (c1 - c0);
      F(Ctrv::X, Ctrv::VH) = w_inv * (s1 - s0);
      F(Ctrv::X, Ctrv::OMEGA) = vw * (c1 * dt - (s1 - s0) * w_inv);
      F(Ctrv::Y, Ctrv::YAW) = vw * (s1 - s0);
      F(Ctrv::Y, Ctrv::VH) = w_inv * (-c1 + c0);
      F(Ctrv::Y, Ctrv::OMEGA) = vw * (s1 * dt - (-c1 + c0) * w_inv);
    } else {
      F(Ctrv::X, Ctrv::YAW) = -vh * s0 * dt;
      F(Ctrv::X, Ctrv::VH) = c0 * dt;
      F(Ctrv::Y, Ctrv::YAW) = vh * c0 * dt;
      F(Ctrv::Y, Ctrv::VH) = s0 * dt;
    }
    F(Ctrv::Z, Ctrv::VV) = dt;
    F(Ctrv::YAW, Ctrv::OMEGA) = dt;
    return F;
  }

  // ----------------------------------------------------------------
  //  Helpers
  // ----------------------------------------------------------------

  // Convert CTRV state to 3D velocity vector
  static inline Eigen::Vector3d toVelocity3D(const CtrvEkf::StateVec& x) {
    return Eigen::Vector3d(x(Ctrv::VH) * std::cos(x(Ctrv::YAW)),
                           x(Ctrv::VH) * std::sin(x(Ctrv::YAW)),
                           x(Ctrv::VV));
  }

  // Compute world-frame surface normal from yaw angle
  //   z_s = R_z(θ) · z̄_s
  inline Eigen::Vector3d computeNormal(double yaw) const {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    Eigen::Matrix3d Rz;
    Rz << cy, -sy, 0,
           sy,  cy, 0,
            0,   0, 1;
    return Rz * z_bar_s_;
  }

  // Time → index + interpolation alpha
  inline void timeToIndexAlpha(double t, int& idx, double& alpha) const {
    int n = static_cast<int>(pred_seq_.size());
    // Clamp to valid range
    if (t <= 0.0) {
      idx = 0;
      alpha = 0.0;
      return;
    }
    double float_idx = t / pred_dt_;
    idx = static_cast<int>(float_idx);
    if (idx >= n - 1) {
      idx = n - 2;
      alpha = 1.0;
      return;
    }
    alpha = float_idx - idx;
  }

  // Generic linear interpolation on prediction sequence
  template <typename Func>
  inline Eigen::Vector3d interpolate(double t, Func accessor) const {
    if (pred_seq_.empty()) return Eigen::Vector3d::Zero();
    int idx;
    double alpha;
    timeToIndexAlpha(t, idx, alpha);
    return accessor(pred_seq_[idx]) * (1.0 - alpha) +
           accessor(pred_seq_[idx + 1]) * alpha;
  }

  // ================================================================
  //  Member variables
  // ================================================================
  CtrvEkf ekf_;

  // Surface normal in platform body frame (default: horizontal)
  Eigen::Vector3d z_bar_s_{0, 0, 1};

  // Prediction sequence (frozen during one optimization cycle)
  std::vector<PredictedState> pred_seq_;
  double pred_dt_ = 0.0;
  double pred_duration_ = 0.0;

};

}  // namespace traj_opt

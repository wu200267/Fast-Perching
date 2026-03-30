/*
    CTRV + Extended Kalman Filter for target platform state estimation.

    State vector χ = (x, y, z, θ, v_h, v_v, ω)ᵀ  (7-dim)
      x, y, z  : position in world frame
      θ        : heading angle (yaw) in world frame
      v_h      : horizontal speed (signed; positive = forward along θ)
      v_v      : vertical speed (for uphill/downhill)
      ω        : yaw rate

    Measurement vector z = (x, y, z)ᵀ  (3-dim, position only)

    Reference: T-RO 2024 extension paper, Section III-C.
*/

#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace traj_opt {

// State vector indices — avoid magic numbers throughout the codebase
struct Ctrv {
  enum { X = 0, Y, Z, YAW, VH, VV, OMEGA, DIM = 7 };
};

class CtrvEkf {
 public:
  static constexpr int DIM = Ctrv::DIM;
  using StateVec = Eigen::Matrix<double, DIM, 1>;
  using StateMat = Eigen::Matrix<double, DIM, DIM>;
  using MeasVec = Eigen::Vector3d;
  using MeasMat = Eigen::Matrix<double, 3, DIM>;

  CtrvEkf() {
    x_.setZero();
    P_ = StateMat::Identity();
    Q_ = StateMat::Identity() * 0.1;
    R_ = Eigen::Matrix3d::Identity() * 0.01;
  }

  // ----------------------------------------------------------------
  //  Initialization
  // ----------------------------------------------------------------

  // Initialize from first position measurement.
  // Velocities and yaw rate are unknown → large initial P on those dims.
  inline void init(const MeasVec& first_pos, double timestamp) {
    x_.setZero();
    x_(Ctrv::X) = first_pos.x();
    x_(Ctrv::Y) = first_pos.y();
    x_(Ctrv::Z) = first_pos.z();

    // Position is known from measurement; dynamics states are unknown
    P_.setZero();
    P_(Ctrv::X, Ctrv::X) = R_(0, 0);       // position uncertainty = meas noise
    P_(Ctrv::Y, Ctrv::Y) = R_(1, 1);
    P_(Ctrv::Z, Ctrv::Z) = R_(2, 2);
    P_(Ctrv::YAW, Ctrv::YAW) = M_PI * M_PI; // heading completely unknown
    P_(Ctrv::VH, Ctrv::VH) = 10.0;          // horizontal speed unknown
    P_(Ctrv::VV, Ctrv::VV) = 1.0;           // vertical speed unknown
    P_(Ctrv::OMEGA, Ctrv::OMEGA) = 1.0;     // yaw rate unknown

    last_time_ = timestamp;
    initialized_ = true;
  }

  // Initialize from a fully specified state (for external seeding)
  inline void initFromState(const StateVec& x0, const StateMat& P0,
                            double timestamp) {
    x_ = x0;
    P_ = P0;
    last_time_ = timestamp;
    initialized_ = true;
  }

  // ----------------------------------------------------------------
  //  EKF Predict Step
  // ----------------------------------------------------------------

  // Propagate state and covariance forward by dt seconds.
  //   x⁻ = f(x, dt)
  //   P⁻ = F · P · Fᵀ + Q
  inline void predict(double dt) {
    if (!initialized_ || dt <= 0.0) return;

    StateMat F = jacobian(x_, dt);
    x_ = stateTransition(x_, dt);
    P_ = F * P_ * F.transpose() + Q_ * dt;

    // Normalize yaw to [-π, π]
    x_(Ctrv::YAW) = normalizeAngle(x_(Ctrv::YAW));
  }

  // ----------------------------------------------------------------
  //  EKF Update Step
  // ----------------------------------------------------------------

  // Fuse a new position measurement z = (x, y, z).
  //   y = z - H · x⁻           (innovation)
  //   S = H · P⁻ · Hᵀ + R      (innovation covariance)
  //   K = P⁻ · Hᵀ · S⁻¹        (Kalman gain)
  //   x = x⁻ + K · y
  //   P = (I - K · H) · P⁻
  inline void update(const MeasVec& z) {
    if (!initialized_) return;

    // H is trivially [I₃ₓ₃ | 0₃ₓ₄]: we only observe position
    // So H · x = (x, y, z)ᵀ and H · P · Hᵀ = P.topLeftCorner<3,3>()
    MeasVec y = z - x_.head<3>();                       // innovation
    Eigen::Matrix3d S = P_.topLeftCorner<3, 3>() + R_;  // innovation cov

    // K = P⁻ · Hᵀ · S⁻¹  →  (DIM×3) = (DIM×3) · (3×3)⁻¹
    // P⁻ · Hᵀ is simply the first 3 columns of P
    Eigen::Matrix<double, DIM, 3> K = P_.leftCols<3>() * S.inverse();

    x_ += K * y;
    StateMat IKH = StateMat::Identity() - K * H();
    P_ = IKH * P_ * IKH.transpose() + K * R_ * K.transpose();

    // Normalize yaw after update
    x_(Ctrv::YAW) = normalizeAngle(x_(Ctrv::YAW));

  }

  // ----------------------------------------------------------------
  //  Combined predict + update driven by timestamped measurement
  // ----------------------------------------------------------------

  inline void processMeasurement(const MeasVec& z, double timestamp) {
    if (!initialized_) {
      init(z, timestamp);
      return;
    }
    double dt = timestamp - last_time_;
    if (dt <= 0.0) return;  // discard stale / duplicate measurement
    last_time_ = timestamp;
    predict(dt);
    update(z);
  }

  // Normalize angle to [-π, π]
  static inline double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // ----------------------------------------------------------------
  //  Accessors
  // ----------------------------------------------------------------

  inline const StateVec& state() const { return x_; }
  inline const StateMat& covariance() const { return P_; }
  inline bool initialized() const { return initialized_; }
  inline double lastTime() const { return last_time_; }
  inline const StateMat& processNoise() const { return Q_; }

  // ----------------------------------------------------------------
  //  Noise configuration (call before first measurement)
  // ----------------------------------------------------------------

  inline void setProcessNoise(const StateMat& Q) { Q_ = Q; }
  inline void setMeasurementNoise(const Eigen::Matrix3d& R) { R_ = R; }

 private:
  // ================================================================
  //  CTRV State Transition:  x_{k+1} = f(x_k, dt)
  //
  //  When |ω| > ε (turning):
  //    x  += (v_h / ω) · [sin(θ + ω·dt) − sin(θ)]
  //    y  += (v_h / ω) · [−cos(θ + ω·dt) + cos(θ)]
  //  When |ω| ≤ ε (straight line, L'Hôpital):
  //    x  += v_h · cos(θ) · dt
  //    y  += v_h · sin(θ) · dt
  //  Always:
  //    z  += v_v · dt
  //    θ  += ω · dt
  //    v_h, v_v, ω unchanged (CTRV assumption)
  // ================================================================
  inline StateVec stateTransition(const StateVec& x, double dt) const {
    const double px  = x(Ctrv::X);
    const double py  = x(Ctrv::Y);
    const double pz  = x(Ctrv::Z);
    const double yaw = x(Ctrv::YAW);
    const double vh  = x(Ctrv::VH);
    const double vv  = x(Ctrv::VV);
    const double w   = x(Ctrv::OMEGA);

    StateVec xn = x;  // copy — v_h, v_v, ω stay unchanged

    const double yaw1 = yaw + w * dt;

    if (std::fabs(w) > kOmegaEps) {
      // Turning motion
      const double vw = vh / w;
      xn(Ctrv::X) = px + vw * (std::sin(yaw1) - std::sin(yaw));
      xn(Ctrv::Y) = py + vw * (-std::cos(yaw1) + std::cos(yaw));
    } else {
      // Straight-line motion (ω ≈ 0)
      xn(Ctrv::X) = px + vh * std::cos(yaw) * dt;
      xn(Ctrv::Y) = py + vh * std::sin(yaw) * dt;
    }
    xn(Ctrv::Z)   = pz + vv * dt;
    xn(Ctrv::YAW) = yaw1;

    return xn;
  }

  // ================================================================
  //  Jacobian F = ∂f/∂x  (7×7 matrix)
  //
  //  Most entries are 0 or 1 (identity on the diagonal for v_h, v_v, ω).
  //  Non-trivial entries only in the x-row and y-row.
  //
  //  Let θ₁ = θ + ω·dt.  When |ω| > ε:
  //
  //  ∂x/∂θ   = (v_h/ω) · [cos(θ₁) − cos(θ)]
  //  ∂x/∂v_h = (1/ω) · [sin(θ₁) − sin(θ)]
  //  ∂x/∂ω   = (v_h/ω) · [cos(θ₁)·dt − (sin(θ₁)−sin(θ))/ω]
  //
  //  ∂y/∂θ   = (v_h/ω) · [sin(θ₁) − sin(θ)]
  //  ∂y/∂v_h = (1/ω) · [−cos(θ₁) + cos(θ)]
  //  ∂y/∂ω   = (v_h/ω) · [sin(θ₁)·dt − (−cos(θ₁)+cos(θ))/ω]
  //
  //  ∂z/∂v_v = dt
  //
  //  When |ω| ≤ ε (straight line):
  //  ∂x/∂θ   = −v_h · sin(θ) · dt
  //  ∂x/∂v_h = cos(θ) · dt
  //  ∂y/∂θ   = v_h · cos(θ) · dt
  //  ∂y/∂v_h = sin(θ) · dt
  // ================================================================
  inline StateMat jacobian(const StateVec& x, double dt) const {
    const double yaw = x(Ctrv::YAW);
    const double vh  = x(Ctrv::VH);
    const double w   = x(Ctrv::OMEGA);

    StateMat F = StateMat::Identity();  // diagonal = 1 for all 7 states

    const double yaw1 = yaw + w * dt;
    const double s0 = std::sin(yaw);
    const double c0 = std::cos(yaw);
    const double s1 = std::sin(yaw1);
    const double c1 = std::cos(yaw1);

    if (std::fabs(w) > kOmegaEps) {
      const double w_inv = 1.0 / w;
      const double vw = vh * w_inv;

      // x-row partials
      F(Ctrv::X, Ctrv::YAW) = vw * (c1 - c0);
      F(Ctrv::X, Ctrv::VH)  = w_inv * (s1 - s0);
      F(Ctrv::X, Ctrv::OMEGA) = vw * (c1 * dt - (s1 - s0) * w_inv);

      // y-row partials
      F(Ctrv::Y, Ctrv::YAW) = vw * (s1 - s0);
      F(Ctrv::Y, Ctrv::VH)  = w_inv * (-c1 + c0);
      F(Ctrv::Y, Ctrv::OMEGA) = vw * (s1 * dt - (-c1 + c0) * w_inv);
    } else {
      // Straight-line (ω ≈ 0) partials
      F(Ctrv::X, Ctrv::YAW) = -vh * s0 * dt;
      F(Ctrv::X, Ctrv::VH)  = c0 * dt;
      // ∂x/∂ω ≈ 0 for straight line, already 0 from Identity init

      F(Ctrv::Y, Ctrv::YAW) = vh * c0 * dt;
      F(Ctrv::Y, Ctrv::VH)  = s0 * dt;
    }

    // z-row: ∂z/∂v_v = dt
    F(Ctrv::Z, Ctrv::VV) = dt;

    // θ-row: ∂θ/∂ω = dt  (θ₁ = θ + ω·dt)
    F(Ctrv::YAW, Ctrv::OMEGA) = dt;

    return F;
  }

  // Observation matrix H (3×7): trivially selects position components
  inline MeasMat H() const {
    MeasMat Hm = MeasMat::Zero();
    Hm(0, Ctrv::X) = 1.0;
    Hm(1, Ctrv::Y) = 1.0;
    Hm(2, Ctrv::Z) = 1.0;
    return Hm;
  }

  // ================================================================
  //  Member variables
  // ================================================================
  StateVec x_;           // state estimate χ̂
  StateMat P_;           // estimation covariance
  StateMat Q_;           // process noise (per-second; scaled by dt in predict)
  Eigen::Matrix3d R_;    // measurement noise
  double last_time_ = 0.0;
  bool initialized_ = false;

  static constexpr double kOmegaEps = 1e-4;  // threshold for ω ≈ 0 degeneracy
};

}  // namespace traj_opt

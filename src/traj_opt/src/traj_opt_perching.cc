#include <traj_opt/traj_opt.h>

#include <traj_opt/lbfgs_raw.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace traj_opt {

static Eigen::Vector3d car_p_, car_v_;
static Eigen::Vector3d tail_q_v_;
static Eigen::Vector3d g_(0, 0, -9.8);
static Eigen::Vector3d land_v_;
static Eigen::Vector3d v_t_x_, v_t_y_;
static Trajectory init_traj_;
static double init_tail_f_;
static Eigen::Vector2d init_vt_;
static bool initial_guess_ = false;

static double thrust_middle_, thrust_half_;

static double tictoc_innerloop_;
static double tictoc_integral_;

static int iter_times_;
static Eigen::MatrixXd f_DN(const Eigen::Vector3d& x);

struct ConstraintMetrics {
  double max_tau_norm = 0.0;
  double min_tau_norm = std::numeric_limits<double>::infinity();
  double max_abs_omega2 = 0.0;
};

struct ViolationMetrics {
  double viol_tau_max = 0.0;
  double viol_tau_min = 0.0;
  double viol_omega = 0.0;
  double viol_total = 0.0;
};

enum class WinnerEpsLevel {
  kStrict = 0,
  kRelaxed = 1,
  kFallback = 2
};

static const char* winnerEpsLevelToString(const WinnerEpsLevel level) {
  switch (level) {
    case WinnerEpsLevel::kStrict:
      return "strict";
    case WinnerEpsLevel::kRelaxed:
      return "relaxed";
    case WinnerEpsLevel::kFallback:
      return "fallback";
    default:
      return "fallback";
  }
}

static std::string trimCopy(const std::string& s) {
  size_t beg = 0;
  while (beg < s.size() && std::isspace(static_cast<unsigned char>(s[beg]))) {
    ++beg;
  }
  size_t end = s.size();
  while (end > beg && std::isspace(static_cast<unsigned char>(s[end - 1]))) {
    --end;
  }
  return s.substr(beg, end - beg);
}

static std::vector<double> parseVt2Seeds(const std::string& csv,
                                         const std::vector<double>& fallback) {
  std::vector<double> seeds;
  std::stringstream ss(csv);
  std::string token;
  while (std::getline(ss, token, ',')) {
    token = trimCopy(token);
    if (token.empty()) {
      continue;
    }
    try {
      seeds.push_back(std::stod(token));
    } catch (...) {
      seeds.clear();
      break;
    }
  }
  if (seeds.empty()) {
    return fallback;
  }
  return seeds;
}

static double computeOmega2Safe(const Eigen::Vector3d& thrust_raw,
                                const Eigen::Vector3d& jer) {
  constexpr double kThrustNormEps = 1e-3;
  constexpr double kDenEps = 1e-6;
  Eigen::Vector3d thrust = thrust_raw;
  const double thrust_norm = thrust.norm();
  if (thrust_norm < kThrustNormEps) {
    if (thrust_norm < 1e-12) {
      thrust = Eigen::Vector3d(0.0, 0.0, kThrustNormEps);
    } else {
      thrust *= (kThrustNormEps / thrust_norm);
    }
  }
  const Eigen::Vector3d zb = thrust.normalized();
  const Eigen::Vector3d zb_dot = f_DN(thrust) * jer;
  double den = zb.z() + 1.0;
  if (den < kDenEps) {
    den = kDenEps;
  }
  return zb_dot.x() - zb.x() * zb_dot.z() / den;
}

static ConstraintMetrics evaluateConstraintMetricsOnIntGrid(const Trajectory& traj,
                                                            const int K) {
  ConstraintMetrics metrics;
  const int sample_div = std::max(1, K);
  const double total_dur = traj.getTotalDuration();
  double t_base = 0.0;
  for (const auto& piece : traj) {
    const double piece_dur = piece.getDuration();
    for (int j = 0; j <= sample_div; ++j) {
      const double alpha = static_cast<double>(j) / sample_div;
      double t = t_base + alpha * piece_dur;
      if (t > total_dur) {
        t = total_dur;
      }
      const Eigen::Vector3d a = traj.getAcc(t);
      const Eigen::Vector3d jer = traj.getJer(t);
      const Eigen::Vector3d thrust = a - g_;
      const double tau_norm = thrust.norm();
      metrics.max_tau_norm = std::max(metrics.max_tau_norm, tau_norm);
      metrics.min_tau_norm = std::min(metrics.min_tau_norm, tau_norm);
      const double omega2 = computeOmega2Safe(thrust, jer);
      metrics.max_abs_omega2 = std::max(metrics.max_abs_omega2, std::abs(omega2));
    }
    t_base += piece_dur;
  }
  if (!std::isfinite(metrics.min_tau_norm)) {
    metrics.min_tau_norm = 0.0;
  }
  return metrics;
}

static ViolationMetrics evaluateViolations(const ConstraintMetrics& metrics,
                                           const double tau_min,
                                           const double tau_max,
                                           const double omega2_limit) {
  ViolationMetrics viol;
  viol.viol_tau_max = std::max(0.0, metrics.max_tau_norm - tau_max);
  viol.viol_tau_min = std::max(0.0, tau_min - metrics.min_tau_norm);
  viol.viol_omega = std::max(0.0, metrics.max_abs_omega2 - omega2_limit);
  viol.viol_total = viol.viol_tau_max + viol.viol_tau_min + viol.viol_omega;
  return viol;
}

static WinnerEpsLevel classifyWinnerByEps(const ViolationMetrics& viol,
                                          const double eps_strict,
                                          const double eps_relaxed) {
  const bool strict_ok = viol.viol_tau_max <= eps_strict &&
                         viol.viol_tau_min <= eps_strict &&
                         viol.viol_omega <= eps_strict;
  if (strict_ok) {
    return WinnerEpsLevel::kStrict;
  }
  const bool relaxed_ok = viol.viol_tau_max <= eps_relaxed &&
                          viol.viol_tau_min <= eps_relaxed &&
                          viol.viol_omega <= eps_relaxed;
  if (relaxed_ok) {
    return WinnerEpsLevel::kRelaxed;
  }
  return WinnerEpsLevel::kFallback;
}

static bool q2v(const Eigen::Quaterniond& q,
                Eigen::Vector3d& v) {
  Eigen::MatrixXd R = q.toRotationMatrix();
  v = R.col(2);
  return true;
}
static Eigen::Vector3d f_N(const Eigen::Vector3d& x) {
  return x.normalized();
}
static Eigen::MatrixXd f_DN(const Eigen::Vector3d& x) {
  double x_norm_2 = x.squaredNorm();
  return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
}
static Eigen::MatrixXd f_D2N(const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
  double x_norm_2 = x.squaredNorm();
  double x_norm_3 = x_norm_2 * x.norm();
  Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
  return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
}

// SECTION  variables transformation and gradient transmission
static double smoothedL1(const double& x,
                         double& grad) {
  static double mu = 0.01;
  if (x < 0.0) {
    return 0.0;
  } else if (x > mu) {
    grad = 1.0;
    return x - 0.5 * mu;
  } else {
    const double xdmu = x / mu;
    const double sqrxdmu = xdmu * xdmu;
    const double mumxd2 = mu - 0.5 * x;
    grad = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);
    return mumxd2 * sqrxdmu * xdmu;
  }
}
static double smoothed01(const double& x,
                         double& grad) {
  static double mu = 0.01;
  static double mu4 = mu * mu * mu * mu;
  static double mu4_1 = 1.0 / mu4;
  if (x < -mu) {
    grad = 0;
    return 0;
  } else if (x < 0) {
    double y = x + mu;
    double y2 = y * y;
    grad = y2 * (mu - 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu - x) * mu4_1;
  } else if (x < mu) {
    double y = x - mu;
    double y2 = y * y;
    grad = y2 * (mu + 2 * x) * mu4_1;
    return 0.5 * y2 * y * (mu + x) * mu4_1 + 1;
  } else {
    grad = 0;
    return 1;
  }
}

static double expC2(double t) {
  return t > 0.0 ? ((0.5 * t + 1.0) * t + 1.0)
                 : 1.0 / ((0.5 * t - 1.0) * t + 1.0);
}
static double logC2(double T) {
  return T > 1.0 ? (sqrt(2.0 * T - 1.0) - 1.0) : (1.0 - sqrt(2.0 / T - 1.0));
}
static inline double gdT2t(double t) {
  if (t > 0) {
    return t + 1.0;
  } else {
    double denSqrt = (0.5 * t - 1.0) * t + 1.0;
    return (1.0 - t) / (denSqrt * denSqrt);
  }
}

static double forward_thrust(const double& f) {
  return thrust_half_ * sin(f) + thrust_middle_;
  // return f;
}
static void addLayerThrust(const double& f,
                           const double& grad_thrust,
                           double& grad_f) {
  grad_f = thrust_half_ * cos(f) * grad_thrust;
  // grad_f = grad_thrust;
}
static void forwardTailV(const Eigen::Ref<const Eigen::Vector2d>& xy,
                         Eigen::Ref<Eigen::Vector3d> tailV) {
  tailV = land_v_ + xy.x() * v_t_x_ + xy.y() * v_t_y_;
}

// !SECTION variables transformation and gradient transmission

// SECTION object function
static inline double objectiveFunc(void* ptrObj,
                                   const double* x,
                                   double* grad,
                                   const int n) {
  // std::cout << "damn" << std::endl;
  iter_times_++;
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  const int base_no_terminal = obj.dim_t_ + 3 * obj.dim_p_;
  const int tail_f_off = base_no_terminal;
  const int vt_off = tail_f_off + 1;
  const double& t = x[0];
  double& gradt = grad[0];
  Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
  Eigen::Map<Eigen::MatrixXd> gradP(grad + obj.dim_t_, 3, obj.dim_p_);

  // tail_f 在 fixed/non-fixed 两种模式下都参与优化并从 x 读取。
  double tail_f = x[tail_f_off];
  // vt 在 fixed 模式下不在优化向量中，读取本次固定值 active_vt_。
  Eigen::Vector2d vt = obj.active_vt_;
  if (!obj.fix_terminal_state_) {
    vt = Eigen::Map<const Eigen::Vector2d>(x + vt_off);
  }

  double dT = expC2(t);
  Eigen::Vector3d tailV, grad_tailV;
  forwardTailV(vt, tailV);

  Eigen::MatrixXd tailS(3, 4);
  tailS.col(0) = car_p_ + car_v_ * obj.N_ * dT + tail_q_v_ * obj.robot_l_;
  tailS.col(1) = tailV;
  tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_;
  tailS.col(3).setZero();

  auto tic = std::chrono::steady_clock::now();
  obj.mincoOpt_.generate(obj.initS_, tailS, P, dT);

  double cost = obj.mincoOpt_.getTrajSnapCost();
  obj.mincoOpt_.calGrads_CT();

  auto toc = std::chrono::steady_clock::now();
  tictoc_innerloop_ += (toc - tic).count();
  // double cost_with_only_energy = cost;
  // std::cout << "cost of energy: " << cost_with_only_energy << std::endl;

  tic = std::chrono::steady_clock::now();
  obj.addTimeIntPenalty(cost);
  toc = std::chrono::steady_clock::now();
  tictoc_integral_ += (toc - tic).count();

  tic = std::chrono::steady_clock::now();
  obj.mincoOpt_.calGrads_PT();
  toc = std::chrono::steady_clock::now();
  tictoc_innerloop_ += (toc - tic).count();
  // std::cout << "cost of penalty: " << cost - cost_with_only_energy << std::endl;

  obj.mincoOpt_.gdT += obj.mincoOpt_.gdTail.col(0).dot(obj.N_ * car_v_);
  grad_tailV = obj.mincoOpt_.gdTail.col(1);
  double grad_thrust = obj.mincoOpt_.gdTail.col(2).dot(tail_q_v_);

  // tail_f 的梯度在两种模式下都必须回传（tail_f 永远是优化变量）。
  double grad_f = 0.0;
  addLayerThrust(tail_f, grad_thrust, grad_f);
  grad[tail_f_off] = grad_f;

  // fixed 模式下 vt 不在优化向量中，必须无条件跳过 rhoVt 成本/梯度。
  if (!obj.fix_terminal_state_) {
    if (obj.rhoVt_ > -1) {
      Eigen::Map<Eigen::Vector2d> grad_vt(grad + vt_off);
      grad_vt.x() = grad_tailV.dot(v_t_x_);
      grad_vt.y() = grad_tailV.dot(v_t_y_);
      double vt_sqr = vt.squaredNorm();
      cost += obj.rhoVt_ * vt_sqr;
      grad_vt += obj.rhoVt_ * 2 * vt;
    }
  }

  obj.mincoOpt_.gdT += obj.rhoT_;
  cost += obj.rhoT_ * dT;
  gradt = obj.mincoOpt_.gdT * gdT2t(t);

  gradP = obj.mincoOpt_.gdP;

  return cost;
}
// !SECTION object function
static inline int earlyExit(void* ptrObj,
                            const double* x,
                            const double* grad,
                            const double fx,
                            const double xnorm,
                            const double gnorm,
                            const double step,
                            int n,
                            int k,
                            int ls) {
  TrajOpt& obj = *(TrajOpt*)ptrObj;
  if (obj.pause_debug_) {
    const int base_no_terminal = obj.dim_t_ + 3 * obj.dim_p_;
    const int tail_f_off = base_no_terminal;
    const int vt_off = tail_f_off + 1;
    const double& t = x[0];
    Eigen::Map<const Eigen::MatrixXd> P(x + obj.dim_t_, 3, obj.dim_p_);
    // earlyExit 的变量读取规则与 objectiveFunc 保持一致。
    double tail_f = x[tail_f_off];
    Eigen::Vector2d vt = obj.active_vt_;
    if (!obj.fix_terminal_state_) {
      vt = Eigen::Map<const Eigen::Vector2d>(x + vt_off);
    }

    double dT = expC2(t);
    double T = obj.N_ * dT;
    Eigen::Vector3d tailV;
    forwardTailV(vt, tailV);

    Eigen::MatrixXd tailS(3, 4);
    tailS.col(0) = car_p_ + car_v_ * T + tail_q_v_ * obj.robot_l_;
    tailS.col(1) = tailV;
    tailS.col(2) = forward_thrust(tail_f) * tail_q_v_ + g_;
    tailS.col(3).setZero();

    obj.mincoOpt_.generate(obj.initS_, tailS, P, dT);
    auto traj = obj.mincoOpt_.getTraj();
    obj.visPtr_->visualize_traj(traj, "debug_traj");
    std::vector<Eigen::Vector3d> int_waypts;
    for (const auto& piece : traj) {
      const auto& dur = piece.getDuration();
      for (int i = 0; i < obj.K_; ++i) {
        double t = dur * i / obj.K_;
        int_waypts.push_back(piece.getPos(t));
      }
    }
    obj.visPtr_->visualize_pointcloud(int_waypts, "int_waypts");

    // NOTE pause
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // return k > 1e3;
  return 0;
}

static void bvp(const double& t,
                const Eigen::MatrixXd i_state,
                const Eigen::MatrixXd f_state,
                CoefficientMat& coeffMat) {
  double t1 = t;
  double t2 = t1 * t1;
  double t3 = t2 * t1;
  double t4 = t2 * t2;
  double t5 = t3 * t2;
  double t6 = t3 * t3;
  double t7 = t4 * t3;
  CoefficientMat boundCond;
  boundCond.leftCols(4) = i_state;
  boundCond.rightCols(4) = f_state;

  coeffMat.col(0) = (boundCond.col(7) / 6.0 + boundCond.col(3) / 6.0) * t3 +
                    (-2.0 * boundCond.col(6) + 2.0 * boundCond.col(2)) * t2 +
                    (10.0 * boundCond.col(5) + 10.0 * boundCond.col(1)) * t1 +
                    (-20.0 * boundCond.col(4) + 20.0 * boundCond.col(0));
  coeffMat.col(1) = (-0.5 * boundCond.col(7) - boundCond.col(3) / 1.5) * t3 +
                    (6.5 * boundCond.col(6) - 7.5 * boundCond.col(2)) * t2 +
                    (-34.0 * boundCond.col(5) - 36.0 * boundCond.col(1)) * t1 +
                    (70.0 * boundCond.col(4) - 70.0 * boundCond.col(0));
  coeffMat.col(2) = (0.5 * boundCond.col(7) + boundCond.col(3)) * t3 +
                    (-7.0 * boundCond.col(6) + 10.0 * boundCond.col(2)) * t2 +
                    (39.0 * boundCond.col(5) + 45.0 * boundCond.col(1)) * t1 +
                    (-84.0 * boundCond.col(4) + 84.0 * boundCond.col(0));
  coeffMat.col(3) = (-boundCond.col(7) / 6.0 - boundCond.col(3) / 1.5) * t3 +
                    (2.5 * boundCond.col(6) - 5.0 * boundCond.col(2)) * t2 +
                    (-15.0 * boundCond.col(5) - 20.0 * boundCond.col(1)) * t1 +
                    (35.0 * boundCond.col(4) - 35.0 * boundCond.col(0));
  coeffMat.col(4) = boundCond.col(3) / 6.0;
  coeffMat.col(5) = boundCond.col(2) / 2.0;
  coeffMat.col(6) = boundCond.col(1);
  coeffMat.col(7) = boundCond.col(0);

  coeffMat.col(0) = coeffMat.col(0) / t7;
  coeffMat.col(1) = coeffMat.col(1) / t6;
  coeffMat.col(2) = coeffMat.col(2) / t5;
  coeffMat.col(3) = coeffMat.col(3) / t4;
}
static double getMaxOmega(Trajectory& traj) {
  double dt = 0.01;
  double max_omega = 0;
  for (double t = 0; t < traj.getTotalDuration(); t += dt) {
    Eigen::Vector3d a = traj.getAcc(t);
    Eigen::Vector3d j = traj.getJer(t);
    Eigen::Vector3d thrust = a - g_;
    Eigen::Vector3d zb_dot = f_DN(thrust) * j;
    double omega12 = zb_dot.norm();
    if (omega12 > max_omega) {
      max_omega = omega12;
    }
  }
  return max_omega;
}

bool TrajOpt::generate_traj(const Eigen::MatrixXd& iniState,
                            const Eigen::Vector3d& car_p,
                            const Eigen::Vector3d& car_v,
                            const Eigen::Quaterniond& land_q,
                            const int& N,
                            Trajectory& traj,
                            const double& t_replan) {
  // 每次规划前先置无效，避免失败时沿用上一次摘要。
  last_terminal_summary_.valid = false;
  N_ = N;
  // 核对离散化参数是否与论文 Table I 一致：
  // 期望值应为 K=16, N=10, N*K=160。
  std::cout << "[TrajOpt] Discretization check: K=" << K_
            << ", N=" << N_
            << ", N*K=" << (N_ * K_) << std::endl;
  dim_t_ = 1;
  dim_p_ = N_ - 1;
  // 优化向量布局（vt-only fixed）：
  // [ t | P(3*(N-1)) | tail_f | vt_x | vt_y(non-fixed only) ]
  const int base_no_terminal = dim_t_ + 3 * dim_p_;
  const int tail_f_off = base_no_terminal;   // tail_f 永远存在
  const int vt_off = tail_f_off + 1;         // vt 仅在 non-fixed 时存在
  const int base_dim = base_no_terminal + 1; // 含 tail_f，不含 vt
  // 每次调用必须重算维度，禁止复用旧值。
  opt_dim_ = base_dim + (fix_terminal_state_ ? 0 : 2);
  if (print_opt_layout_once_ && !opt_layout_printed_) {
    std::cout << "[TrajOpt] Opt layout: base_no_terminal=" << base_no_terminal
              << ", base_dim=" << base_dim
              << ", opt_dim=" << opt_dim_
              << ", tail_f_off=" << tail_f_off
              << ", vt_off=" << vt_off
              << ", fix_terminal_state=" << (fix_terminal_state_ ? "true" : "false")
              << std::endl;
    opt_layout_printed_ = true;
  }
  x_ = new double[opt_dim_];
  // 统一释放入口，保证所有 return false 路径不泄露。
  auto cleanup_x = [&]() {
    if (x_ != nullptr) {
      delete[] x_;
      x_ = nullptr;
    }
  };

  double& t = x_[0];
  Eigen::Map<Eigen::MatrixXd> P(x_ + dim_t_, 3, dim_p_);
  double guessed_tail_f = 0.0;
  Eigen::Vector2d guessed_vt = Eigen::Vector2d::Zero();
  car_p_ = car_p;
  car_v_ = car_v;
  // std::cout << "land_q: "
  //           << land_q.w() << ","
  //           << land_q.x() << ","
  //           << land_q.y() << ","
  //           << land_q.z() << "," << std::endl;
  q2v(land_q, tail_q_v_);
  thrust_middle_ = (thrust_max_ + thrust_min_) / 2;
  thrust_half_ = (thrust_max_ - thrust_min_) / 2;

  land_v_ = car_v - tail_q_v_ * v_plus_;
  // std::cout << "tail_q_v_: " << tail_q_v_.transpose() << std::endl;

  v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 0, 1));
  if (v_t_x_.squaredNorm() == 0) {
    v_t_x_ = tail_q_v_.cross(Eigen::Vector3d(0, 1, 0));
  }
  v_t_x_.normalize();
  v_t_y_ = tail_q_v_.cross(v_t_x_);
  v_t_y_.normalize();

  // NOTE set boundary conditions
  initS_ = iniState;

  // set initial guess with obvp minimum jerk + rhoT
  mincoOpt_.reset(N_);

  bool opt_once = initial_guess_ && t_replan > 0 && t_replan < init_traj_.getTotalDuration();
  if (opt_once) {
    double init_T = init_traj_.getTotalDuration() - t_replan;
    t = logC2(init_T / N_);
    for (int i = 1; i < N_; ++i) {
      double tt0 = (i * 1.0 / N_) * init_T;
      P.col(i - 1) = init_traj_.getPos(tt0 + t_replan);
    }
    guessed_tail_f = init_tail_f_;
    guessed_vt = init_vt_;
  } else {
    Eigen::MatrixXd bvp_i = initS_;
    Eigen::MatrixXd bvp_f(3, 4);
    bvp_f.col(0) = car_p_;
    bvp_f.col(1) = car_v_;
    bvp_f.col(2) = forward_thrust(guessed_tail_f) * tail_q_v_ + g_;
    bvp_f.col(3).setZero();
    double T_bvp = (bvp_f.col(0) - bvp_i.col(0)).norm() / vmax_;
    CoefficientMat coeffMat;
    double max_omega = 0;
    do {
      T_bvp += 1.0;
      bvp_f.col(0) = car_p_ + car_v_ * T_bvp;
      bvp(T_bvp, bvp_i, bvp_f, coeffMat);
      std::vector<double> durs{T_bvp};
      std::vector<CoefficientMat> coeffs{coeffMat};
      Trajectory traj(durs, coeffs);
      max_omega = getMaxOmega(traj);
    } while (max_omega > 1.5 * omega_max_);
    Eigen::VectorXd tt(8);
    tt(7) = 1.0;
    for (int i = 1; i < N_; ++i) {
      double tt0 = (i * 1.0 / N_) * T_bvp;
      for (int j = 6; j >= 0; j -= 1) {
        tt(j) = tt(j + 1) * tt0;
      }
      P.col(i - 1) = coeffMat * tt;
    }
    t = logC2(T_bvp / N_);
  }

  // 记录本次 vt guess，供 fixed 模式“未显式给 fixed_vt_* 参数”时回退使用。
  active_vt_guess_ = guessed_vt;

  // 在单次 lbfgs 调用前刷新 active_vt_，并在该次优化期间保持不变。
  active_vt_.x() = has_fixed_vt_x_ ? fixed_vt_x_param_ : active_vt_guess_.x();
  active_vt_.y() = has_fixed_vt_y_ ? fixed_vt_y_param_ : active_vt_guess_.y();

  // tail_f 在 fixed/non-fixed 下都参与优化，初值都要写入 x。
  x_[tail_f_off] = guessed_tail_f;
  // vt 仅在 non-fixed 下参与优化，初值写入 x 作为 warm-start。
  if (!fix_terminal_state_) {
    Eigen::Map<Eigen::Vector2d> vt(x_ + vt_off);
    vt = guessed_vt;
  }
  // std::cout << "initial guess >>> t: " << t << std::endl;
  // std::cout << "initial guess >>> tail_f: " << guessed_tail_f << std::endl;
  // std::cout << "initial guess >>> vt: " << guessed_vt.transpose() << std::endl;

  // NOTE optimization
  lbfgs::lbfgs_parameter_t lbfgs_params;
  lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
  lbfgs_params.mem_size = 32;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 0.0;
  lbfgs_params.min_step = 1e-16;
  lbfgs_params.delta = 1e-4;
  lbfgs_params.line_search_type = 0;
  double minObjective = 0.0;
  int opt_ret = 0;
  double winner_seed_vt2 = (!fix_terminal_state_) ? guessed_vt.y() : active_vt_.y();
  WinnerEpsLevel winner_eps_level = WinnerEpsLevel::kFallback;
  double winner_viol_total_int = 0.0;

  struct SeedCandidate {
    double seed_vt2 = 0.0;
    int opt_ret = -1;
    double objective = 0.0;
    ViolationMetrics viol_int;
    std::vector<double> x_opt;
  };

  auto run_single_optimization = [&](double& objective_out, int& ret_out) {
    tictoc_innerloop_ = 0;
    tictoc_integral_ = 0;
    iter_times_ = 0;
    ret_out = lbfgs::lbfgs_optimize(opt_dim_, x_, &objective_out,
                                    &objectiveFunc, nullptr,
                                    &earlyExit, this, &lbfgs_params);
  };

  const bool run_multistart = vt_multistart_enable_ && !fix_terminal_state_;
  const std::string solve_mode = run_multistart ? "multistart" : "single";
  if (run_multistart) {
    std::vector<double> x0(x_, x_ + opt_dim_);
    std::vector<SeedCandidate> candidates;
    auto tic = std::chrono::steady_clock::now();
    for (double seed_vt2 : vt_multistart_vt2_seeds_) {
      std::copy(x0.begin(), x0.end(), x_);
      x_[vt_off + 1] = seed_vt2;

      SeedCandidate cand;
      cand.seed_vt2 = seed_vt2;
      run_single_optimization(cand.objective, cand.opt_ret);
      std::cout << "[TrajOpt][multistart] seed_vt2=" << seed_vt2
                << ", ret=" << cand.opt_ret
                << ", objective=" << cand.objective << std::endl;
      if (cand.opt_ret < 0) {
        continue;
      }

      const double t_seed = x_[0];
      const double dT_seed = expC2(t_seed);
      const double T_seed = N_ * dT_seed;
      Eigen::Map<const Eigen::MatrixXd> P_seed(x_ + dim_t_, 3, dim_p_);
      const double tail_f_seed = x_[tail_f_off];
      const Eigen::Vector2d vt_seed = Eigen::Map<const Eigen::Vector2d>(x_ + vt_off);
      Eigen::Vector3d tailV_seed;
      forwardTailV(vt_seed, tailV_seed);
      Eigen::MatrixXd tailS_seed(3, 4);
      tailS_seed.col(0) = car_p_ + car_v_ * T_seed + tail_q_v_ * robot_l_;
      tailS_seed.col(1) = tailV_seed;
      tailS_seed.col(2) = forward_thrust(tail_f_seed) * tail_q_v_ + g_;
      tailS_seed.col(3).setZero();
      mincoOpt_.generate(initS_, tailS_seed, P_seed, dT_seed);
      const Trajectory seed_traj = mincoOpt_.getTraj();
      const ConstraintMetrics seed_int_metrics = evaluateConstraintMetricsOnIntGrid(seed_traj, K_);
      cand.viol_int = evaluateViolations(seed_int_metrics, thrust_min_, thrust_max_, omega_max_);
      cand.x_opt.assign(x_, x_ + opt_dim_);
      candidates.push_back(std::move(cand));
    }
    auto toc = std::chrono::steady_clock::now();
    std::cout << "[TrajOpt][multistart] total costs: " << (toc - tic).count() * 1e-6
              << "ms, candidates=" << candidates.size() << std::endl;
    if (pause_debug_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    if (candidates.empty()) {
      last_terminal_summary_.valid = false;
      cleanup_x();
      return false;
    }

    auto better_obj = [&](const SeedCandidate& a, const SeedCandidate& b) {
      if (a.objective != b.objective) {
        return a.objective < b.objective;
      }
      if (a.viol_int.viol_total != b.viol_int.viol_total) {
        return a.viol_int.viol_total < b.viol_int.viol_total;
      }
      return a.seed_vt2 < b.seed_vt2;
    };
    auto better_fallback = [&](const SeedCandidate& a, const SeedCandidate& b) {
      if (a.viol_int.viol_total != b.viol_int.viol_total) {
        return a.viol_int.viol_total < b.viol_int.viol_total;
      }
      if (a.objective != b.objective) {
        return a.objective < b.objective;
      }
      return a.seed_vt2 < b.seed_vt2;
    };
    auto feasible_with_eps = [&](const SeedCandidate& cand, const double eps) {
      return cand.viol_int.viol_tau_max <= eps &&
             cand.viol_int.viol_tau_min <= eps &&
             cand.viol_int.viol_omega <= eps;
    };

    const double eps_strict = std::max(0.0, vt_multistart_eps_strict_);
    const double eps_relaxed = std::max(eps_strict, vt_multistart_eps_relaxed_);

    const SeedCandidate* winner = nullptr;
    winner_eps_level = WinnerEpsLevel::kFallback;
    for (const auto& cand : candidates) {
      if (!feasible_with_eps(cand, eps_strict)) {
        continue;
      }
      if (winner == nullptr || better_obj(cand, *winner)) {
        winner = &cand;
      }
    }
    if (winner != nullptr) {
      winner_eps_level = WinnerEpsLevel::kStrict;
    } else {
      for (const auto& cand : candidates) {
        if (!feasible_with_eps(cand, eps_relaxed)) {
          continue;
        }
        if (winner == nullptr || better_obj(cand, *winner)) {
          winner = &cand;
        }
      }
      if (winner != nullptr) {
        winner_eps_level = WinnerEpsLevel::kRelaxed;
      } else {
        for (const auto& cand : candidates) {
          if (winner == nullptr || better_fallback(cand, *winner)) {
            winner = &cand;
          }
        }
      }
    }

    if (winner == nullptr || winner->x_opt.empty()) {
      last_terminal_summary_.valid = false;
      cleanup_x();
      return false;
    }

    std::copy(winner->x_opt.begin(), winner->x_opt.end(), x_);
    minObjective = winner->objective;
    opt_ret = winner->opt_ret;
    winner_seed_vt2 = winner->seed_vt2;
    winner_viol_total_int = winner->viol_int.viol_total;
    std::cout << "[TrajOpt][multistart] winner seed_vt2=" << winner_seed_vt2
              << ", eps_level=" << winnerEpsLevelToString(winner_eps_level)
              << ", viol_total_int=" << winner_viol_total_int
              << ", objective=" << minObjective << std::endl;
  } else {
    auto tic = std::chrono::steady_clock::now();
    run_single_optimization(minObjective, opt_ret);
    auto toc = std::chrono::steady_clock::now();
    std::cout << "\033[32m>ret: " << opt_ret << "\033[0m" << std::endl;
    std::cout << "optmization costs: " << (toc - tic).count() * 1e-6 << "ms" << std::endl;
    if (pause_debug_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    if (opt_ret < 0) {
      last_terminal_summary_.valid = false;
      cleanup_x();
      return false;
    }
  }

  // 取本次最终终端变量：
  // tail_f 始终来自优化向量；vt 在 fixed 用 active_vt_，non-fixed 用优化结果。
  double tail_f_final = x_[tail_f_off];
  Eigen::Vector2d vt_final = active_vt_;
  if (!fix_terminal_state_) {
    vt_final = Eigen::Map<const Eigen::Vector2d>(x_ + vt_off);
  }

  double dT = expC2(t);
  double T = N_ * dT;
  Eigen::Vector3d tailV;
  forwardTailV(vt_final, tailV);
  Eigen::MatrixXd tailS(3, 4);
  tailS.col(0) = car_p_ + car_v_ * T + tail_q_v_ * robot_l_;
  tailS.col(1) = tailV;
  tailS.col(2) = forward_thrust(tail_f_final) * tail_q_v_ + g_;
  tailS.col(3).setZero();
  // std::cout << "tail thrust: " << forward_thrust(tail_f_final) << std::endl;
  // std::cout << tailS << std::endl;
  mincoOpt_.generate(initS_, tailS, P, dT);
  traj = mincoOpt_.getTraj();
  const ConstraintMetrics int_metrics = evaluateConstraintMetricsOnIntGrid(traj, K_);
  const ViolationMetrics int_viol = evaluateViolations(int_metrics, thrust_min_, thrust_max_, omega_max_);
  if (!run_multistart) {
    winner_viol_total_int = int_viol.viol_total;
    const double eps_strict = std::max(0.0, vt_multistart_eps_strict_);
    const double eps_relaxed = std::max(eps_strict, vt_multistart_eps_relaxed_);
    winner_eps_level = classifyWinnerByEps(int_viol, eps_strict, eps_relaxed);
  }

  std::cout << "tailV: " << tailV.transpose() << std::endl;
  std::cout << "max_abs_omega2_int: " << int_metrics.max_abs_omega2 << std::endl;
  std::cout << "max_tau_norm_int: " << int_metrics.max_tau_norm << std::endl;
  std::cout << "min_tau_norm_int: " << int_metrics.min_tau_norm << std::endl;

  // 记录给应用层导出的终端摘要（不做文件 IO）。
  const Eigen::Vector3d vt3d = vt_final.x() * v_t_x_ + vt_final.y() * v_t_y_;
  const double vt1_abs = std::abs(vt_final.x());
  const double vt2_abs = std::abs(vt_final.y());
  last_terminal_summary_.valid = true;
  last_terminal_summary_.vt1 = vt_final.x();
  last_terminal_summary_.vt2 = vt_final.y();
  last_terminal_summary_.vt_norm = vt_final.norm();
  last_terminal_summary_.vt_signed = vt3d.dot(Eigen::Vector3d::UnitZ());
  last_terminal_summary_.vt_primary = (vt1_abs >= vt2_abs) ? vt_final.x() : vt_final.y();
  last_terminal_summary_.vt_primary_axis = (vt1_abs >= vt2_abs) ? 1 : 2;
  last_terminal_summary_.vt3d = vt3d;
  last_terminal_summary_.zd = tail_q_v_;
  last_terminal_summary_.v1 = v_t_x_;
  last_terminal_summary_.v2 = v_t_y_;
  last_terminal_summary_.duration = T;
  last_terminal_summary_.max_tau_norm_int = int_metrics.max_tau_norm;
  last_terminal_summary_.min_tau_norm_int = int_metrics.min_tau_norm;
  last_terminal_summary_.max_abs_omega2_int = int_metrics.max_abs_omega2;
  last_terminal_summary_.solve_mode = solve_mode;
  last_terminal_summary_.winner_seed_vt2 = winner_seed_vt2;
  last_terminal_summary_.winner_objective = minObjective;
  last_terminal_summary_.winner_viol_total_int = winner_viol_total_int;
  last_terminal_summary_.winner_eps_level = winnerEpsLevelToString(winner_eps_level);

  init_traj_ = traj;
  // 保存为下一次 warm-start 来源。
  init_tail_f_ = tail_f_final;
  init_vt_ = vt_final;
  initial_guess_ = true;
  cleanup_x();
  return true;
}

TrajOpt::TerminalSummary TrajOpt::getLastTerminalSummary() const {
  return last_terminal_summary_;
}

void TrajOpt::addTimeIntPenalty(double& cost) {
  Eigen::Vector3d pos, vel, acc, jer, snp;
  Eigen::Vector3d grad_tmp, grad_tmp2, grad_tmp3, grad_p, grad_v, grad_a, grad_j;
  double cost_tmp, cost_inner;
  Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
  double s1, s2, s3, s4, s5, s6, s7;
  double step, alpha;
  Eigen::Matrix<double, 8, 3> gradViola_c;
  double gradViola_t;
  double omg;

  int innerLoop = K_ + 1;
  step = mincoOpt_.t(1) / K_;

  s1 = 0.0;

  for (int j = 0; j < innerLoop; ++j) {
    s2 = s1 * s1;
    s3 = s2 * s1;
    s4 = s2 * s2;
    s5 = s4 * s1;
    s6 = s4 * s2;
    s7 = s4 * s3;
    beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
    beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
    alpha = 1.0 / K_ * j;
    omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

    for (int i = 0; i < N_; ++i) {
      const auto& c = mincoOpt_.c.block<8, 3>(i * 8, 0);

      pos = c.transpose() * beta0;
      vel = c.transpose() * beta1;
      acc = c.transpose() * beta2;
      jer = c.transpose() * beta3;
      snp = c.transpose() * beta4;

      grad_p.setZero();
      grad_v.setZero();
      grad_a.setZero();
      grad_j.setZero();
      grad_tmp3.setZero();
      cost_inner = 0.0;

      if (grad_cost_floor(pos, grad_tmp, cost_tmp)) {
        grad_p += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_v(vel, grad_tmp, cost_tmp)) {
        grad_v += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_thrust(acc, grad_tmp, cost_tmp)) {
        grad_a += grad_tmp;
        cost_inner += cost_tmp;
      }

      if (grad_cost_omega(acc, jer, grad_tmp, grad_tmp2, cost_tmp)) {
        grad_a += grad_tmp;
        grad_j += grad_tmp2;
        cost_inner += cost_tmp;
      }

      if (grad_cost_omega_yaw(acc, jer, grad_tmp, grad_tmp2, cost_tmp)) {
        grad_a += grad_tmp;
        grad_j += grad_tmp2;
        cost_inner += cost_tmp;
      }

      double dur2now = (i + alpha) * mincoOpt_.t(1);
      Eigen::Vector3d car_p = car_p_ + car_v_ * dur2now;
      if (grad_cost_perching_collision(pos, acc, car_p,
                                       grad_tmp, grad_tmp2, grad_tmp3,
                                       cost_tmp)) {
        grad_p += grad_tmp;
        grad_a += grad_tmp2;
        cost_inner += cost_tmp;
      }
      double grad_car_t = grad_tmp3.dot(car_v_);

      gradViola_c = beta0 * grad_p.transpose();
      gradViola_t = grad_p.transpose() * vel;
      gradViola_c += beta1 * grad_v.transpose();
      gradViola_t += grad_v.transpose() * acc;
      gradViola_c += beta2 * grad_a.transpose();
      gradViola_t += grad_a.transpose() * jer;
      gradViola_c += beta3 * grad_j.transpose();
      gradViola_t += grad_j.transpose() * snp;
      gradViola_t += grad_car_t;

      mincoOpt_.gdC.block<8, 3>(i * 8, 0) += omg * step * gradViola_c;
      mincoOpt_.gdT += omg * (cost_inner / K_ + alpha * step * gradViola_t);
      mincoOpt_.gdT += i * omg * step * grad_car_t;
      cost += omg * step * cost_inner;
    }
    s1 += step;
  }
}

TrajOpt::TrajOpt(ros::NodeHandle& nh) {
  // nh.getParam("N", N_);
  nh.getParam("K", K_);
  // load dynamic paramters
  nh.getParam("vmax", vmax_);
  nh.getParam("amax", amax_);
  nh.getParam("thrust_max", thrust_max_);
  nh.getParam("thrust_min", thrust_min_);
  nh.getParam("omega_max", omega_max_);
  nh.getParam("omega_yaw_max", omega_yaw_max_);
  nh.getParam("v_plus", v_plus_);
  nh.getParam("robot_l", robot_l_);
  nh.getParam("robot_r", robot_r_);
  nh.getParam("platform_r", platform_r_);
  nh.getParam("rhoT", rhoT_);
  nh.getParam("rhoVt", rhoVt_);
  nh.getParam("rhoP", rhoP_);
  nh.getParam("rhoV", rhoV_);
  nh.getParam("rhoA", rhoA_);
  nh.getParam("rhoThrust", rhoThrust_);
  nh.getParam("rhoOmega", rhoOmega_);
  nh.getParam("rhoPerchingCollision", rhoPerchingCollision_);
  nh.param("vt_multistart_enable", vt_multistart_enable_, false);
  nh.param("vt_multistart_eps_strict", vt_multistart_eps_strict_, 1e-3);
  nh.param("vt_multistart_eps_relaxed", vt_multistart_eps_relaxed_, 1e-2);
  std::string vt2_seeds_csv;
  nh.param("vt_multistart_vt2_seeds",
           vt2_seeds_csv,
           std::string("0.0,0.1,-0.1,0.4,-0.4"));
  vt_multistart_vt2_seeds_ = parseVt2Seeds(
      vt2_seeds_csv,
      std::vector<double>{0.0, 0.1, -0.1, 0.4, -0.4});
  // hard-fix 相关参数：
  // - fix_terminal_state=false: 与旧版本完全一致；
  // - fix_terminal_state=true : 仅从优化变量中移除 vt，tail_f 仍参与优化。
  nh.param("fix_terminal_state", fix_terminal_state_, false);
  nh.param("print_opt_layout_once", print_opt_layout_once_, false);
  // getParam 返回值用于区分“参数未给”与“参数给了且值为 0”两种情况。
  has_fixed_vt_x_ = nh.getParam("fixed_vt_x", fixed_vt_x_param_);
  has_fixed_vt_y_ = nh.getParam("fixed_vt_y", fixed_vt_y_param_);
  double deprecated_fixed_tail_f = 0.0;
  if (nh.getParam("fixed_tail_f", deprecated_fixed_tail_f)) {
    std::cout << "[TrajOpt][deprecated] 'fixed_tail_f' is ignored in vt-only strict mode. "
                 "received fixed_tail_f=" << deprecated_fixed_tail_f << std::endl;
  }
  std::cout << "[TrajOpt] Terminal-fix config: fix_terminal_state="
            << (fix_terminal_state_ ? "true" : "false")
            << ", has_fixed_vt_x=" << (has_fixed_vt_x_ ? "true" : "false")
            << ", fixed_vt_x_param=" << fixed_vt_x_param_
            << ", has_fixed_vt_y=" << (has_fixed_vt_y_ ? "true" : "false")
            << ", fixed_vt_y_param=" << fixed_vt_y_param_
            << ", print_opt_layout_once=" << (print_opt_layout_once_ ? "true" : "false")
            << std::endl;
  std::ostringstream vt2_seed_stream;
  for (size_t i = 0; i < vt_multistart_vt2_seeds_.size(); ++i) {
    if (i > 0) {
      vt2_seed_stream << ",";
    }
    vt2_seed_stream << vt_multistart_vt2_seeds_[i];
  }
  std::cout << "[TrajOpt] vt multistart: enable=" << (vt_multistart_enable_ ? "true" : "false")
            << ", vt2_seeds=[" << vt2_seed_stream.str() << "]"
            << ", eps_strict=" << vt_multistart_eps_strict_
            << ", eps_relaxed=" << vt_multistart_eps_relaxed_
            << std::endl;
  nh.getParam("pause_debug", pause_debug_);
  visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
}

bool TrajOpt::grad_cost_v(const Eigen::Vector3d& v,
                          Eigen::Vector3d& gradv,
                          double& costv) {
  double vpen = v.squaredNorm() - vmax_ * vmax_;
  if (vpen > 0) {
    double grad = 0;
    costv = smoothedL1(vpen, grad);
    gradv = rhoV_ * grad * 2 * v;
    costv *= rhoV_;
    return true;
  }
  return false;
}

bool TrajOpt::grad_cost_thrust(const Eigen::Vector3d& a,
                               Eigen::Vector3d& grada,
                               double& costa) {
  bool ret = false;
  grada.setZero();
  costa = 0;
  Eigen::Vector3d thrust_f = a - g_;
  double max_pen = thrust_f.squaredNorm() - thrust_max_ * thrust_max_;
  if (max_pen > 0) {
    double grad = 0;
    costa = rhoThrust_ * smoothedL1(max_pen, grad);
    grada = rhoThrust_ * 2 * grad * thrust_f;
    ret = true;
  }

  double min_pen = thrust_min_ * thrust_min_ - thrust_f.squaredNorm();
  if (min_pen > 0) {
    double grad = 0;
    costa = rhoThrust_ * smoothedL1(min_pen, grad);
    grada = -rhoThrust_ * 2 * grad * thrust_f;
    ret = true;
  }

  return ret;
}

// using hopf fibration:
// [a,b,c] = thrust.normalized()
// \omega_1 = sin(\phi) \dot{a] - cos(\phi) \dot{b} - (a sin(\phi) - b cos(\phi)) (\dot{c}/(1+c))
// \omega_2 = cos(\phi) \dot{a] - sin(\phi) \dot{b} - (a cos(\phi) - b sin(\phi)) (\dot{c}/(1+c))
// \omega_3 = (b \dot{a} - a \dot(b)) / (1+c)
// || \omega_12 ||^2 = \omega_1^2 + \omega_2^2 = \dot{a}^2 + \dot{b}^2 + \dot{c}^2

bool TrajOpt::grad_cost_omega(const Eigen::Vector3d& a,
                              const Eigen::Vector3d& j,
                              Eigen::Vector3d& grada,
                              Eigen::Vector3d& gradj,
                              double& cost) {
  Eigen::Vector3d thrust_f = a - g_;
  Eigen::Vector3d zb_dot = f_DN(thrust_f) * j;
  double omega_12_sq = zb_dot.squaredNorm();
  double pen = omega_12_sq - omega_max_ * omega_max_;
  if (pen > 0) {
    double grad = 0;
    cost = smoothedL1(pen, grad);

    Eigen::Vector3d grad_zb_dot = 2 * zb_dot;
    // std::cout << "grad_zb_dot: " << grad_zb_dot.transpose() << std::endl;
    gradj = f_DN(thrust_f).transpose() * grad_zb_dot;
    grada = f_D2N(thrust_f, j).transpose() * grad_zb_dot;

    cost *= rhoOmega_;
    grad *= rhoOmega_;
    grada *= grad;
    gradj *= grad;

    return true;
  }
  return false;
}
bool TrajOpt::grad_cost_omega_yaw(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& j,
                                  Eigen::Vector3d& grada,
                                  Eigen::Vector3d& gradj,
                                  double& cost) {
  // TODO
  return false;
}

bool TrajOpt::grad_cost_floor(const Eigen::Vector3d& p,
                              Eigen::Vector3d& gradp,
                              double& costp) {
  static double z_floor = 0.4;
  double pen = z_floor - p.z();
  if (pen > 0) {
    double grad = 0;
    costp = smoothedL1(pen, grad);
    costp *= rhoP_;
    gradp.setZero();
    gradp.z() = -rhoP_ * grad;
    return true;
  } else {
    return false;
  }
}

// plate: \Epsilon = \left{ x = RBu + c | \norm(u) \leq r \right}
// x \in R_{3\times1}, u \in R_{2\times1}, B \in R_{3\times2}
// c: center of the plate; p: center of the drone bottom
//  c = p - l * z_b
// plane: a^T x \leq b
//        a^T(RBu + c) \leq b
//        a^T(RBu + p - l * z_b) \leq b
//        u^T(B^T R^T a) + a^Tp - a^T*l*z_b - b \leq 0
//        r \norm(B^T R^T a) + a^Tp - a^T*l*z_b - b \leq 0
// B^T R^T = [1-2y^2,    2xy, -2yw;
//               2xy, 1-2x^2,  2xw]
// B^T R^T = [1-a^2/(1+c),   -ab/(1+c), -a;
//              -ab/(1+c), 1-b^2/(1+c), -b]
bool TrajOpt::grad_cost_perching_collision(const Eigen::Vector3d& pos,
                                           const Eigen::Vector3d& acc,
                                           const Eigen::Vector3d& car_p,
                                           Eigen::Vector3d& gradp,
                                           Eigen::Vector3d& grada,
                                           Eigen::Vector3d& grad_car_p,
                                           double& cost) {
  static double eps = 1e-6;

  double dist_sqr = (pos - car_p).squaredNorm();
  double safe_r = platform_r_ + robot_r_;
  double safe_r_sqr = safe_r * safe_r;
  double pen_dist = safe_r_sqr - dist_sqr;
  pen_dist /= safe_r_sqr;
  double grad_dist = 0;
  double var01 = smoothed01(pen_dist, grad_dist);
  if (var01 == 0) {
    return false;
  }
  Eigen::Vector3d gradp_dist = grad_dist * 2 * (car_p - pos);
  Eigen::Vector3d grad_carp_dist = -gradp_dist;

  Eigen::Vector3d a_i = -tail_q_v_;
  double b_i = a_i.dot(car_p);

  Eigen::Vector3d thrust_f = acc - g_;
  Eigen::Vector3d zb = f_N(thrust_f);

  Eigen::MatrixXd BTRT(2, 3);
  double a = zb.x();
  double b = zb.y();
  double c = zb.z();

  double c_1 = 1.0 / (1 + c);

  BTRT(0, 0) = 1 - a * a * c_1;
  BTRT(0, 1) = -a * b * c_1;
  BTRT(0, 2) = -a;
  BTRT(1, 0) = -a * b * c_1;
  BTRT(1, 1) = 1 - b * b * c_1;
  BTRT(1, 2) = -b;

  Eigen::Vector2d v2 = BTRT * a_i;
  double v2_norm = sqrt(v2.squaredNorm() + eps);
  double pen = a_i.dot(pos) - (robot_l_ - 0.005) * a_i.dot(zb) - b_i + robot_r_ * v2_norm;

  if (pen > 0) {
    double grad = 0;
    cost = smoothedL1(pen, grad);
    // gradients: pos, car_p, v2
    gradp = a_i;
    grad_car_p = -a_i;
    Eigen::Vector2d grad_v2 = robot_r_ * v2 / v2_norm;

    Eigen::MatrixXd pM_pa(2, 3), pM_pb(2, 3), pM_pc(2, 3);
    double c2_1 = c_1 * c_1;

    pM_pa(0, 0) = -2 * a * c_1;
    pM_pa(0, 1) = -b * c_1;
    pM_pa(0, 2) = -1;
    pM_pa(1, 0) = -b * c_1;
    pM_pa(1, 1) = 0;
    pM_pa(1, 2) = 0;

    pM_pb(0, 0) = 0;
    pM_pb(0, 1) = -a * c_1;
    pM_pb(0, 2) = 0;
    pM_pb(1, 0) = -a * c_1;
    pM_pb(1, 1) = -2 * b * c_1;
    pM_pb(1, 2) = -1;

    pM_pc(0, 0) = a * a * c2_1;
    pM_pc(0, 1) = a * b * c2_1;
    pM_pc(0, 2) = 0;
    pM_pc(1, 0) = a * b * c2_1;
    pM_pc(1, 1) = b * b * c2_1;
    pM_pc(1, 2) = 0;

    Eigen::MatrixXd pv2_pzb(2, 3);
    pv2_pzb.col(0) = pM_pa * a_i;
    pv2_pzb.col(1) = pM_pb * a_i;
    pv2_pzb.col(2) = pM_pc * a_i;

    Eigen::Vector3d grad_zb = pv2_pzb.transpose() * grad_v2 - robot_l_ * a_i;

    grada = f_DN(thrust_f).transpose() * grad_zb;

    grad *= var01;
    gradp_dist *= cost;
    grad_carp_dist *= cost;
    cost *= var01;
    gradp = grad * gradp + gradp_dist;
    grada *= grad;
    grad_car_p = grad * grad_car_p + grad_carp_dist;

    cost *= rhoPerchingCollision_;
    gradp *= rhoPerchingCollision_;
    grada *= rhoPerchingCollision_;
    grad_car_p *= rhoPerchingCollision_;

    // std::cout << "var01: " << var01 << std::endl;

    return true;
  }
  return false;
}

bool TrajOpt::check_collilsion(const Eigen::Vector3d& pos,
                               const Eigen::Vector3d& acc,
                               const Eigen::Vector3d& car_p) {
  if ((pos - car_p).norm() > platform_r_) {
    return false;
  }
  static double eps = 1e-6;

  Eigen::Vector3d a_i = -tail_q_v_;
  double b_i = a_i.dot(car_p);

  Eigen::Vector3d thrust_f = acc - g_;
  Eigen::Vector3d zb = f_N(thrust_f);

  Eigen::MatrixXd BTRT(2, 3);
  double a = zb.x();
  double b = zb.y();
  double c = zb.z();

  double c_1 = 1.0 / (1 + c);

  BTRT(0, 0) = 1 - a * a * c_1;
  BTRT(0, 1) = -a * b * c_1;
  BTRT(0, 2) = -a;
  BTRT(1, 0) = -a * b * c_1;
  BTRT(1, 1) = 1 - b * b * c_1;
  BTRT(1, 2) = -b;

  Eigen::Vector2d v2 = BTRT * a_i;
  double v2_norm = sqrt(v2.squaredNorm() + eps);
  double pen = a_i.dot(pos) - (robot_l_ - 0.005) * a_i.dot(zb) - b_i + robot_r_ * v2_norm;

  return pen > 0;
}

bool TrajOpt::feasibleCheck(Trajectory& traj) {
  double dt = 0.01;
  for (double t = 0; t < traj.getTotalDuration(); t += dt) {
    Eigen::Vector3d p = traj.getPos(t);
    Eigen::Vector3d a = traj.getAcc(t);
    Eigen::Vector3d j = traj.getJer(t);
    Eigen::Vector3d thrust = a - g_;
    Eigen::Vector3d zb_dot = f_DN(thrust) * j;
    double omega12 = zb_dot.norm();
    if (omega12 > omega_max_ + 0.2) {
      return false;
    }
    if (p.z() < 0.1) {
      return false;
    }
  }
  return true;
}

}  // namespace traj_opt

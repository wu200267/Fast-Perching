#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>

#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <vis_utils/vis_utils.hpp>

namespace planning {

/*
 * 文件职责总览（planning_nodelet.cpp）
 * 1) 作为 ROS nodelet 的调度入口，接收触发信号后调用轨迹优化器。
 * 2) 从参数服务器读取“平台状态/初始状态/调试开关”等配置。
 * 3) 将优化结果以可视化 topic 的方式发布，供 RViz 与辅助节点显示。
 * 4) 可选地在轨迹执行过程中进行重规划（replan 模式）。
 *
 * 设计特点：
 * - 该文件不直接实现优化数学细节，优化在 traj_opt::TrajOpt 中完成。
 * - 该文件主要负责“流程编排”：触发 -> 组装状态 -> 调优化 -> 回放/可视化。
 */
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

/*
 * Nodelet 生命周期说明：
 * - onInit(): nodelet 被 manager 加载后调用，启动初始化线程。
 * - init(): 读取参数、创建订阅与定时器、构造可视化与优化对象。
 * - triger_callback(): 接收外部触发事件，只置位标志位。
 * - debug_timer_callback(): 定时检查触发标志，执行一次完整规划流程。
 */
class Nodelet : public nodelet::Nodelet {
 private:
  // =========================
  // 运行时基础组件
  // =========================
  std::thread initThread_;
  ros::Subscriber triger_sub_;
  ros::Timer plan_timer_;

  // 可视化与优化模块句柄
  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;

  // NOTE planning or fake target
  bool target_ = false;
  Eigen::Vector3d goal_;

  // NOTE just for debug
  bool debug_ = false;
  bool once_ = false;
  bool debug_replan_ = false;
  // benchmark 快速模式：只执行一次轨迹优化，不做后续逐点回放/可视化循环，
  // 用于避免回调长时间占用导致的触发堆积和超时。
  bool bench_mode_ = false;
  // Fig.11 导出模式（离线采样 + CSV/summary）
  bool fig11_export_enable_ = false;
  std::string fig11_export_dir_;
  std::string fig11_case_tag_;
  double fig11_sample_dt_ = 0.005;

  double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d ini_p_;
  Eigen::Vector3d ini_v_;
  std::string ini_v_source_ = "fallback_whole_vector";
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_;
  double perching_theta_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  int plan_hz_ = 10;

  // 触发标志：回调置 true，主定时器消费后置 false
  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);

  /*
   * 触发回调：
   * - 输入: geometry_msgs/PoseStamped
   * - 当前实现只把 x/y 存入 goal_（z 固定为 1.0），并置触发标志。
   * - 真实规划目标并非来自 goal_，而是来自 launch 参数 perching_*。
   *   这里的触发消息更多承担“开始规划”的开关作用。
   */
  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 1.0;
    triger_received_ = true;
  }

  static Eigen::Matrix3d safe_f_DN(const Eigen::Vector3d& x) {
    constexpr double kNormEps = 1e-3;
    Eigen::Vector3d xs = x;
    double n = xs.norm();
    if (n < kNormEps) {
      if (n < 1e-12) {
        xs = Eigen::Vector3d(0.0, 0.0, kNormEps);
      } else {
        xs *= (kNormEps / n);
      }
    }
    double x_norm_2 = xs.squaredNorm();
    return (Eigen::Matrix3d::Identity() - xs * xs.transpose() / x_norm_2) / std::sqrt(x_norm_2);
  }

  bool export_fig11_artifacts(const Trajectory& traj) {
    if (!fig11_export_enable_) {
      return true;
    }
    if (fig11_export_dir_.empty() || fig11_case_tag_.empty()) {
      ROS_ERROR("[Fig11] Export is enabled but fig11_export_dir/case_tag is empty.");
      return false;
    }

    const double dt = (fig11_sample_dt_ > 0.0) ? fig11_sample_dt_ : 0.005;
    const std::string csv_path = fig11_export_dir_ + "/" + fig11_case_tag_ + ".csv";
    const std::string summary_path = fig11_export_dir_ + "/" + fig11_case_tag_ + "_summary.txt";

    std::ofstream csv(csv_path.c_str(), std::ios::out | std::ios::trunc);
    if (!csv.is_open()) {
      ROS_ERROR_STREAM("[Fig11] Failed to open csv: " << csv_path);
      return false;
    }
    csv.setf(std::ios::fixed);
    csv << std::setprecision(9);
    csv << "t,x,z,tau_norm2,tau_norm,omega2\n";

    const Eigen::Vector3d g(0.0, 0.0, -9.8);
    const double T = traj.getTotalDuration();
    std::vector<double> samples_t;
    if (T <= 0.0) {
      samples_t.push_back(0.0);
    } else {
      for (int k = 0;; ++k) {
        const double tk = k * dt;
        if (tk >= T) {
          break;
        }
        samples_t.push_back(tk);
      }
      if (samples_t.empty() || T > samples_t.back() + 1e-12) {
        samples_t.push_back(T);
      }
    }

    for (size_t i = 1; i < samples_t.size(); ++i) {
      if (!(samples_t[i] > samples_t[i - 1])) {
        ROS_ERROR("[Fig11] Non-monotonic sampling timeline.");
        return false;
      }
    }

    int den_clamp_count = 0;
    int thrust_clamp_count = 0;
    double max_tau_norm_export = 0.0;
    double min_tau_norm_export = std::numeric_limits<double>::infinity();
    double max_abs_omega2_export = 0.0;
    constexpr double kDenEps = 1e-6;
    constexpr double kThrustNormEps = 1e-3;
    for (double t : samples_t) {
      const Eigen::Vector3d p = traj.getPos(t);
      const Eigen::Vector3d a = traj.getAcc(t);
      const Eigen::Vector3d j = traj.getJer(t);
      const Eigen::Vector3d thrust = a - g;

      const double tau_norm2 = thrust.squaredNorm();
      const double tau_norm = std::sqrt(tau_norm2);
      max_tau_norm_export = std::max(max_tau_norm_export, tau_norm);
      min_tau_norm_export = std::min(min_tau_norm_export, tau_norm);

      Eigen::Vector3d thrust_safe = thrust;
      const double thrust_norm = thrust_safe.norm();
      if (thrust_norm < kThrustNormEps) {
        thrust_clamp_count++;
        if (thrust_norm < 1e-12) {
          thrust_safe = Eigen::Vector3d(0.0, 0.0, kThrustNormEps);
        } else {
          thrust_safe *= (kThrustNormEps / thrust_norm);
        }
      }
      const Eigen::Vector3d zb = thrust_safe.normalized();
      const Eigen::Vector3d zb_dot = safe_f_DN(thrust_safe) * j;
      double den = zb.z() + 1.0;
      if (den < kDenEps) {
        den = kDenEps;
        den_clamp_count++;
      }
      const double omega2 = zb_dot.x() - zb.x() * zb_dot.z() / den;
      max_abs_omega2_export = std::max(max_abs_omega2_export, std::abs(omega2));

      csv << t << "," << p.x() << "," << p.z() << "," << tau_norm2 << "," << tau_norm << "," << omega2 << "\n";
    }
    csv.close();
    if (!std::isfinite(min_tau_norm_export)) {
      min_tau_norm_export = 0.0;
    }

    const traj_opt::TrajOpt::TerminalSummary terminal = trajOptPtr_->getLastTerminalSummary();
    std::ofstream summary(summary_path.c_str(), std::ios::out | std::ios::trunc);
    if (!summary.is_open()) {
      ROS_ERROR_STREAM("[Fig11] Failed to open summary: " << summary_path);
      return false;
    }
    summary.setf(std::ios::fixed);
    summary << std::setprecision(9);

    auto vec_to_csv = [](const Eigen::Vector3d& v) {
      std::ostringstream oss;
      oss.setf(std::ios::fixed);
      oss << std::setprecision(9) << v.x() << "," << v.y() << "," << v.z();
      return oss.str();
    };

    summary << "case_tag=" << fig11_case_tag_ << "\n";
    summary << "sample_dt=" << dt << "\n";
    summary << "duration=" << T << "\n";
    summary << "perching_px=" << perching_p_.x() << "\n";
    summary << "perching_py=" << perching_p_.y() << "\n";
    summary << "perching_pz=" << perching_p_.z() << "\n";
    summary << "perching_vx=" << perching_v_.x() << "\n";
    summary << "perching_vy=" << perching_v_.y() << "\n";
    summary << "perching_vz=" << perching_v_.z() << "\n";
    summary << "ini_vx=" << ini_v_.x() << "\n";
    summary << "ini_vy=" << ini_v_.y() << "\n";
    summary << "ini_vz=" << ini_v_.z() << "\n";
    summary << "ini_v_source=" << ini_v_source_ << "\n";
    summary << "perching_axis=" << vec_to_csv(perching_axis_) << "\n";
    summary << "perching_theta=" << perching_theta_ << "\n";
    summary << "rhoPerchingCollision=" << trajOptPtr_->rhoPerchingCollision_ << "\n";
    summary << "den_clamp_count=" << den_clamp_count << "\n";
    summary << "thrust_clamp_count=" << thrust_clamp_count << "\n";
    summary << "summary_valid=" << (terminal.valid ? 1 : 0) << "\n";
    summary << "vt1=" << terminal.vt1 << "\n";
    summary << "vt2=" << terminal.vt2 << "\n";
    summary << "vt_norm=" << terminal.vt_norm << "\n";
    summary << "vt_signed=" << terminal.vt_signed << "\n";
    summary << "vt_primary=" << terminal.vt_primary << "\n";
    summary << "vt_primary_axis=" << terminal.vt_primary_axis << "\n";
    summary << "vt3d=" << vec_to_csv(terminal.vt3d) << "\n";
    summary << "zd=" << vec_to_csv(terminal.zd) << "\n";
    summary << "v1=" << vec_to_csv(terminal.v1) << "\n";
    summary << "v2=" << vec_to_csv(terminal.v2) << "\n";
    summary << "max_tau_norm_int=" << terminal.max_tau_norm_int << "\n";
    summary << "min_tau_norm_int=" << terminal.min_tau_norm_int << "\n";
    summary << "max_abs_omega2_int=" << terminal.max_abs_omega2_int << "\n";
    summary << "max_tau_norm_export=" << max_tau_norm_export << "\n";
    summary << "min_tau_norm_export=" << min_tau_norm_export << "\n";
    summary << "max_abs_omega2_export=" << max_abs_omega2_export << "\n";
    summary << "winner_seed_vt2=" << terminal.winner_seed_vt2 << "\n";
    summary << "winner_objective=" << terminal.winner_objective << "\n";
    summary << "winner_viol_total_int=" << terminal.winner_viol_total_int << "\n";
    summary << "winner_eps_level=" << terminal.winner_eps_level << "\n";
    summary << "solve_mode=" << terminal.solve_mode << "\n";
    summary.close();

    ROS_INFO_STREAM("[Fig11] Exported case '" << fig11_case_tag_ << "' to " << csv_path);
    return true;
  }

  /*
   * 主流程回调（由 plan_timer_ 周期触发）：
   * - 若未触发，直接返回；
   * - 已触发时，组装 iniState / target_p / target_v / land_q；
   * - 调用 trajOptPtr_->generate_traj() 求解；
   * - bench_mode 下只计时并结束；
   * - 非 bench_mode 下执行可视化回放，并可选重规划。
   */
  void debug_timer_callback(const ros::TimerEvent& event) {
    // 没收到触发信号时，不做任何计算，避免空转消耗。
    if (!triger_received_) {
      return;
    }

    // ---------- 本次规划的局部变量 ----------
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 4);
    bool generate_new_traj_success = false;
    Trajectory traj;
    Eigen::Vector3d target_p, target_v;
    Eigen::Quaterniond target_q;
    Eigen::Quaterniond land_q(1, 0, 0, 0);

    /*
     * 初始化边界状态：
     * iniState 列定义通常为 [p, v, a, j]（每列 3 维）。
     * 当前设置：
     * - p 来自 ini_p_（可由 ini_px/py/pz 参数覆盖默认值）
     * - v 来自 ini_v_（与平台速度 perching_v_ 解耦）
     * - a, j 默认 0
     */
    iniState.setZero();
    iniState.col(0) = ini_p_;
    iniState.col(1) = ini_v_;

    // 目标平台当前状态：位置/速度均由 launch 参数 perching_* 提供。
    target_p = perching_p_;
    target_v = perching_v_;
    target_q.x() = 0.0;
    target_q.y() = 0.0;
    target_q.z() = 0.0;
    target_q.w() = 1.0;

    /*
     * 构造期望着陆姿态四元数 land_q：
     * - 旋转轴：perching_axis_（单位化）
     * - 旋转角：perching_theta_
     * - 当前 target_q 为单位四元数，因此 land_q 等价于 axis-angle 的结果
     */
    Eigen::Vector3d axis = perching_axis_.normalized();
    double theta = perching_theta_ * 0.5;
    land_q.w() = cos(theta);
    land_q.x() = axis.x() * sin(theta);
    land_q.y() = axis.y() * sin(theta);
    land_q.z() = axis.z() * sin(theta);
    land_q = target_q * land_q;

    std::cout << "iniState: \n"
              << iniState << std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl;
    std::cout << "target_v: " << target_v.transpose() << std::endl;
    std::cout << "land_q: "
              << land_q.w() << ","
              << land_q.x() << ","
              << land_q.y() << ","
              << land_q.z() << "," << std::endl;

    // 核心求解调用：N=10 段，输出轨迹 traj。
    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    if (!generate_new_traj_success) {
      // 失败时清除触发标志，等待下一次触发。
      triger_received_ = false;
      return;
      // assert(false);
    }
    if (fig11_export_enable_) {
      if (!export_fig11_artifacts(traj)) {
        ROS_ERROR("[Fig11] Export failed.");
        triger_received_ = false;
        return;
      }
    }
    if (bench_mode_) {
      // bench 模式到此结束：计时日志已在 generate_traj 内打印，Fig.11 导出在此之前执行。
      triger_received_ = false;
      return;
    }

    // 发布整条轨迹与末端速度箭头，便于直观看到落点和末端速度方向。
    visPtr_->visualize_traj(traj, "traj");

    Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
    Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
    visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");

    // NOTE run vis
    // hopf fiberation
    /*
     * v2q:
     * - 输入: 推力方向单位向量 zb
     * - 输出: 将机体 z 轴对齐到 zb 的四元数（忽略 yaw 自由度）
     * - 用于从平坦输出（加速度）恢复可视化姿态
     */
    auto v2q = [](const Eigen::Vector3d& v, Eigen::Quaterniond& q) -> bool {
      double a = v.x();
      double b = v.y();
      double c = v.z();
      if (c == -1) {
        return false;
      }
      double d = 1.0 / sqrt(2.0 * (1 + c));
      q.w() = (1 + c) * d;
      q.x() = -b * d;
      q.y() = a * d;
      q.z() = 0;
      return true;
    };

    // f_DN: 单位化映射 N(x)=x/||x|| 的一阶导，用于估计角速度相关量。
    auto f_DN = [](const Eigen::Vector3d& x) {
      double x_norm_2 = x.squaredNorm();
      return (Eigen::MatrixXd::Identity(3, 3) - x * x.transpose() / x_norm_2) / sqrt(x_norm_2);
    };
    // auto f_D2N = [](const Eigen::Vector3d& x, const Eigen::Vector3d& y) {
    //   double x_norm_2 = x.squaredNorm();
    //   double x_norm_3 = x_norm_2 * x.norm();
    //   Eigen::MatrixXd A = (3 * x * x.transpose() / x_norm_2 - Eigen::MatrixXd::Identity(3, 3));
    //   return (A * y * x.transpose() - x * y.transpose() - x.dot(y) * Eigen::MatrixXd::Identity(3, 3)) / x_norm_3;
    // };

    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    double dt = 0.001;
    Eigen::Quaterniond q_last;
    double max_omega = 0;

    /*
     * 轨迹回放循环：
     * - 沿时间采样 traj，发布无人机 odom 与目标平台 target_odom；
     * - 计算并记录最大角速度指标 max_omega；
     * - 若开启 replan，在执行中段重新调用优化器生成剩余轨迹。
     */
    for (double t = 0; t <= traj.getTotalDuration(); t += dt) {
      ros::Duration(dt).sleep();
      // drone
      Eigen::Vector3d p = traj.getPos(t);
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);
      Eigen::Vector3d g(0, 0, -9.8);
      Eigen::Vector3d thrust = a - g;

      // std::cout << p.x() << " , " << p.z() << " , ";

      Eigen::Vector3d zb = thrust.normalized();
      {
        // double a = zb.x();
        // double b = zb.y();
        // double c = zb.z();
        Eigen::Vector3d zb_dot = f_DN(thrust) * j;
        double omega12 = zb_dot.norm();
        // if (omega12 > 3.1) {
        //   std::cout << "omega: " << omega12 << "rad/s  t: " << t << std::endl;
        // }
        if (omega12 > max_omega) {
          max_omega = omega12;
        }
        // double a_dot = zb_dot.x();
        // double b_dot = zb_dot.y();
        // double omega3 = (b * a_dot - a * b_dot) / (1 + c);
        // std::cout << "jer: " << j.transpose() << std::endl;
        // std::cout << "omega12: " << zb_dot.norm() << std::endl;
        // std::cout << "omega3: " << omega3 << std::endl;
        // std::cout << thrust.x() << " , " << thrust.z() << " , ";
        // double omega2 = zb_dot.x() - zb.x() * zb_dot.z() / (zb.z() + 1);
        // std::cout << omega2 << std::endl;
        // std::cout << zb_dot.norm() << std::endl;
      }

      Eigen::Quaterniond q;
      bool no_singlarity = v2q(zb, q);
      Eigen::MatrixXd R_dot = (q.toRotationMatrix() - q_last.toRotationMatrix()) / dt;
      Eigen::MatrixXd omega_M = q.toRotationMatrix().transpose() * R_dot;
      // std::cout << "omega_M: \n" << omega_M << std::endl;
      Eigen::Vector3d omega_real;
      omega_real.x() = -omega_M(1, 2);
      omega_real.y() = omega_M(0, 2);
      omega_real.z() = -omega_M(0, 1);
      // std::cout << "omega_real: " << omega_real.transpose() << std::endl;
      q_last = q;
      if (no_singlarity) {
        msg.pose.pose.position.x = p.x();
        msg.pose.pose.position.y = p.y();
        msg.pose.pose.position.z = p.z();
        msg.pose.pose.orientation.w = q.w();
        msg.pose.pose.orientation.x = q.x();
        msg.pose.pose.orientation.y = q.y();
        msg.pose.pose.orientation.z = q.z();
        msg.header.stamp = ros::Time::now();
        visPtr_->visualize_traj(traj, "traj");
        visPtr_->pub_msg(msg, "odom");
      }
      // target
      // Eigen::Vector3d fake_target_v = target_v * (1.0 + 0.5 * sin(1e6 * t));
      // target_p = target_p + fake_target_v * dt;
      // target_v *= 1.0001;
      // 目标平台按匀速模型前推，用于动态目标场景可视化与碰撞检测。
      target_p = target_p + target_v * dt;
      msg.pose.pose.position.x = target_p.x();
      msg.pose.pose.position.y = target_p.y();
      msg.pose.pose.position.z = target_p.z();
      msg.pose.pose.orientation.w = land_q.w();
      msg.pose.pose.orientation.x = land_q.x();
      msg.pose.pose.orientation.y = land_q.y();
      msg.pose.pose.orientation.z = land_q.z();
      msg.header.stamp = ros::Time::now();
      visPtr_->pub_msg(msg, "target_odom");
      if (trajOptPtr_->check_collilsion(p, a, target_p)) {
        std::cout << "collide!  t: " << t << std::endl;
      }
      // TODO replan
      // 重规划条件：开启 replan 且已执行一小段时间且轨迹剩余时长足够。
      if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
        // ros::Duration(3.0).sleep();

        // 把当前执行状态作为下一次规划的初始状态，实现 warm-start replan。
        iniState.col(0) = traj.getPos(t);
        iniState.col(1) = traj.getVel(t);
        iniState.col(2) = traj.getAcc(t);
        iniState.col(3) = traj.getJer(t);
        std::cout << "iniState: \n"
                  << iniState << std::endl;
        trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj, t);
        visPtr_->visualize_traj(traj, "traj");
        t = 0;
        std::cout << "max omega: " << max_omega << std::endl;
      }
    }
    std::cout << "tailV: " << traj.getVel(traj.getTotalDuration()).transpose() << std::endl;
    std::cout << "max thrust: " << traj.getMaxThrust() << std::endl;
    std::cout << "max omega: " << max_omega << std::endl;

    // 本次触发流程结束，复位触发标志。
    triger_received_ = false;
  }

  /*
   * 初始化函数：
   * - 从参数服务器读取配置；
   * - 构建可视化与优化器对象；
   * - 注册定时器与触发订阅；
   * - 打印“Planning node initialized!” 表示节点就绪。
   */
  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    nh.getParam("replan", debug_replan_);
    nh.param("bench_mode", bench_mode_, false);
    nh.param("fig11_export_enable", fig11_export_enable_, false);
    nh.param("fig11_export_dir", fig11_export_dir_, std::string(""));
    nh.param("fig11_case_tag", fig11_case_tag_, std::string(""));
    nh.param("fig11_sample_dt", fig11_sample_dt_, 0.005);
    if (fig11_sample_dt_ <= 0.0) {
      fig11_sample_dt_ = 0.005;
    }

    // Initial state position (default keeps legacy behavior)
    nh.param("ini_px", ini_p_.x(), 0.0);
    nh.param("ini_py", ini_p_.y(), 0.0);
    nh.param("ini_pz", ini_p_.z(), 2.0);

    // NOTE once
    nh.getParam("perching_px", perching_p_.x());
    nh.getParam("perching_py", perching_p_.y());
    nh.getParam("perching_pz", perching_p_.z());
    nh.getParam("perching_vx", perching_v_.x());
    nh.getParam("perching_vy", perching_v_.y());
    nh.getParam("perching_vz", perching_v_.z());
    nh.getParam("perching_axis_x", perching_axis_.x());
    nh.getParam("perching_axis_y", perching_axis_.y());
    nh.getParam("perching_axis_z", perching_axis_.z());
    nh.getParam("perching_theta", perching_theta_);

    const bool has_ini_vx = nh.getParam("ini_vx", ini_v_.x());
    const bool has_ini_vy = nh.getParam("ini_vy", ini_v_.y());
    const bool has_ini_vz = nh.getParam("ini_vz", ini_v_.z());
    if (has_ini_vx && has_ini_vy && has_ini_vz) {
      ini_v_source_ = "explicit";
    } else {
      ini_v_ = perching_v_;
      ini_v_source_ = "fallback_whole_vector";
    }

    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);

    // 定时器驱动主流程，周期为 1 / plan_hz_。
    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::debug_timer_callback, this);

    // 订阅触发 topic（注意项目内拼写为 triger）。
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    if (fig11_export_enable_) {
      ROS_INFO_STREAM("[Fig11] export enabled, dir=" << fig11_export_dir_
                      << ", case_tag=" << fig11_case_tag_
                      << ", sample_dt=" << fig11_sample_dt_
                      << ", ini_v=" << ini_v_.transpose()
                      << ", ini_v_source=" << ini_v_source_);
    }
    ROS_WARN("Planning node initialized!");
  }

 public:
  // nodelet 标准入口：使用 private NodeHandle，并在独立线程中执行 init。
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);

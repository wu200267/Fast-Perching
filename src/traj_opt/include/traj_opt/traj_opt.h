#pragma once
#include <ros/ros.h>

#include <chrono>
#include <thread>
#include <vis_utils/vis_utils.hpp>

#include "minco.hpp"

namespace traj_opt {

class TrajOpt {
 public:
  ros::NodeHandle nh_;
  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  bool pause_debug_ = false;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // weight for time regularization term
  double rhoT_, rhoVt_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double rhoP_, rhoV_, rhoA_;
  double rhoThrust_, rhoOmega_;
  double rhoPerchingCollision_;
  // landing parameters
  double v_plus_, robot_l_, robot_r_, platform_r_;
  // SE3 dynamic limitation parameters
  double thrust_max_, thrust_min_;
  double omega_max_, omega_yaw_max_;
  // MINCO Optimizer
  minco::MINCO_S4_Uniform mincoOpt_;
  Eigen::MatrixXd initS_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  // L-BFGS 优化变量缓存。每次 generate_traj() 重新分配，结束后释放。
  double* x_ = nullptr;
  // 本次优化维度：base_dim(+2 in non-fixed, +0 in fixed)。
  int opt_dim_ = 0;

  // ========== Hard-Fix Terminal State 配置参数 ==========
  // false: 维持旧行为，tail_f 与 vt 仍然参与优化
  // true : 仅 vt 从优化向量移除；tail_f 仍参与优化
  bool fix_terminal_state_ = false;
  // 开发期调试：每个节点生命周期内仅打印一次优化变量布局。
  bool print_opt_layout_once_ = false;
  bool opt_layout_printed_ = false;
  // 对应 fixed_* 参数是否由 ROS 显式提供。
  // 若未提供则回退到本次 guess（0 或 warm-start）。
  bool has_fixed_vt_x_ = false;
  bool has_fixed_vt_y_ = false;
  // ROS 参数 fixed_* 的数值缓存，仅在 has_fixed_* 为 true 时生效。
  double fixed_vt_x_param_ = 0.0;
  double fixed_vt_y_param_ = 0.0;

  // ========== 本次优化上下文（供 objectiveFunc/earlyExit 读取） ==========
  // active_vt_: 本次优化真正使用的固定 vt 值。
  // active_vt_guess_: 本次生成的 vt guess（用于 fixed 模式下“未显式传参”回退策略）。
  // 注意：active_vt_ 在单次 lbfgs_optimize() 期间保持不变。
  Eigen::Vector2d active_vt_ = Eigen::Vector2d::Zero();
  Eigen::Vector2d active_vt_guess_ = Eigen::Vector2d::Zero();

  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<Eigen::Vector3d> tracking_visible_ps_;
  std::vector<double> tracking_thetas_;

 public:
  TrajOpt(ros::NodeHandle& nh);
  ~TrajOpt() {}

  int optimize(const double& delta = 1e-4);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::Vector3d& car_p,
                     const Eigen::Vector3d& car_v,
                     const Eigen::Quaterniond& land_q,
                     const int& N,
                     Trajectory& traj, 
                     const double& t_replan = -1.0);

  bool feasibleCheck(Trajectory& traj);

  void addTimeIntPenalty(double& cost);
  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_thrust(const Eigen::Vector3d& a,
                        Eigen::Vector3d& grada,
                        double& costa);
  bool grad_cost_omega(const Eigen::Vector3d& a,
                       const Eigen::Vector3d& j,
                       Eigen::Vector3d& grada,
                       Eigen::Vector3d& gradj,
                       double& cost);
  bool grad_cost_omega_yaw(const Eigen::Vector3d& a,
                           const Eigen::Vector3d& j,
                           Eigen::Vector3d& grada,
                           Eigen::Vector3d& gradj,
                           double& cost);
  bool grad_cost_floor(const Eigen::Vector3d& p,
                       Eigen::Vector3d& gradp,
                       double& costp);
  bool grad_cost_perching_collision(const Eigen::Vector3d& pos,
                                    const Eigen::Vector3d& acc,
                                    const Eigen::Vector3d& car_p,
                                    Eigen::Vector3d& gradp,
                                    Eigen::Vector3d& grada,
                                    Eigen::Vector3d& grad_car_p,
                                    double& cost);
  bool check_collilsion(const Eigen::Vector3d& pos,
                        const Eigen::Vector3d& acc,
                        const Eigen::Vector3d& car_p);
};

}  // namespace traj_opt

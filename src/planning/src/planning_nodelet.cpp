#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <traj_opt/traj_opt.h>
#include <traj_opt/target_predictor.hpp>

#include <Eigen/Core>
#include <atomic>
#include <mutex>
#include <thread>
#include <vis_utils/vis_utils.hpp>

namespace planning {

Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

class Nodelet : public nodelet::Nodelet {
 private:
  std::thread initThread_;
  ros::Subscriber triger_sub_;
  ros::Timer plan_timer_;
  ros::Timer target_timer_;
  std::mutex predictor_mutex_;

  std::shared_ptr<vis_utils::VisUtils> visPtr_;
  std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
  traj_opt::TargetPredictor predictor_;

  // NOTE planning or fake target
  bool target_ = false;
  Eigen::Vector3d goal_;

  // NOTE just for debug
  bool debug_ = false;
  bool once_ = false;
  bool debug_replan_ = false;

  double tracking_dur_, tracking_dist_, tolerance_d_;
  Eigen::Vector3d perching_p_, perching_v_, perching_axis_;
  double perching_theta_;

  Trajectory traj_poly_;
  ros::Time replan_stamp_;
  int traj_id_ = 0;
  bool wait_hover_ = true;
  bool force_hover_ = true;

  int plan_hz_ = 10;
  int target_hz_ = 100;
  int min_meas_count_ = 10;

  Eigen::Vector3d target_p_;
  Eigen::Vector3d target_v_;
  Eigen::Quaterniond land_q_{1, 0, 0, 0};
  Eigen::Vector3d bootstrap_pos_;
  double bootstrap_time_ = 0.0;
  double target_yaw_ = 0.0;
  double perching_omega_ = 0.0;  // 从参数读取，0 = 直线
  double target_vh_ = 0.0;

  bool target_running_ = false;
  bool planning_started_ = false;
  bool bootstrap_ready_ = false;
  int meas_count_ = 0;

  std::atomic_bool triger_received_ = ATOMIC_VAR_INIT(false);

  void publish_target_odom(const Eigen::Vector3d& target_p,
                           const Eigen::Quaterniond& land_q,
                           const ros::Time& stamp) {
    nav_msgs::Odometry msg;
    msg.header.frame_id = "world";
    msg.header.stamp = stamp;
    msg.pose.pose.position.x = target_p.x();
    msg.pose.pose.position.y = target_p.y();
    msg.pose.pose.position.z = target_p.z();
    msg.pose.pose.orientation.w = land_q.w();
    msg.pose.pose.orientation.x = land_q.x();
    msg.pose.pose.orientation.y = land_q.y();
    msg.pose.pose.orientation.z = land_q.z();
    visPtr_->pub_msg(msg, "target_odom");
  }

  void triger_callback(const geometry_msgs::PoseStampedConstPtr& msgPtr) {
    Eigen::Vector3d target_p;
    Eigen::Quaterniond land_q;

    {
      std::lock_guard<std::mutex> lock(predictor_mutex_);
      if (target_running_ || planning_started_) {
        return;
      }

      goal_ << msgPtr->pose.position.x, msgPtr->pose.position.y, 1.0;

      predictor_.reset();

      target_p_ = perching_p_;
      target_v_ = perching_v_;
      target_yaw_ = std::atan2(perching_v_.y(), perching_v_.x());
      target_vh_ = 0.0;  // 重置，让 timer 里重新记录

      Eigen::Vector3d axis = perching_axis_.normalized();
      double theta = perching_theta_ * 0.5;
      land_q_.w() = std::cos(theta);
      land_q_.x() = axis.x() * std::sin(theta);
      land_q_.y() = axis.y() * std::sin(theta);
      land_q_.z() = axis.z() * std::sin(theta);

      predictor_.setSurfaceNormal(land_q_.toRotationMatrix().col(2));

      bootstrap_ready_ = false;
      bootstrap_time_ = 0.0;
      meas_count_ = 0;
      planning_started_ = false;
      target_running_ = true;
      triger_received_ = true;

      target_p = target_p_;
      land_q = land_q_;
    }

    publish_target_odom(target_p, land_q, ros::Time::now());
  }

  void target_timer_callback(const ros::TimerEvent& event) {
    Eigen::Vector3d target_p;
    Eigen::Quaterniond land_q;

    {
      std::lock_guard<std::mutex> lock(predictor_mutex_);

      if (!target_running_) {
        return;
      }

      const double dt = (event.current_real - event.last_real).toSec();
      if (dt <= 0.0) {
        return;
      }

      // CTRV motion simulation
      target_yaw_ += perching_omega_ * dt;
      double vh = target_v_.head<2>().norm();  // 只在第一帧有意义，后续 vh 不变
      if (target_vh_ == 0.0) target_vh_ = vh;  // 记住初始水平速度
      Eigen::Vector3d vel(target_vh_ * std::cos(target_yaw_),
                          target_vh_ * std::sin(target_yaw_),
                          target_v_.z());
      target_p_ += vel * dt;
      target_v_ = vel;  // 更新 target_v_ 以便 publish 和其他地方使用
      const double stamp = event.current_real.toSec();
      if (!bootstrap_ready_) {
        bootstrap_pos_ = target_p_;
        bootstrap_time_ = stamp;
        bootstrap_ready_ = true;
      } else if (!predictor_.ready()) {
        Eigen::Vector3d bootstrap_vel =
            (target_p_ - bootstrap_pos_) / std::max(stamp - bootstrap_time_, 1e-3);
        predictor_.seedConstantVelocity(target_p_,
                                        bootstrap_vel,
                                        land_q_.toRotationMatrix().col(2),
                                        stamp);
      } else {
        predictor_.feedMeasurement(target_p_, stamp);
      }
      ++meas_count_;

      target_p = target_p_;
      land_q = land_q_;
    }

    publish_target_odom(target_p, land_q, event.current_real);
  }


  void debug_timer_callback(const ros::TimerEvent& event) {
    if (!triger_received_) {
      return;
    }
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 4);
    bool generate_new_traj_success = false;
    Trajectory traj;
    Eigen::Vector3d target_p;
    Eigen::Quaterniond land_q(1, 0, 0, 0);

    iniState.setZero();
    iniState.col(0).x() = 0.0;
    iniState.col(0).y() = 0.0;
    iniState.col(0).z() = 2.0;
    iniState.col(1) = perching_v_;
    {
      std::lock_guard<std::mutex> lock(predictor_mutex_);
      if (!target_running_ || planning_started_ || meas_count_ < min_meas_count_) {
        return;
      }

      planning_started_ = true;
      target_p = target_p_;
      land_q = land_q_;
    }


    std::cout << "iniState: \n"
              << iniState << std::endl;
    std::cout << "target_p: " << target_p.transpose() << std::endl;
    std::cout << "land_q: "
              << land_q.w() << ","
              << land_q.x() << ","
              << land_q.y() << ","
              << land_q.z() << "," << std::endl;

    // Compute surface normal: land_q 的 z 轴就是法向量（和原代码 q2v 逻辑一致）
    Eigen::Vector3d surface_normal = land_q.toRotationMatrix().col(2);

    ros::Time plan_begin = ros::Time::now();

    // Freeze the predictor from the current EKF estimate for this planning cycle.
    {
      std::lock_guard<std::mutex> lock(predictor_mutex_);
      predictor_.setSurfaceNormal(surface_normal);
      predictor_.generatePrediction(10.0, 0.01);
      std::cout << "EKF state before planning: "
                << predictor_.ekfState().transpose() << std::endl;
    }

    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, predictor_, 10, traj);
    double traj_time_offset = (ros::Time::now() - plan_begin).toSec();
    if (generate_new_traj_success) {
      visPtr_->visualize_traj(traj, "traj");

      Eigen::Vector3d tail_pos = traj.getPos(traj.getTotalDuration());
      Eigen::Vector3d tail_vel = traj.getVel(traj.getTotalDuration());
      visPtr_->visualize_arrow(tail_pos, tail_pos + 0.5 * tail_vel, "tail_vel");
    }
    if (!generate_new_traj_success) {
      triger_received_ = false;
      {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        planning_started_ = false;
        target_running_ = false;
      }
      return;
      // assert(false);
    }

    // NOTE run vis
    // hopf fiberation
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
    const double sleep_dt = 0.001;
    Eigen::Quaterniond q_last;
    bool has_q_last = false;
    double max_omega = 0;
    Eigen::Vector3d final_drone_p = traj.getPos(0.0);
    ros::Time traj_start_stamp = ros::Time::now();
    ros::Time last_stamp = traj_start_stamp;
    while (ros::ok()) {
      ros::Time now = ros::Time::now();
      double t = traj_time_offset + (now - traj_start_stamp).toSec();
      if (t > traj.getTotalDuration()) {
        break;
      }
      double sample_dt = (now - last_stamp).toSec();
      if (sample_dt <= 1e-4) {
        sample_dt = 1e-4;
      }
      last_stamp = now;
      // drone
      Eigen::Vector3d p = traj.getPos(t);
      Eigen::Vector3d a = traj.getAcc(t);
      Eigen::Vector3d j = traj.getJer(t);
      final_drone_p = p;
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
      if (!has_q_last) {
        q_last = q;
        has_q_last = true;
      }
      Eigen::MatrixXd R_dot =
          (q.toRotationMatrix() - q_last.toRotationMatrix()) / sample_dt;
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

      Eigen::Vector3d target_p_now;
      {
        std::lock_guard<std::mutex> lock(predictor_mutex_);
        target_p_now = target_p_;
      }
      if (trajOptPtr_->check_collilsion(p, a, target_p_now)) {
        std::cout << "collide!  t: " << t << std::endl;
      }
      // TODO replan
      if (debug_replan_ && t > 1.0 / plan_hz_ && traj.getTotalDuration() > 0.5) {
        // ros::Duration(3.0).sleep();

        iniState.col(0) = traj.getPos(t);
        iniState.col(1) = traj.getVel(t);
        iniState.col(2) = traj.getAcc(t);
        iniState.col(3) = traj.getJer(t);
        std::cout << "iniState: \n"
                  << iniState << std::endl;
        ros::Time replan_begin = ros::Time::now();
        {
          std::lock_guard<std::mutex> lock(predictor_mutex_);
          predictor_.generatePrediction(10.0, 0.01);
        }
        bool replan_success = trajOptPtr_->generate_traj(iniState, predictor_, 10, traj, t);
        if (replan_success) {
          visPtr_->visualize_traj(traj, "traj");
          traj_time_offset = (ros::Time::now() - replan_begin).toSec();
          traj_start_stamp = ros::Time::now();
          last_stamp = traj_start_stamp;
          has_q_last = false;
        }
        std::cout << "max omega: " << max_omega << std::endl;
      }
      ros::Duration(sleep_dt).sleep();
    }
    std::cout << "tailV: " << traj.getVel(traj.getTotalDuration()).transpose() << std::endl;
    std::cout << "max thrust: " << traj.getMaxThrust() << std::endl;
    std::cout << "max omega: " << max_omega << std::endl;
    Eigen::Vector3d final_target_p;
    {
      std::lock_guard<std::mutex> lock(predictor_mutex_);
      final_target_p = target_p_;
    }
    Eigen::Vector3d final_rel_p = final_drone_p - final_target_p;
    std::cout << "final relative error: " << final_rel_p.transpose() << std::endl;
    std::cout << "final xy error norm: " << final_rel_p.head<2>().norm() << std::endl;

    {
      std::lock_guard<std::mutex> lock(predictor_mutex_);
      planning_started_ = false;
      target_running_ = false;
    }
    triger_received_ = false;
  }

  void init(ros::NodeHandle& nh) {
    // set parameters of planning
    nh.getParam("replan", debug_replan_);
    nh.getParam("plan_hz", plan_hz_);

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
    nh.getParam("perching_omega", perching_omega_);

    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh);

    plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_hz_), &Nodelet::debug_timer_callback, this);
    target_timer_ = nh.createTimer(ros::Duration(1.0 / target_hz_),
                                   &Nodelet::target_timer_callback, this);
    triger_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("triger", 10, &Nodelet::triger_callback, this, ros::TransportHints().tcpNoDelay());
    ROS_WARN("Planning node initialized!");
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    initThread_ = std::thread(std::bind(&Nodelet::init, this, nh));
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace planning

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(planning::Nodelet, nodelet::Nodelet);

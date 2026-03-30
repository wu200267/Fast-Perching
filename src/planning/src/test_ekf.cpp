#include "traj_opt/ctrv_ekf.hpp"
#include <iostream>

int main() {
  using namespace traj_opt;

  CtrvEkf ekf;

  // Configure noise
  CtrvEkf::StateMat Q = CtrvEkf::StateMat::Identity() * 0.1;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.01;
  ekf.setProcessNoise(Q);
  ekf.setMeasurementNoise(R);

  // Feed first measurement → triggers init
  ekf.processMeasurement(Eigen::Vector3d(1.0, 2.0, 0.0), 0.0);
  std::cout << "After init:\n" << ekf.state().transpose() << "\n\n";

  // Feed a second measurement 0.1s later, slightly moved
  ekf.processMeasurement(Eigen::Vector3d(1.2, 2.0, 0.0), 0.1);
  std::cout << "After 2nd meas:\n" << ekf.state().transpose() << "\n";
  std::cout << "v_h = " << ekf.state()(Ctrv::VH) << "\n";
  std::cout << "yaw = " << ekf.state()(Ctrv::YAW) << "\n\n";

  // Feed more measurements simulating straight-line motion at v=2 m/s along x
  for (int i = 2; i <= 20; ++i) {
    double t = i * 0.1;
    Eigen::Vector3d z(1.0 + 2.0 * t, 2.0, 0.0);
    ekf.processMeasurement(z, t);
  }
  std::cout << "After 20 meas (straight line v=2):\n";
  std::cout << "  state: " << ekf.state().transpose() << "\n";
  std::cout << "  v_h = " << ekf.state()(Ctrv::VH)
            << "  (expect ~2.0)\n";
  std::cout << "  yaw = " << ekf.state()(Ctrv::YAW)
            << "  (expect ~0.0)\n";
  std::cout << "  P diag: " << ekf.covariance().diagonal().transpose() << "\n";

  return 0;
}

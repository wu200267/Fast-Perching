#include "traj_opt/target_predictor.hpp"
#include <iostream>
#include <iomanip>

int main() {
  using namespace traj_opt;

  // ============================================================
  // Test 1: Constant-velocity fallback (should match original code)
  // ============================================================
  {
    std::cout << "=== Test 1: CV fallback ===\n";
    TargetPredictor pred;
    Eigen::Vector3d car_p(0.5, 0.0, 1.0);
    Eigen::Vector3d car_v(2.0, 0.0, 0.0);
    Eigen::Vector3d normal(0.0, 0.0, 1.0);
    pred.seedConstantVelocity(car_p, car_v, normal, 0.0);
    pred.generatePrediction(3.0, 0.1);

    // Check: getPos(t) should equal car_p + car_v * t
    for (double t : {0.0, 0.5, 1.0, 2.0, 3.0}) {
      Eigen::Vector3d expected = car_p + car_v * t;
      Eigen::Vector3d actual = pred.getPos(t);
      double err = (expected - actual).norm();
      std::cout << "  t=" << t << "  pos=" << actual.transpose()
                << "  err=" << err << "\n";
    }
    std::cout << "  normal(1.0) = " << pred.getNormal(1.0).transpose() << "\n\n";
  }

  // ============================================================
  // Test 2: CTRV forward propagation (straight line)
  // ============================================================
  {
    std::cout << "=== Test 2: CTRV straight line ===\n";
    TargetPredictor pred;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.001;
    pred.setMeasurementNoise(R);

    // Feed measurements: v=2 m/s along x for 2 seconds
    for (int i = 0; i <= 20; ++i) {
      double t = i * 0.1;
      Eigen::Vector3d z(2.0 * t, 0.0, 0.0);
      pred.feedMeasurement(z, t);
    }
    std::cout << "  EKF state: " << pred.ekfState().transpose() << "\n";
    std::cout << "  v_h = " << pred.ekfState()(Ctrv::VH) << "\n";

    // Generate 3-second prediction
    pred.generatePrediction(3.0, 0.1);

    // The prediction should continue the straight line
    std::cout << "  Predictions:\n";
    for (double t : {0.0, 1.0, 2.0, 3.0}) {
      std::cout << "    t=" << t << "  pos=" << pred.getPos(t).transpose()
                << "  vel=" << pred.getVel(t).transpose() << "\n";
    }
    std::cout << "\n";
  }

  // ============================================================
  // Test 3: CTRV forward propagation (turning motion)
  // ============================================================
  {
    std::cout << "=== Test 3: CTRV turning motion ===\n";
    TargetPredictor pred;
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.001;
    pred.setMeasurementNoise(R);

    // Simulate a vehicle turning in a circle: ω=0.5 rad/s, v_h=2 m/s
    // Radius = v_h / ω = 4m
    double omega = 0.5;
    double vh = 2.0;
    double dt_sim = 0.05;
    for (int i = 0; i <= 100; ++i) {
      double t = i * dt_sim;
      double theta = omega * t;
      // Position on a circle of radius r = vh/omega, center at (0, r)
      double r = vh / omega;
      double x = r * std::sin(theta);
      double y = r * (1.0 - std::cos(theta));
      Eigen::Vector3d z(x, y, 0.0);
      pred.feedMeasurement(z, t);
    }

    std::cout << "  EKF state: " << pred.ekfState().transpose() << "\n";
    std::cout << "  v_h = " << pred.ekfState()(Ctrv::VH)
              << "  (expect ~" << vh << ")\n";
    std::cout << "  omega = " << pred.ekfState()(Ctrv::OMEGA)
              << "  (expect ~" << omega << ")\n";

    // Generate prediction and check it continues the arc
    pred.generatePrediction(2.0, 0.1);
    std::cout << "  Prediction curve:\n";
    for (double t = 0.0; t <= 2.0; t += 0.5) {
      Eigen::Vector3d p = pred.getPos(t);
      std::cout << "    t=" << t << "  pos=(" << std::fixed << std::setprecision(3)
                << p.x() << ", " << p.y() << ", " << p.z() << ")\n";
    }

    // Check covariance grows along prediction horizon
    double cov_start = pred.getCovariance(0.0).trace();
    double cov_end = pred.getCovariance(2.0).trace();
    std::cout << "  cov trace: t=0 → " << cov_start
              << "  t=2 → " << cov_end
              << "  (should grow)\n";
  }

  // ============================================================
  // Test 4: Surface normal rotation
  // ============================================================
  {
    std::cout << "\n=== Test 4: Surface normal ===\n";
    TargetPredictor pred;
    // Tilted platform: normal tilted 45° from vertical in body frame
    pred.setSurfaceNormal(Eigen::Vector3d(std::sin(M_PI / 4), 0, std::cos(M_PI / 4)));
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.001;
    pred.setMeasurementNoise(R);

    // Vehicle moving along x, so yaw ≈ 0 initially
    for (int i = 0; i <= 20; ++i) {
      double t = i * 0.1;
      pred.feedMeasurement(Eigen::Vector3d(2.0 * t, 0.0, 0.0), t);
    }

    pred.generatePrediction(1.0, 0.1);
    Eigen::Vector3d n = pred.getNormal(0.0);
    std::cout << "  yaw ≈ 0 → normal = " << n.transpose()
              << "  (expect ~[0.707, 0, 0.707])\n";
  }

  std::cout << "\nAll tests done.\n";
  return 0;
}
/*
This file is part of the package
https://github.com/mikelasa/force_control_impedance

Reference: Y. Hou and M. T. Mason, "Robust Execution of Contact-Rich Motion Plans by Hybrid Force-Velocity Control,"
           2019 International Conference on Robotics and Automation (ICRA), Montreal, QC, Canada, 2019, pp. 1933-1939

*/

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <force_control_impedance/impedance_controller.h>
#include <franka/robot_state.h>

#include <Eigen/QR>
#include <cmath>
#include <iostream>
#include <string>

using RUT::Matrix4d;
using RUT::Matrix6d;
using RUT::MatrixXd;
using RUT::Vector6d;
using RUT::Vector3d;
using RUT::Quaterniond;
using RUT::VectorXd;
using RUT::MatrixXd;

// Helper function to log Eigen vectors of length 7
void stream_vector7(std::ostream& os, const Eigen::Matrix<double, 7, 1>& vec) {
    for (int i = 0; i < 7; ++i) {
        os << vec(i) << "\t";
    }
}

static const Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[",
                          "]");

struct ImpedanceController::Implementation {
  Implementation();
  ~Implementation();
  bool initialize(
      const RUT::TimePoint& time_initial_0,
      const ImpedanceControllerConfig& impedance_controller_config);

  void setRobotStatus(const RUT::Vector7d& pose_WT,
                      const RUT::Vector6d& wrench_WT);
  void setRobotReference(const RUT::Vector7d& pose_WT,
                         const RUT::Vector6d& wrench_WTr);
  void setForceControlledAxis(const Matrix6d& Tr_new, int n_af);
  void setStiffnessMatrix(const Matrix6d& stiffness);
  void setDampingMatrix(const Matrix6d& damping);
  void setNullspaceStiffnessMatrix(const RUT::MatrixXd& stiffness);
  void setNullspaceDampingMatrix(const RUT::MatrixXd& damping);
  int step(RUT::Vector7d& pose_to_send);
  void reset();
  void logStates();
  void displayStates();
  void getJacobian(const Eigen::Matrix<double, 6, 7>& jacob);
  void getRobotState(franka::RobotState& state);

  ImpedanceControllerConfig config{};

  // internal controller states
  // poses and positions
  VectorXd pose_fb{7};
  Vector3d pose_fb_trans{3};
  VectorXd pose_ref{7};
  Vector3d pose_ref_trans{3};
  VectorXd pose_error{6};
  VectorXd joint_pos{7};
  Matrix4d end_effector_pose{};
  VectorXd joint_pos_null_desired{7};

  //velocities
  VectorXd joint_vel{7};

  // rotations
  Quaterniond error_quaternion{};
  Quaterniond pose_fb_rot{};
  Quaterniond pose_ref_rot{};

  //jacobian
  MatrixXd jacobian{};
  MatrixXd jacobian_pseud_inv{};
  MatrixXd null_jacobian{};

  //forces
  Vector6d wrench_T_fb{};
  Vector6d wrench_ref{};
  Vector6d external_wrench{};

  // torques
  VectorXd tau_task{7};
  VectorXd tau_nullspace{7};
  VectorXd tau_ext{7};
  VectorXd tau_d{7};

  // misc
  Matrix6d Tr{};
  RUT::Timer timer{};
  RUT::Profiler profiler{};
  std::ofstream log_file{};
};

ImpedanceController::Implementation::Implementation() {}

ImpedanceController::Implementation::~Implementation() {
  if (config.log_to_file)
    log_file.close();
}

bool ImpedanceController::Implementation::initialize(
    const RUT::TimePoint& time_initial_0,
    const ImpedanceControllerConfig& impedance_controller_config) {
  std::cout << "[ImpedanceController] Begin initialization.\n";
  config = impedance_controller_config;
  timer.tic(time_initial_0);

  reset();

  if (config.log_to_file) {
    log_file.open(config.log_file_path);
    if (log_file.is_open())
      std::cout << "[ImpedanceController] log file opened successfully at "
                << config.log_file_path << std::endl;
    else
      std::cerr << "[ImpedanceController] Failed to open log file at "
                << config.log_file_path << std::endl;
  }
  return true;
}

void ImpedanceController::Implementation::setRobotStatus(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WT) {
  
  //gets feedback
  pose_fb = pose_WT;
  wrench_T_fb = -wrench_WT;

  // split pose into position and quaternions
  pose_fb_trans = pose_fb.head(3);
  //todo: CONVENCIONES!
  Eigen::Quaterniond q;
  q.w() = pose_fb[3]; // index 3 = w
  q.x() = pose_fb[4]; // index 4 = x
  q.y() = pose_fb[5]; // index 5 = y
  q.z() = pose_fb[6]; // index 6 = z
  pose_fb_rot = q;

}

void ImpedanceController::Implementation::setRobotReference(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WTr) {
  //gets ref command
  pose_ref = pose_WT;
  wrench_ref = wrench_WTr;

  // split pose into position and quaternions
  pose_ref_trans = pose_ref.head(3);
  Eigen::Quaterniond q;
  q.w() = pose_ref[3]; // index 3 = w
  q.x() = pose_ref[4]; // index 4 = x
  q.y() = pose_ref[5]; // index 5 = y
  q.z() = pose_ref[6]; // index 6 = z
  pose_ref_rot = q;
}

// After axis update, the goal pose with offset should be equal to current pose
// in the new velocity controlled axes. To satisfy this requirement, we need to
// change SE3_TrefTadj accordingly
void ImpedanceController::Implementation::setForceControlledAxis(
    const Matrix6d& Tr_new, int n_af) {

  /*
  if (std::isnan(SE3_TrefTadj(0, 0))) {
    std::cerr << "\nThe computed offset has NaN." << std::endl;
    std::cerr << "SE3_WT:\n" << SE3_WT.format(MatlabFmt) << std::endl;
    std::cerr << "SE3_TTadj:\n" << SE3_TTadj.format(MatlabFmt) << std::endl;
    std::cerr << "spt_TTadj:\n" << spt_TTadj.format(MatlabFmt) << std::endl;
    std::cerr << "Jac_v_spt_inv:\n"
              << Jac_v_spt_inv.format(MatlabFmt) << std::endl;
    std::cerr << "Jac_v_spt:\n" << Jac_v_spt.format(MatlabFmt) << std::endl;
    std::cerr << "m_anni:\n" << m_anni.format(MatlabFmt) << std::endl;
    std::cerr << "spt_TTadj_new:\n"
              << spt_TTadj_new.format(MatlabFmt) << std::endl;
    std::cerr << "SE3_TrefTadj:\n"
              << SE3_TrefTadj.format(MatlabFmt) << std::endl;
    std::cerr << "\nNow paused at setForceControlledAxis()";
    getchar();
  }
    */
}

void ImpedanceController::Implementation::setStiffnessMatrix(
    const Matrix6d& stiffness) {
  config.compliance6d.stiffness = stiffness;
}

void ImpedanceController::Implementation::setDampingMatrix(
    const Matrix6d& damping) {
  config.compliance6d.damping = damping;
}

void ImpedanceController::Implementation::setNullspaceStiffnessMatrix(
    const MatrixXd& stiffness) {
  config.compliance6d.nullspace_stiffness = stiffness;
}

void ImpedanceController::Implementation::setNullspaceDampingMatrix(
    const MatrixXd& damping) {
  config.compliance6d.nullspace_damping = damping;
}

/*
Control step based on the sensorless cartesian impedance control:

  total torque is compute based on:
    - task torque
    - nullspace torque
    - external torque

*/
int ImpedanceController::Implementation::step(RUT::Vector7d& torque_to_send) {
  profiler.clear();
  profiler.start();
  timer.tic();
  // ----------------------------------------
  //  /* Position updates */
  // ----------------------------------------
  
  // compute pose translation error
  pose_error.head(3) << pose_fb_trans - pose_ref_trans;
  
  //check for quaternion discontinuity
  if (pose_ref_rot.dot(pose_fb_rot) < 0.0) {
  pose_fb_rot.coeffs() *= -1.0; // flip sign in-place
  }

  // Quaternion error: current * inverse(desired)
  const Eigen::Quaterniond error_quaternion(pose_fb_rot * pose_ref_rot.inverse());
  Eigen::AngleAxisd angle_axis(error_quaternion);
  pose_error.tail(3) << angle_axis.axis() * angle_axis.angle();
  //for plot purposes
   
  profiler.stop("1");
  profiler.start();

  // ----------------------------------------
  //  compute torques
  // ----------------------------------------

  // compute task torques (impedance model)
  tau_task << jacobian.transpose() * (-config.compliance6d.stiffness * pose_error - config.compliance6d.damping * (jacobian * joint_vel));
  // compute nullspace torques (joint space regulation)
  null_jacobian = MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_pseud_inv.transpose();

  //print debug
  tau_nullspace << null_jacobian * (- config.compliance6d.nullspace_stiffness * (joint_pos - joint_pos_null_desired) - config.compliance6d.nullspace_damping * joint_vel);
  // compute external torques (from external forces)
  tau_ext << jacobian.transpose() * wrench_T_fb;

  // compute all torques
  tau_d << tau_task  + tau_nullspace;// + tau_ext;

  profiler.stop("2");
  profiler.start();

  torque_to_send = tau_d;

  if (std::isnan(torque_to_send[0])) {
    std::cerr << "==================== pose is nan. =====================\n";
    displayStates();
    std::cerr << "Press ENTER to continue..." << std::endl;
    getchar();
    return false;
  }
  profiler.stop("4");
  profiler.start();
  if (config.log_to_file) {
    logStates();
  }
  profiler.stop("5");
  double timenow = timer.toc_ms();
  if (config.alert_overrun && timenow > config.dt * 1000.) {
    std::cerr << "impedanceController: step took too long: " << timenow << "ms"
              << std::endl;
    std::cerr << "Profiler: " << std::endl;
    profiler.show();
  }

  return true;
}

void ImpedanceController::Implementation::reset() {
  Tr = Matrix6d::Identity();
  pose_fb = RUT::Vector7d::Zero();
  pose_fb_trans = Vector3d::Zero();
  pose_fb_rot = Quaterniond::Identity();
  pose_ref = RUT::Vector7d::Zero();
  pose_ref_trans = Vector3d::Zero();
  pose_ref_rot = Quaterniond::Identity();
  pose_error = RUT::Vector6d::Zero();
  jacobian = MatrixXd::Zero(6, 7);
  jacobian_pseud_inv = MatrixXd::Zero(7, 6);
  tau_task = RUT::Vector7d::Zero();
  tau_nullspace = RUT::Vector7d::Zero();
  tau_ext = RUT::Vector7d::Zero();
  tau_d = RUT::Vector7d::Zero();
  error_quaternion = Quaterniond::Identity();


}

void ImpedanceController::Implementation::logStates() {
  log_file << timer.toc_ms() << " ";
  stream_vector7(log_file, pose_fb);
  stream_vector7(log_file, pose_ref);
  stream_vector7(log_file, joint_pos);
  stream_vector7(log_file, joint_vel);
  RUT::stream_array_in6d(log_file, wrench_T_fb);
  stream_vector7(log_file, tau_task);
  stream_vector7(log_file, tau_nullspace);
  stream_vector7(log_file, tau_ext);
  stream_vector7(log_file, tau_d);
  log_file << std::endl;
}

void ImpedanceController::Implementation::displayStates() {
  
  std::cout << "================= Parameters ================== " << std::endl;
  std::cout << "dt: " << config.dt << std::endl;
  std::cout << "log_to_file: " << config.log_to_file << std::endl;
  std::cout << "log_file_path: " << config.log_file_path << std::endl;
  std::cout << "compliance6d.stiffness: "
            << config.compliance6d.stiffness.format(MatlabFmt) << std::endl;
  std::cout << "compliance6d.damping: "
            << config.compliance6d.damping.format(MatlabFmt) << std::endl;
  std::cout << "compliance6d.nullspace_stiffness: "
            << config.compliance6d.nullspace_stiffness.format(MatlabFmt) << std::endl;
  std::cout << "compliance6d.nullspace_damping: "
            << config.compliance6d.nullspace_damping.format(MatlabFmt) << std::endl;
  std::cout << "compliance6d.stiction: "
            << config.compliance6d.stiction.format(MatlabFmt) << std::endl;
  /*
  std::cout << "================= Internal states ================== "
            << std::endl;
  std::cout << "Tr: " << Tr.format(MatlabFmt) << std::endl;
            */
}

void ImpedanceController::Implementation::getJacobian(const Eigen::Matrix<double, 6, 7>& jacob) {

  // get the jacobian
  jacobian = jacob;

  // compute the pseudoinverse of the jacobian 
  jacobian_pseud_inv = jacob.completeOrthogonalDecomposition().solve(Eigen::MatrixXd::Identity(6, 6));

}

void ImpedanceController::Implementation::getRobotState(franka::RobotState& state) {
  // extract each element and save into internal
  joint_pos = Eigen::Map<const Eigen::VectorXd>(state.q.data(), 7);
  joint_vel = Eigen::Map<const Eigen::VectorXd>(state.dq.data(), 7);
  end_effector_pose = Eigen::Matrix4d::Map(state.O_T_EE.data());
  external_wrench = Eigen::Map<const Eigen::VectorXd>(state.O_F_ext_hat_K.data(), 6);
  //... add other elements as needed
  // Initialize joint_pos_null_desired only once, keeping the initial position as the desired nullspace position
    static bool nullspace_initialized = false;
    if (!nullspace_initialized) {
        joint_pos_null_desired = joint_pos;
        nullspace_initialized = true;
    }

}


ImpedanceController::ImpedanceController()
    : m_impl{std::make_unique<Implementation>()} {}
ImpedanceController::~ImpedanceController() = default;
ImpedanceController::ImpedanceController(ImpedanceController&&) = default;

bool ImpedanceController::init(const RUT::TimePoint& time0,
                                const ImpedanceControllerConfig& config,
                                const RUT::Vector7d& pose_current) {
  m_impl->initialize(time0, config);

  setRobotStatus(pose_current, RUT::Vector6d::Zero());
  setRobotReference(pose_current, RUT::Vector6d::Zero());
  getJacobian(Eigen::Matrix<double, 6, 7>::Zero());

  RUT::Vector7d torque_out;
  step(torque_out);
  setForceControlledAxis(Matrix6d::Identity(), 0);

  std::cout << "[impedanceController] initialization is done." << std::endl;
  return true;
}

void ImpedanceController::setRobotStatus(const RUT::Vector7d& pose_WT,
                                          const RUT::Vector6d& wrench_WT) {
  m_impl->setRobotStatus(pose_WT, wrench_WT);
}

void ImpedanceController::setRobotReference(const RUT::Vector7d& pose_WT,
                                             const RUT::Vector6d& wrench_WTr) {
  m_impl->setRobotReference(pose_WT, wrench_WTr);
}

void ImpedanceController::setForceControlledAxis(const Matrix6d& Tr_new,
                                                  int n_af) {
  m_impl->setForceControlledAxis(Tr_new, n_af);
}

void ImpedanceController::setStiffnessMatrix(const Matrix6d& stiffness) {
  m_impl->setStiffnessMatrix(stiffness);
}

void ImpedanceController::setDampingMatrix(const Matrix6d& damping) {
  m_impl->setDampingMatrix(damping);
}

void ImpedanceController::setNullspaceStiffnessMatrix(const MatrixXd& stiffness) {
  m_impl->setNullspaceStiffnessMatrix(stiffness);
}

void ImpedanceController::setNullspaceDampingMatrix(const MatrixXd& damping) {
  m_impl->setNullspaceDampingMatrix(damping);
}

int ImpedanceController::step(RUT::Vector7d& pose_to_send) {
  return m_impl->step(pose_to_send);
}

void ImpedanceController::displayStates() {
  m_impl->displayStates();
}

void ImpedanceController::getJacobian(const Eigen::Matrix<double, 6, 7>& jacob) {
    m_impl->getJacobian(jacob);
}

void ImpedanceController::getRobotState(franka::RobotState& state) {
    m_impl->getRobotState(state);
}
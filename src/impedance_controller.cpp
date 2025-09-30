/*
This file is part of the package
https://github.com/mikelasa/force_control_impedance

Reference: Y. Hou and M. T. Mason, "Robust Execution of Contact-Rich Motion Plans by Hybrid Force-Velocity Control,"
           2019 International Conference on Robotics and Automation (ICRA), Montreal, QC, Canada, 2019, pp. 1933-1939

*/

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <force_control_impedance/impedance_controller.h>

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
  int step(RUT::Vector7d& pose_to_send);
  void reset();
  void logStates();
  void displayStates();
  void getJacobian(const Eigen::Matrix<double, 6, 7>& jacob);
  void getVelocity(const Eigen::Matrix<double, 7, 1>&  joint_vel);

  ImpedanceControllerConfig config{};

  // internal controller states
  // poses and positions
  VectorXd pose_fb{7};
  Vector3d pose_fb_trans{3};
  VectorXd pose_ref{7};
  Vector3d pose_ref_trans{3};
  VectorXd pose_error{6};
  VectorXd dq{7};

  // rotations
  Quaterniond error_quaternion{};
  Quaterniond pose_fb_rot{};
  Quaterniond pose_ref_rot{};

  //jacobian
  MatrixXd jacobian;
  MatrixXd jacobian_pseud_inv;

  //forces
  Vector6d wrench_T_fb{};
  Vector6d wrench_ref{};

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
  pose_fb_rot.coeffs() = pose_fb.tail(4);

}

void ImpedanceController::Implementation::setRobotReference(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WTr) {
  //gets ref command
  pose_ref = pose_WT;
  wrench_ref = wrench_WTr;

  // split pose into position and quaternions
  pose_ref_trans = pose_ref.head(3);
  pose_ref_rot.coeffs() = pose_ref.tail(4);
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
  
  //orientation error
  if (pose_ref_rot.coeffs().dot(pose_fb_rot.coeffs()) < 0.0) 
  {
    pose_fb_rot.coeffs() << -pose_fb_rot.coeffs();
  }
  
  //difference quaternion
  error_quaternion = pose_fb_rot.inverse() * pose_ref_rot;
  pose_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  //transform to angle axis
  Eigen::AngleAxisd angle_axis(error_quaternion);
  pose_error.tail(3) << angle_axis.axis() * angle_axis.angle();
   
  profiler.stop("1");
  profiler.start();

  // ----------------------------------------
  //  compute torques
  // ----------------------------------------

  // compute task torques (impedance model)
  // debug each element of the equation
  tau_task << jacobian.transpose() * (-config.compliance6d.stiffness * pose_error - config.compliance6d.damping * (jacobian * dq));
  tau_d << tau_task;

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
  wrench_T_fb = Vector6d::Zero();
  pose_fb = RUT::Vector7d::Zero();
  pose_fb_trans = Vector3d::Zero();
  pose_fb_rot = Quaterniond::Identity();
  pose_ref = RUT::Vector7d::Zero();
  pose_ref_trans = Vector3d::Zero();
  pose_ref_rot = Quaterniond::Identity();
  pose_error = RUT::Vector6d::Zero();
  dq = RUT::Vector7d::Zero();
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
  RUT::stream_array_in6d(log_file, pose_error);
  stream_vector7(log_file, dq);
  RUT::stream_array_in6d(log_file, wrench_T_fb);
  stream_vector7(log_file, tau_task);
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
  /*
  std::cout << "================= Internal states ================== "
            << std::endl;
  std::cout << "Tr: " << Tr.format(MatlabFmt) << std::endl;
  std::cout << "Tr_inv: " << Tr_inv.format(MatlabFmt) << std::endl;
  std::cout << "v_force_selection: " << v_force_selection.format(MatlabFmt)
            << std::endl;
  std::cout << "v_velocity_selection: "
            << v_velocity_selection.format(MatlabFmt) << std::endl;
  std::cout << "diag_force_selection: "
            << diag_force_selection.format(MatlabFmt) << std::endl;
  std::cout << "diag_velocity_selection: "
            << diag_velocity_selection.format(MatlabFmt) << std::endl;
  std::cout << "m_anni: " << m_anni.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WTref: " << SE3_WTref.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WT: " << SE3_WT.format(MatlabFmt) << std::endl;
  std::cout << "SE3_TrefTadj: " << SE3_TrefTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WTadj: " << SE3_WTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_TTadj: " << SE3_TTadj.format(MatlabFmt) << std::endl;
  std::cout << "SE3_WT_cmd: " << SE3_WT_cmd.format(MatlabFmt) << std::endl;
  std::cout << "spt_TTadj: " << spt_TTadj.format(MatlabFmt) << std::endl;
  std::cout << "spt_TTadj_new: " << spt_TTadj_new.format(MatlabFmt)
            << std::endl;
  std::cout << "Adj_WT: " << Adj_WT.format(MatlabFmt) << std::endl;
  std::cout << "Adj_TW: " << Adj_TW.format(MatlabFmt) << std::endl;
  std::cout << "Jac_v_spt: " << Jac_v_spt.format(MatlabFmt) << std::endl;
  std::cout << "Jac_v_spt_inv: " << Jac_v_spt_inv.format(MatlabFmt)
            << std::endl;
  std::cout << "v_spatial_WT: " << v_spatial_WT.format(MatlabFmt) << std::endl;
  std::cout << "v_body_WT: " << v_body_WT.format(MatlabFmt) << std::endl;
  std::cout << "v_body_WT_ref: " << v_body_WT_ref.format(MatlabFmt)
            << std::endl;
  std::cout << "v_Tr: " << v_Tr.format(MatlabFmt) << std::endl;
  std::cout << "vd_Tr: " << vd_Tr.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_Err_prev: " << wrench_T_Err_prev.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_Err_I: " << wrench_T_Err_I.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_fb: " << wrench_T_fb.format(MatlabFmt) << std::endl;
  std::cout << "wrench_Tr_cmd: " << wrench_Tr_cmd.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_T_spring: " << wrench_T_spring.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_spring: " << wrench_Tr_spring.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_fb: " << wrench_Tr_fb.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_cmd: " << wrench_T_cmd.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_Err: " << wrench_T_Err.format(MatlabFmt) << std::endl;
  std::cout << "wrench_T_PID: " << wrench_T_PID.format(MatlabFmt) << std::endl;
  std::cout << "wrench_Tr_PID: " << wrench_Tr_PID.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_Err: " << wrench_Tr_Err.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_damping: " << wrench_Tr_damping.format(MatlabFmt)
            << std::endl;
  std::cout << "wrench_Tr_All: " << wrench_Tr_All.format(MatlabFmt)
            << std::endl;
            */
}

void ImpedanceController::Implementation::getJacobian(const Eigen::Matrix<double, 6, 7>& jacob) {

  // get the jacobian
  jacobian = jacob;

  // compute the pseudoinverse of the jacobian 
  jacobian_pseud_inv = jacob.completeOrthogonalDecomposition().solve(Eigen::MatrixXd::Identity(6, 6));

}

void ImpedanceController::Implementation::getVelocity(const Eigen::Matrix<double, 7, 1>&  joint_vel) {

  dq = Eigen::VectorXd(joint_vel);

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

  RUT::Vector7d pose_out;
  step(pose_out);
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

int ImpedanceController::step(RUT::Vector7d& pose_to_send) {
  return m_impl->step(pose_to_send);
}

void ImpedanceController::displayStates() {
  m_impl->displayStates();
}

void ImpedanceController::getJacobian(const Eigen::Matrix<double, 6, 7>& jacob) {
    m_impl->getJacobian(jacob);
}

void ImpedanceController::getVelocity(const Eigen::Matrix<double, 7, 1>&  joint_vel) {
    m_impl->getVelocity(joint_vel);
}
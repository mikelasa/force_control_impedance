/**
 * IMPEDANCE CONTROLLER
 *
 * Purpose:
 *   Cartesian impedance controller implementation for robotic manipulators.
 *   Computes joint torques from pose error, velocity feedback, and optional
 *   nullspace regulation to achieve compliant behavior in task space.
 *
 * This file is part of the package:
 *   https://github.com/mikelasa/force_control_impedance
 *
 * Reference:
 *   Matthias Mayr, Julian M. Salt-Ducaju,
 *   "A C++ Implementation of a Cartesian Impedance Controller for Robotic Manipulators"
 *
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

/**
 * Helper function to log Eigen vectors of length 7 to output stream
 * @param os  Output stream
 * @param vec 7-element vector to log
 */
void stream_vector7(std::ostream& os, const Eigen::Matrix<double, 7, 1>& vec) {
    for (int i = 0; i < 7; ++i) {
        os << vec(i) << "\t";
    }
}

// MATLAB-style formatting for Eigen matrices
static const Eigen::IOFormat MatlabFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");

/**
 * Private implementation structure for ImpedanceController
 * Contains all internal state variables and control logic.
 *
 * Thread Safety:
 *   This class is not internally synchronized; callers must ensure
 *   thread-safe access (typically through controller mutex in the
 *   host application).
 */
struct ImpedanceController::Implementation {
    // ========== Constructor/Destructor ==========
    Implementation();
    ~Implementation();

    // ========== Initialization ==========
    bool initialize(const RUT::TimePoint& time_initial_0,
                   const ImpedanceControllerConfig& impedance_controller_config);

    // ========== Robot State Management ==========
    void setRobotStatus(const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WT);
    void setRobotReference(const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WTr);

    // ========== Control Mode Configuration ==========
    void setForceControlledAxis(const Matrix6d& Tr_new, int n_af);

    // ========== Impedance Parameter Configuration ==========
    void setStiffnessMatrix(const Matrix6d& stiffness);
    void setDampingMatrix(const Matrix6d& damping);
    void setNullspaceStiffnessMatrix(const RUT::MatrixXd& stiffness);
    void setNullspaceDampingMatrix(const RUT::MatrixXd& damping);

    // ========== Control Execution ==========
    int step(RUT::Vector7d& pose_to_send);

    // ========== State Management ==========
    void reset();

    // ========== Debugging and Monitoring ==========
    void logStates();
    void displayStates();

    // ========== Robot Interface ==========
    void getJacobian(const Eigen::Matrix<double, 6, 7>& jacob);
    void getRobotState(franka::RobotState& state);

    // ========== Configuration ==========
    ImpedanceControllerConfig config{};

    // ========== Internal Controller States ==========
    
    // Pose and position states
    VectorXd pose_fb{7};                    // Current pose feedback [x, y, z, qw, qx, qy, qz]
    Vector3d pose_fb_trans{3};              // Current position [x, y, z]
    VectorXd pose_ref{7};                   // Reference pose [x, y, z, qw, qx, qy, qz]
    Vector3d pose_ref_trans{3};             // Reference position [x, y, z]
    VectorXd pose_error{6};                 // Pose error [dx, dy, dz, drx, dry, drz]
    VectorXd joint_pos{7};                  // Current joint positions [rad]
    Matrix4d end_effector_pose{};           // End-effector transformation matrix
    VectorXd joint_pos_null_desired{7};     // Desired nullspace joint configuration [rad]

    // Velocity states
    VectorXd joint_vel{7};                  // Current joint velocities [rad/s]

    // Rotation states
    Quaterniond error_quaternion{};         // Orientation error quaternion
    Quaterniond pose_fb_rot{};              // Current orientation quaternion
    Quaterniond pose_ref_rot{};             // Reference orientation quaternion

    // Jacobian matrices
    MatrixXd jacobian{};                    // Robot Jacobian matrix (6x7)
    MatrixXd jacobian_pseud_inv{};          // Jacobian pseudo-inverse (7x6)
    MatrixXd null_jacobian{};               // Nullspace projection matrix (7x7)

    // Force and wrench states
    Vector6d wrench_T_fb{};                 // Current wrench feedback in tool frame [N, Nm]
    Vector6d wrench_ref{};                  // Reference wrench command [N, Nm]
    Vector6d external_wrench{};             // External wrench estimate [N, Nm]

    // Torque states
    VectorXd tau_task{7};                   // Task-space torques [Nm]
    VectorXd tau_nullspace{7};              // Nullspace torques [Nm]
    VectorXd tau_ext{7};                    // External torques [Nm]
    VectorXd tau_d{7};                      // Desired total torques [Nm]

    // Utility objects
    Matrix6d Tr{};                          // Force/position control selection matrix
    RUT::Timer timer{};                     // Timing utility
    RUT::Profiler profiler{};               // Performance profiling utility
    std::ofstream log_file{};               // Data logging file stream
};

// ========== Implementation Constructor/Destructor ==========

ImpedanceController::Implementation::Implementation() {}

ImpedanceController::Implementation::~Implementation() {
    if (config.log_to_file)
        log_file.close();
}

// ========== Implementation Initialization ==========

bool ImpedanceController::Implementation::initialize(
    const RUT::TimePoint& time_initial_0,
    const ImpedanceControllerConfig& impedance_controller_config) {

    std::cout << "[ImpedanceController] Begin initialization.\n";

    // ========================================================================
    // Step 1: Load configuration and start synchronized timer
    // ========================================================================
    config = impedance_controller_config;
    timer.tic(time_initial_0);

    // ========================================================================
    // Step 2: Reset all internal states to default values
    // ========================================================================
    reset();

    // ========================================================================
    // Step 3: Initialize data logging (optional)
    // ========================================================================
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

// ========== Robot State Management Implementation ==========

void ImpedanceController::Implementation::setRobotStatus(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WT) {

    // ========================================================================
    // Feedback acquisition (pose + wrench)
    // ========================================================================
    // Note: wrench is negated to match internal sign convention
    pose_fb = pose_WT;
    wrench_T_fb = -wrench_WT;

    // Split pose into translation and quaternion
    pose_fb_trans = pose_fb.head(3);

    // TODO: Confirm quaternion conventions (w, x, y, z)
    Eigen::Quaterniond q;
    q.w() = pose_fb[3]; // index 3 = w
    q.x() = pose_fb[4]; // index 4 = x
    q.y() = pose_fb[5]; // index 5 = y
    q.z() = pose_fb[6]; // index 6 = z
    pose_fb_rot = q;
}

void ImpedanceController::Implementation::setRobotReference(
    const RUT::Vector7d& pose_WT, const RUT::Vector6d& wrench_WTr) {

    // ========================================================================
    // Reference command update (pose + wrench)
    // ========================================================================
    pose_ref = pose_WT;
    wrench_ref = wrench_WTr;

    // Split pose into translation and quaternion
    pose_ref_trans = pose_ref.head(3);
    Eigen::Quaterniond q;
    q.w() = pose_ref[3]; // index 3 = w
    q.x() = pose_ref[4]; // index 4 = x
    q.y() = pose_ref[5]; // index 5 = y
    q.z() = pose_ref[6]; // index 6 = z
    pose_ref_rot = q;
}

// ========== Control Mode Configuration Implementation ==========

// After axis update, the goal pose with offset should be equal to current pose
// in the new velocity controlled axes. To satisfy this requirement, we need to
// change SE3_TrefTadj accordingly
void ImpedanceController::Implementation::setForceControlledAxis(
    const Matrix6d& Tr_new, int n_af) {

    /*
    // TODO: Implement axis selection logic
    // The commented code below shows debugging functionality
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

// ========== Impedance Parameter Configuration Implementation ==========

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

// ========== Control Execution Implementation ==========

/*
Control step based on the sensorless cartesian impedance control:

Total torque is computed based on:
  - task torque (impedance model)
  - nullspace torque (joint space regulation)
  - external torque (from external forces)
*/
int ImpedanceController::Implementation::step(RUT::Vector7d& torque_to_send) {
    profiler.clear();
    profiler.start();
    timer.tic();

    // ========================================================================
    // Phase 1: Compute pose error (translation + orientation)
    // ========================================================================
    // Translation error
    pose_error.head(3) << pose_fb_trans - pose_ref_trans;

    // Quaternion discontinuity check (ensure shortest rotation)
    if (pose_ref_rot.dot(pose_fb_rot) < 0.0) {
        pose_fb_rot.coeffs() *= -1.0; // Flip sign in-place
    }

    // Orientation error: current * inverse(desired)
    const Eigen::Quaterniond error_quaternion(pose_fb_rot * pose_ref_rot.inverse());
    Eigen::AngleAxisd angle_axis(error_quaternion);
    pose_error.tail(3) << angle_axis.axis() * angle_axis.angle();

    profiler.stop("1");
    profiler.start();

    // ========================================================================
    // Phase 2: Compute control torques
    // ========================================================================
    // Task-space impedance torque
    // tau_task = J^T * (-K_x * x_error - D_x * x_dot)
    tau_task << jacobian.transpose() *
        (-config.compliance6d.stiffness * pose_error -
         config.compliance6d.damping * (jacobian * joint_vel));

    // Nullspace torque (joint space regulation)
    // N = I - J^T * (J^T)^+
    null_jacobian = MatrixXd::Identity(7, 7) -
        jacobian.transpose() * jacobian_pseud_inv.transpose();

    tau_nullspace << null_jacobian *
        (-config.compliance6d.nullspace_stiffness *
             (joint_pos - joint_pos_null_desired) -
         config.compliance6d.nullspace_damping * joint_vel);

    // External torque (if enabled by user, currently not applied)
    tau_ext << jacobian.transpose() * wrench_ref;

    // Total desired torque
    tau_d << tau_task + tau_nullspace; // + tau_ext;

    profiler.stop("2");
    profiler.start();

    // ========================================================================
    // Phase 3: Output command and run safety checks
    // ========================================================================
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

    // ========================================================================
    // Phase 4: Optional logging and performance monitoring
    // ========================================================================
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

// ========== State Management Implementation ==========

void ImpedanceController::Implementation::reset() {
    // ========================================================================
    // Reset all internal states to safe defaults
    // ========================================================================
    Tr = Matrix6d::Identity();

    // Pose states
    pose_fb = RUT::Vector7d::Zero();
    pose_fb_trans = Vector3d::Zero();
    pose_fb_rot = Quaterniond::Identity();
    pose_ref = RUT::Vector7d::Zero();
    pose_ref_trans = Vector3d::Zero();
    pose_ref_rot = Quaterniond::Identity();
    pose_error = RUT::Vector6d::Zero();

    // Jacobians
    jacobian = MatrixXd::Zero(6, 7);
    jacobian_pseud_inv = MatrixXd::Zero(7, 6);

    // Torques
    tau_task = RUT::Vector7d::Zero();
    tau_nullspace = RUT::Vector7d::Zero();
    tau_ext = RUT::Vector7d::Zero();
    tau_d = RUT::Vector7d::Zero();

    // Orientation error
    error_quaternion = Quaterniond::Identity();
}

// ========== Debugging and Monitoring Implementation ==========

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

// ========== Robot Interface Implementation ==========

void ImpedanceController::Implementation::getJacobian(
    const Eigen::Matrix<double, 6, 7>& jacob) {
    // ========================================================================
    // Update Jacobian and compute pseudoinverse
    // ========================================================================
    jacobian = jacob;

    // Complete orthogonal decomposition provides a numerically stable pseudo-inverse
    jacobian_pseud_inv =
        jacob.completeOrthogonalDecomposition().solve(Eigen::MatrixXd::Identity(6, 6));
}

void ImpedanceController::Implementation::getRobotState(franka::RobotState& state) {
    // ========================================================================
    // Extract relevant robot state fields
    // ========================================================================
    joint_pos = Eigen::Map<const Eigen::VectorXd>(state.q.data(), 7);
    joint_vel = Eigen::Map<const Eigen::VectorXd>(state.dq.data(), 7);
    end_effector_pose = Eigen::Matrix4d::Map(state.O_T_EE.data());
    external_wrench = Eigen::Map<const Eigen::VectorXd>(state.O_F_ext_hat_K.data(), 6);
    // ... add other elements as needed

    // Initialize desired nullspace posture only once (first read)
    static bool nullspace_initialized = false;
    if (!nullspace_initialized) {
        joint_pos_null_desired = joint_pos;
        nullspace_initialized = true;
    }
}

// ========== ImpedanceController Public Interface Implementation ==========

ImpedanceController::ImpedanceController()
    : m_impl{std::make_unique<Implementation>()} {}

ImpedanceController::~ImpedanceController() = default;

ImpedanceController::ImpedanceController(ImpedanceController&&) = default;

bool ImpedanceController::init(const RUT::TimePoint& time0,
                                const ImpedanceControllerConfig& config,
                                const RUT::Vector7d& pose_current) {
    // Initialize the implementation
    m_impl->initialize(time0, config);

    // Set initial robot state
    setRobotStatus(pose_current, RUT::Vector6d::Zero());
    setRobotReference(pose_current, RUT::Vector6d::Zero());
    getJacobian(Eigen::Matrix<double, 6, 7>::Zero());

    // Perform initial control step
    RUT::Vector7d torque_out;
    step(torque_out);
    
    // Set initial control mode (all position control)
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
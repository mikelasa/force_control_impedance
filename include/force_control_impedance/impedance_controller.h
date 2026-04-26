/**
 * IMPEDANCE CONTROLLER
 * 
 * This file is part of the package:
 * https://github.com/mikelasa/force_control_impedance
 *
 * Reference: Matthias Mayr, Julian M. Salt-Ducaju, "A C++ Implementation of a Cartesian Impedance Controller for Robotic Manipulators"
 *
 */

#pragma once
#ifndef _IMPEDANCE_CONTROLLER_H_
#define _IMPEDANCE_CONTROLLER_H_

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <franka/robot_state.h>

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <fstream>

class ImpedanceController {
 public:
  /**
   * Configuration structure for ImpedanceController initialization
   * Contains all necessary parameters for controller setup and operation
   */
  struct ImpedanceControllerConfig {
    double dt{0.001};                                        // Integration/differentiation time step (s)
    bool log_to_file{false};                                 // Enable data logging to file
    std::string log_file_path{""};                           // Path for log file output
    int log_max_samples{600000};                             // Pre-allocated log buffer size (600000 = 10 min at 1kHz)
    bool alert_overrun{false};                               // Print warning when step() exceeds time limit

    /**
     * 6DOF compliance parameters for force/position control
     * Defines the mechanical impedance characteristics of the controller
     */
    struct ComplianceParameters6d {
      RUT::Matrix6d stiffness{};                             // Cartesian stiffness matrix [N/m, Nm/rad]
      RUT::Matrix6d damping{};                               // Cartesian damping matrix [Ns/m, Nms/rad]
      RUT::MatrixXd nullspace_stiffness{7, 7};               // Joint nullspace stiffness matrix [Nm/rad]
      RUT::MatrixXd nullspace_damping{7, 7};                 // Joint nullspace damping matrix [Nms/rad]
      RUT::Vector6d stiction{};                              // Static friction compensation [N, Nm]
    };
    ComplianceParameters6d compliance6d{};
  };

  // ========== Constructor/Destructor ==========
  
  ImpedanceController();
  ~ImpedanceController();
  ImpedanceController(ImpedanceController&&);

  // ========== Initialization ==========

  /**
   * Initialize the impedance controller
   * 
   * @param time0         Start time reference point for controller timing
   * @param config        Configuration structure containing all controller parameters
   * @param pose_current  Current robot pose (tool frame) [x, y, z, qw, qx, qy, qz] (mm, quaternion)
   * @return true if initialization successful, false otherwise
   */
  bool init(const RUT::TimePoint& time0,
            const ImpedanceControllerConfig& config,
            const RUT::Vector7d& pose_current);

  // ========== Robot State Management ==========

  /**
   * Update robot status with current measurements
   * 
   * @param pose_WT    Current tool frame pose in world frame [x, y, z, qw, qx, qy, qz] (mm, quaternion)
   * @param wrench_T   Tool wrench feedback in tool frame [fx, fy, fz, mx, my, mz] (N, Nm)
   */
  void setRobotStatus(const RUT::Vector7d& pose_WT,
                      const RUT::Vector6d& wrench_T);

  /**
   * Set position and force reference commands
   * 
   * @param pose_WT     Target tool pose in world frame [x, y, z, qw, qx, qy, qz] (mm, quaternion)
   * @param wrench_WTr  Target wrench in transformed frame [fx, fy, fz, mx, my, mz] (N, Nm)
   * 
   * @warning Always call step() after setRobotReference() and before setForceControlledAxis().
   *          step() updates internal states required for setForceControlledAxis() to work properly.
   */
  void setRobotReference(const RUT::Vector7d& pose_WT,
                         const RUT::Vector6d& wrench_WTr);

  // ========== Control Mode Configuration ==========

  /**
   * Configure which axes are force-controlled vs position-controlled
   * 
   * @param Tr    6x6 orthonormal transformation matrix defining axis directions
   * @param n_af  Number of force-controlled axes (0-6, remaining axes are position-controlled)
   */
  void setForceControlledAxis(const RUT::Matrix6d& Tr, int n_af);

  // ========== Impedance Parameter Configuration ==========

  /**
   * Set Cartesian stiffness matrix
   * 
   * @param stiffness  6x6 Cartesian stiffness matrix [N/m for translation, Nm/rad for rotation]
   */
  void setStiffnessMatrix(const RUT::Matrix6d& stiffness);

  /**
   * Set Cartesian damping matrix
   * 
   * @param damping  6x6 Cartesian damping matrix [Ns/m for translation, Nms/rad for rotation]
   */
  void setDampingMatrix(const RUT::Matrix6d& damping);

  /**
   * Set joint nullspace stiffness matrix
   * 
   * @param stiffness  7x7 joint stiffness matrix for nullspace behavior [Nm/rad]
   */
  void setNullspaceStiffnessMatrix(const RUT::MatrixXd& stiffness);

  /**
   * Set joint nullspace damping matrix
   * 
   * @param damping  7x7 joint damping matrix for nullspace behavior [Nms/rad]
   */
  void setNullspaceDampingMatrix(const RUT::MatrixXd& damping);

  // ========== Control Execution ==========

  /**
   * Execute one control step and compute output pose
   * 
   * @param pose  Output target pose [x, y, z, qw, qx, qy, qz] (mm, quaternion)
   * @return 0 if successful, error code otherwise
   */
  int step(RUT::Vector7d& pose);

  // ========== State Management ==========

  /**
   * Reset all internal states to default values
   * 
   * Recommended to call when robot starts from complete stop or when switching
   * from trajectory following to reactive control. Resets position offsets and
   * force errors to zero. Call setRobotReference() immediately after reset().
   */
  void reset();

  // ========== Debugging and Monitoring ==========

  /**
   * Flush the in-memory log buffer to disk immediately.
   * Call this before the program exits (e.g. on Ctrl+C) to ensure data is saved.
   * Safe to call multiple times; subsequent calls after the first are no-ops.
   */
  void flushLog();

  /**
   * Display current controller states to console
   * Useful for debugging and monitoring controller performance
   */
  void displayStates();

  // ========== Robot Interface ==========

  /**
   * Update robot Jacobian matrix
   * 
   * @param jacob  6x7 robot Jacobian matrix relating joint velocities to end-effector velocity
   */
  void getJacobian(const Eigen::Matrix<double, 6, 7>& jacob);

  /**
   * Update robot state information
   * 
   * @param state  Current Franka robot state containing joint positions, velocities, torques, etc.
   */
  void getRobotState(franka::RobotState& state);

 private:
  // Pimpl idiom: Private implementation pointer
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif  // _IMPEDANCE_CONTROLLER_H_
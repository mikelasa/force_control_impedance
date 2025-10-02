/*
This file is part of the package
https://github.com/mikelasa/force_control_impedance

Reference: Y. Hou and M. T. Mason, "Robust Execution of Contact-Rich Motion Plans by Hybrid Force-Velocity Control,"
           2019 International Conference on Robotics and Automation (ICRA), Montreal, QC, Canada, 2019, pp. 1933-1939

*/

#pragma once
#ifndef _IMPEDANCE_CONROLLER_H_
#define _IMPEDANCE_CONROLLER_H_

#include <RobotUtilities/spatial_utilities.h>
#include <RobotUtilities/timer_linux.h>
#include <franka/robot_state.h>

#include <Eigen/Geometry>
#include <chrono>
#include <deque>
#include <fstream>

class ImpedanceController {
 public:
  struct ImpedanceControllerConfig {
    double dt{0.001};  // used for integration/differentiation
    bool log_to_file{false};
    std::string log_file_path{""};
    bool alert_overrun{false};  // if true, print warning when step() takes too long

    struct ComplianceParameters6d {
      // Admittance parameters
      RUT::Matrix6d stiffness{};
      RUT::Matrix6d damping{};
      RUT::MatrixXd nullspace_stiffness{7, 7};
      RUT::MatrixXd nullspace_damping{7, 7};
      RUT::Vector6d stiction{};  // static friction, eliminates drifting
    };
    ComplianceParameters6d compliance6d{};
  };

  ImpedanceController();
  ~ImpedanceController();
  ImpedanceController(ImpedanceController&&);

  /**
   * @brief      initialize the controller.
   *
   * @param[in]  time0         The time point to start ticking from.
   * @param[in]  config        The struct contains all configs.
   * @param[in]  pose_current  The current robot pose (tool frame)
   *
   * @return     True if successfully initialized.
   */
  bool init(const RUT::TimePoint& time0,
            const ImpedanceControllerConfig& config,
            const RUT::Vector7d& pose_current);

  /**
   * @brief      Sets the robot status.
   *
   * @param[in]  pose_WT    The current tool frame pose in the world frame.
   * @param[in]  wrench_WT  The tool wrench feedback.
   */
  void setRobotStatus(const RUT::Vector7d& pose_WT,
                      const RUT::Vector6d& wrench_T);
  /**
   * @brief      Set the position and force reference (user command).
   *
   * @param[in] pose_WT     tool pose represented in the world frame.
   * @param[in] wrench_WTr  wrench measured in the transformed frame.
   *
   * WARNING remember to call step() after setRobotReference, before
   * setForceControlledAxis. step() will properly update internal states, which
   * is required for setForceControlledAxis to work properly.
   */
  void setRobotReference(const RUT::Vector7d& pose_WT,
                         const RUT::Vector6d& wrench_WTr);
  /**
   * @brief      Sets the force controlled axis.
   *
   * @param[in]  Tr    6x6 orthonormal matrix. Describes the axis direction.
   * @param[in]  n_af  The number of force controlled axes.
   */
  void setForceControlledAxis(const RUT::Matrix6d& Tr, int n_af);

  /**
   * @brief      Sets the stiffness matrix.
   *
   * @param[in]  stiffness  The stiffness matrix.
   */
  void setStiffnessMatrix(const RUT::Matrix6d& stiffness);

  /**
   * @brief      Sets the damping matrix.
   *
   * @param[in]  damping  The damping matrix.
   */
  void setDampingMatrix(const RUT::Matrix6d& damping);

  /**
   * @brief      Sets the stiffness matrix.
   *
   * @param[in]  stiffness  The stiffness matrix.
   */
  void setNullspaceStiffnessMatrix(const RUT::MatrixXd& stiffness);

  /**
   * @brief      Sets the damping matrix.
   *
   * @param[in]  damping  The damping matrix.
   */
  void setNullspaceDampingMatrix(const RUT::MatrixXd& damping);

  /**
   * @brief return true if no error.
   */
  int step(RUT::Vector7d& pose);

  /**
   * @brief      Reset all internal states to default. It is recommended to call
   * reset() everytime the robot starts from a complete stop in the air. This
   * includes setting all position offsets/force errors to zero. Call reset()
   * when the next action is computed based on the robot's current pose instead
   *  of being part of a pre-planned trajectory. After a reset(), call
   * setRobotReference() immediately.
   */
  void reset();

  /**
   * @brief      Print the current states to the console.
   */
  void displayStates();

  /**
   * @brief      Get robot Jacobian
   */
  void getJacobian(const Eigen::Matrix<double, 6, 7>& jacob);

  /**
   * @brief      Get robot state (franka::RobotState)
   */
  void getRobotState(franka::RobotState& state);

 private:
  struct Implementation;
  std::unique_ptr<Implementation> m_impl;
};

#endif  // _IMPEDANCE_CONROLLER_H_

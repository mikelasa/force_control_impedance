#include "force_control_impedance/config_deserialize_impedance.h"
#include <RobotUtilities/spatial_utilities.h>

#include <yaml-cpp/yaml.h>

#include "force_control_impedance/impedance_controller.h"

template <>
bool deserialize(const YAML::Node& node,
                 ImpedanceController::ImpedanceControllerConfig& config) {
  try {
    config.dt = node["dt"].as<double>();
    config.log_to_file = node["log_to_file"].as<bool>();
    config.log_file_path = node["log_file_path"].as<std::string>();
    config.alert_overrun = node["alert_overrun"].as<bool>();
    config.compliance6d.stiffness = RUT::deserialize_vector<RUT::Vector6d>(
                                        node["compliance6d"]["stiffness"])
                                        .asDiagonal();
    config.compliance6d.damping =
        RUT::deserialize_vector<RUT::Vector6d>(node["compliance6d"]["damping"])
            .asDiagonal();
    config.compliance6d.stiction = RUT::deserialize_vector<RUT::Vector6d>(
        node["compliance6d"]["stiction"]);

  } catch (const std::exception& e) {
    std::cerr << "Failed to load the config file: " << e.what() << std::endl;
    return false;
  }

  // validity check
  if (config.dt <= 0) {
    std::cerr << "Invalid dt: " << config.dt << std::endl;
    return false;
  }
  // diagonal elements of stiffness, damping, and inertia should be positive/non-negative
  if ((config.compliance6d.stiffness.diagonal().array() < 0).any()) {
    std::cerr << "Invalid compliance6d.stiffness: "
              << config.compliance6d.stiffness.diagonal().transpose()
              << ". All diagonal elements should be positive." << std::endl;
    return false;
  }
  if ((config.compliance6d.damping.diagonal().array() < 0).any()) {
    std::cerr << "Invalid compliance6d.damping: "
              << config.compliance6d.damping.diagonal().transpose()
              << ". All diagonal elements should be positive." << std::endl;
    return false;
  }
  // stiction should be non-negative
  if ((config.compliance6d.stiction.array() < 0).any()) {
    std::cerr << "Invalid compliance6d.stiction: "
              << config.compliance6d.stiction.transpose()
              << ". All elements should be non-negative." << std::endl;
    return false;
  }

  if ((config.compliance6d.stiction.array() > 0).any()) {
    std::cerr << "Invalid parameters. direct_force_control_gains must be all "
                 "zero if stiction is non-zero."
              << std::endl;
    return false;
  }

  return true;
}
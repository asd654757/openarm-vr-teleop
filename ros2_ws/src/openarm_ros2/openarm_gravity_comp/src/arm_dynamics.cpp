// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include "openarm_gravity_comp/arm_dynamics.hpp"

#include <urdf_parser/urdf_parser.h>

#include <cmath>
#include <utility>

#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace openarm_gravity_comp
{

ArmDynamics::ArmDynamics(
  std::string robot_description, std::string base_link,
  std::string tip_link)
: robot_description_(std::move(robot_description)),
  base_link_(std::move(base_link)),
  tip_link_(std::move(tip_link)) {}

bool ArmDynamics::initialize(std::string & error)
{
  const auto urdf_model = urdf::parseURDF(robot_description_);
  if (!urdf_model) {
    error = "failed to parse robot_description";
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(*urdf_model, tree)) {
    error = "failed to create KDL tree";
    return false;
  }
  if (!tree.getChain(base_link_, tip_link_, chain_)) {
    error = "failed to create KDL chain from " + base_link_ + " to " + tip_link_;
    return false;
  }

  joint_names_.clear();
  for (const auto & segment : chain_.segments) {
    const auto & joint = segment.getJoint();
    if (joint.getType() != KDL::Joint::None) {
      joint_names_.push_back(joint.getName());
    }
  }
  if (joint_names_.size() != 7U) {
    error = "expected 7 movable joints in chain, got " +
      std::to_string(joint_names_.size());
    return false;
  }

  // The chain starts at openarm_body_link0, whose axes match the world frame.
  // The fixed +/-90 degree arm mounts are part of the KDL chain.
  solver_ = std::make_unique<KDL::ChainDynParam>(
    chain_, KDL::Vector(0.0, 0.0, -9.80665));
  return true;
}

bool ArmDynamics::compute_gravity(
  const std::vector<double> & positions,
  std::vector<double> & efforts,
  std::string & error)
{
  if (!solver_) {
    error = "dynamics solver is not initialized";
    return false;
  }
  if (positions.size() != joint_names_.size()) {
    error = "joint position size does not match KDL chain";
    return false;
  }

  KDL::JntArray q(positions.size());
  KDL::JntArray gravity(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    if (!std::isfinite(positions[i])) {
      error = "joint position is not finite";
      return false;
    }
    q(i) = positions[i];
  }

  const int result = solver_->JntToGravity(q, gravity);
  if (result < 0) {
    error = "KDL JntToGravity failed with code " + std::to_string(result);
    return false;
  }

  efforts.resize(positions.size());
  for (std::size_t i = 0; i < positions.size(); ++i) {
    efforts[i] = gravity(i);
    if (!std::isfinite(efforts[i])) {
      error = "computed gravity effort is not finite";
      return false;
    }
  }
  return true;
}

}  // namespace openarm_gravity_comp
